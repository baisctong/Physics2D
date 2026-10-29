// Week 7 P13 — island-grouped solving is BIT-IDENTICAL to the flat Week 6 solve.
//
// The hard acceptance criterion: given the same initial state, installing the
// canonical island partition must not change a single float of the solved body
// velocities or of the persistent contact impulses. The reason it holds:
// constraints from different islands touch disjoint body sets, so their
// Gauss-Seidel updates commute; the canonical partition preserves the relative
// contact order inside each island. The tests below compare the raw IEEE-754 bit
// patterns (std::bit_cast), not tolerances.
//
// Coverage:
//  - velocity layer: Prepare + WarmStart + SolveVelocity, 1/3/8 iterations, with
//    and without warm starting, two islands sharing one static ground (the static
//    boundary must not couple them);
//  - sleeping island: skipped in both modes, its velocities AND persistent contact
//    impulses stay bit-identical; the awake island is unaffected;
//  - installed-but-empty partition = flat mode; HasIslandPartition/Clear semantics;
//  - position layer (split impulse): grouped vs flat transforms bit-identical and
//    sleeping islands are never pushed;
//  - a hand-computed contact response matches all modes exactly;
//  - THREE solve modes are bit-identical everywhere: flat, grouped-direct
//    (SolverSettings::packIslandBodies == false, the measured/declared default)
//    and grouped-packed (opt-in for very large scenes). The dense 400-body island
//    is checked explicitly because it is the scene where packing lost;
//  - a three-mode SolveVelocity timing print (numbers depend on the build, so the
//    timing test only prints - Release numbers are the quotable ones).

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/IDPool.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/Vec2.h"
#include "physics2d/Solver/ContactSolver.h"
#include "physics2d/Solver/Island.h"

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::ContactSolver;
using p2::IDPool;
using p2::SolverSettings;
using p2::Vec2;

constexpr float kDt = p2::kFixedTimeStep;

[[nodiscard]] uint32_t FloatBits(float value) noexcept {
    return std::bit_cast<uint32_t>(value);
}

Body MakeBody(int32_t index, BodyType type, Vec2 position, Vec2 velocity, float angularVelocity) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.linearVelocity = velocity;
    def.angularVelocity = angularVelocity;
    def.mass = 1.0f;
    def.inertia = 1.0f;
    Body body;
    body.Initialize(def);
    body.handle = BodyHandle{index, 1u};
    body.isAwake = (type == BodyType::Dynamic); // static bodies are boundaries/asleep
    return body;
}

struct SolverScene {
    std::vector<Body> bodies;
    std::vector<Contact> contacts;
    std::vector<std::vector<int32_t>> islandBodies;
    std::vector<std::vector<int32_t>> islandContacts;
};

// One point contact. `worldAnchor` is stored in both bodies' local frames (the
// narrow-phase invariant), and the inherited impulses simulate a persistent
// contact from the previous frame.
Contact MakePointContact(const Body& bodyA, const Body& bodyB, Vec2 worldAnchor, Vec2 normal,
                         float penetration, float friction, float inheritedNormalImpulse,
                         float inheritedTangentImpulse) {
    Contact contact;
    contact.bodyA = bodyA.handle;
    contact.bodyB = bodyB.handle;
    contact.manifold.normal = normal;
    contact.manifold.penetration = penetration;
    contact.manifold.pointCount = 1;
    p2::ContactPoint& point = contact.manifold.points[0];
    point.localPointA = worldAnchor - bodyA.xf.p; // identity rotations in these fixtures
    point.localPointB = worldAnchor - bodyB.xf.p;
    point.normalImpulse = inheritedNormalImpulse;
    point.tangentImpulse = inheritedTangentImpulse;
    point.id.type = p2::kContactTypeCircleCircle;
    contact.friction = friction;
    contact.restitution = 0.0f;
    contact.isPersistent = true;
    return contact;
}

// Two independent islands on one shared static ground: {A,B} at x ~ 0.5 and
// {C,D} at x ~ 5.5. The shared ground is the crucial boundary case: it must not
// couple the islands.
SolverScene MakeTwoIslandScene() {
    SolverScene scene;
    IDPool ids;
    const auto addBody = [&](BodyType type, Vec2 position, Vec2 velocity, float angularVelocity) {
        const int32_t index = ids.Allocate();
        if (static_cast<size_t>(index) >= scene.bodies.size()) {
            scene.bodies.resize(static_cast<size_t>(index) + 1);
        }
        scene.bodies[static_cast<size_t>(index)] = MakeBody(index, type, position, velocity, angularVelocity);
        return index;
    };

    const int32_t ground = addBody(BodyType::Static, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f);   // 0
    const int32_t boxA = addBody(BodyType::Dynamic, Vec2{0.5f, 0.5f}, Vec2{0.4f, -0.2f}, 0.3f);   // 1
    const int32_t boxB = addBody(BodyType::Dynamic, Vec2{0.5f, 1.5f}, Vec2{-0.1f, 0.3f}, -0.2f);  // 2
    const int32_t boxC = addBody(BodyType::Dynamic, Vec2{5.5f, 0.5f}, Vec2{0.2f, 0.1f}, 0.15f);   // 3
    const int32_t boxD = addBody(BodyType::Dynamic, Vec2{5.5f, 1.5f}, Vec2{0.6f, -0.5f}, -0.25f); // 4

    scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(ground)],
                                              scene.bodies[static_cast<size_t>(boxA)], Vec2{0.5f, 0.0f},
                                              Vec2{0.0f, 1.0f}, 0.012f, 0.6f, 2.0f, 0.4f));
    scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(boxA)],
                                              scene.bodies[static_cast<size_t>(boxB)], Vec2{0.5f, 1.0f},
                                              Vec2{0.0f, 1.0f}, 0.008f, 0.6f, 1.0f, -0.2f));
    scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(ground)],
                                              scene.bodies[static_cast<size_t>(boxC)], Vec2{5.5f, 0.0f},
                                              Vec2{0.0f, 1.0f}, 0.015f, 0.6f, 1.5f, 0.1f));
    scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(boxC)],
                                              scene.bodies[static_cast<size_t>(boxD)], Vec2{5.5f, 1.0f},
                                              Vec2{0.0f, 1.0f}, 0.006f, 0.6f, 0.8f, 0.3f));

    p2::BuildIslandPartition(scene.bodies, ids, scene.contacts, scene.islandBodies, scene.islandContacts);
    return scene;
}

struct BitSnapshot {
    std::vector<uint32_t> bodyBits;      // per body: vx, vy, w
    std::vector<uint32_t> impulseBits;   // per contact point: accN, accT
    std::vector<uint32_t> transformBits; // per body: px, py, qc, qs
};

BitSnapshot TakeSnapshot(const SolverScene& scene) {
    BitSnapshot snapshot;
    for (const Body& body : scene.bodies) {
        if (!body.handle.IsValid()) {
            continue;
        }
        snapshot.bodyBits.push_back(FloatBits(body.linearVelocity.x));
        snapshot.bodyBits.push_back(FloatBits(body.linearVelocity.y));
        snapshot.bodyBits.push_back(FloatBits(body.angularVelocity));
        snapshot.transformBits.push_back(FloatBits(body.xf.p.x));
        snapshot.transformBits.push_back(FloatBits(body.xf.p.y));
        snapshot.transformBits.push_back(FloatBits(body.xf.q.c));
        snapshot.transformBits.push_back(FloatBits(body.xf.q.s));
    }
    for (const Contact& contact : scene.contacts) {
        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            snapshot.impulseBits.push_back(FloatBits(contact.manifold.points[pointIndex].normalImpulse));
            snapshot.impulseBits.push_back(FloatBits(contact.manifold.points[pointIndex].tangentImpulse));
        }
    }
    return snapshot;
}

// Returns the number of differing bit patterns; prints nothing (the caller
// reports the total float count to the devlog).
size_t CountDifferences(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b) {
    if (a.size() != b.size()) {
        return std::max(a.size(), b.size());
    }
    size_t differences = 0;
    for (size_t index = 0; index < a.size(); ++index) {
        if (a[index] != b[index]) {
            ++differences;
        }
    }
    return differences;
}

struct SolveResult {
    SolverScene scene;
    int32_t constraintCount = 0;
    BitSnapshot snapshot;
};

// Week 7 supports three interchangeable solving variants that must all produce the
// same bits:
//  - Flat: no partition, global contact order (Week 6 behaviour);
//  - GroupedDirect: canonical island order, solving straight on the Body array -
//    the measured/declared default (SolverSettings::packIslandBodies == false);
//  - GroupedPacked: canonical island order, hot state copied into a contiguous
//    scratch array (opt-in for very large scenes).
enum class SolveMode { Flat, GroupedDirect, GroupedPacked };

SolveResult RunVelocitySolve(const SolverScene& source, SolveMode mode, int32_t iterations, bool warmStart) {
    SolveResult result;
    result.scene = source;
    SolverSettings settings;
    settings.packIslandBodies = (mode == SolveMode::GroupedPacked);
    ContactSolver solver;
    if (mode != SolveMode::Flat) {
        solver.SetIslandPartition(&result.scene.islandBodies, &result.scene.islandContacts);
    }
    solver.Prepare(result.scene.bodies, result.scene.contacts, settings);
    if (warmStart) {
        solver.WarmStart(result.scene.bodies, result.scene.contacts, settings);
    }
    solver.SolveVelocity(result.scene.bodies, result.scene.contacts, iterations, settings);
    result.constraintCount = solver.GetConstraintCount();
    result.snapshot = TakeSnapshot(result.scene);
    return result;
}

SolveResult RunPositionLayer(const SolverScene& source, bool grouped) {
    SolveResult result;
    result.scene = source;
    SolverSettings settings;
    ContactSolver solver;
    if (grouped) {
        solver.SetIslandPartition(&result.scene.islandBodies, &result.scene.islandContacts);
    }
    solver.SolvePosition(result.scene.bodies, result.scene.contacts, kDt, settings.positionIterations,
                         settings);
    result.snapshot = TakeSnapshot(result.scene);
    return result;
}

// Applies the sleeping-island fixture: island 1 (C,D) is fully asleep, island 0
// stays awake. Whole-island atomicity is the precondition (World::UpdateSleep does
// this); the solver must skip the sleeping island.
SolverScene MakeSceneWithSleepingSecondIsland() {
    SolverScene scene = MakeTwoIslandScene();
    for (const int32_t bodyIndex : scene.islandBodies[1]) {
        scene.bodies[static_cast<size_t>(bodyIndex)].isAwake = false;
    }
    return scene;
}

constexpr int32_t kGridWidth = 20;
constexpr int32_t kGridHeight = 20;

// One large island: a 20x20 grid of dynamic bodies connected to their right and
// upper neighbours (760 one-point contacts). This is the scene on which the packed
// variant measured slower than reading the Body array directly (Week 7 benchmark),
// so it is also the stress case for "all three solve modes are bit-identical".
SolverScene MakeLargeGridScene() {
    SolverScene scene;
    IDPool ids;
    const int32_t bodyCount = kGridWidth * kGridHeight;
    scene.bodies.resize(static_cast<size_t>(bodyCount));
    for (int32_t row = 0; row < kGridHeight; ++row) {
        for (int32_t column = 0; column < kGridWidth; ++column) {
            const int32_t bodyIndex = row * kGridWidth + column;
            const int32_t slot = ids.Allocate();
            scene.bodies[static_cast<size_t>(bodyIndex)] =
                MakeBody(slot, BodyType::Dynamic,
                         Vec2{0.9f * static_cast<float>(column), 0.9f * static_cast<float>(row)},
                         Vec2{0.01f * static_cast<float>(row), -0.01f * static_cast<float>(column)}, 0.001f);
        }
    }
    for (int32_t row = 0; row < kGridHeight; ++row) {
        for (int32_t column = 0; column < kGridWidth; ++column) {
            const int32_t bodyIndex = row * kGridWidth + column;
            const Vec2 center = scene.bodies[static_cast<size_t>(bodyIndex)].xf.p;
            if (column + 1 < kGridWidth) {
                const int32_t neighbour = row * kGridWidth + column + 1;
                const Vec2 neighbourCenter = scene.bodies[static_cast<size_t>(neighbour)].xf.p;
                scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(bodyIndex)],
                                                          scene.bodies[static_cast<size_t>(neighbour)],
                                                          (center + neighbourCenter) * 0.5f, Vec2{1.0f, 0.0f},
                                                          0.001f, 0.5f, 0.0f, 0.0f));
            }
            if (row + 1 < kGridHeight) {
                const int32_t neighbour = (row + 1) * kGridWidth + column;
                const Vec2 neighbourCenter = scene.bodies[static_cast<size_t>(neighbour)].xf.p;
                scene.contacts.push_back(MakePointContact(scene.bodies[static_cast<size_t>(bodyIndex)],
                                                          scene.bodies[static_cast<size_t>(neighbour)],
                                                          (center + neighbourCenter) * 0.5f, Vec2{0.0f, 1.0f},
                                                          0.001f, 0.5f, 0.0f, 0.0f));
            }
        }
    }
    p2::BuildIslandPartition(scene.bodies, ids, scene.contacts, scene.islandBodies, scene.islandContacts);
    return scene;
}

} // namespace

// ---------------------------------------------------------------------------
// Velocity layer: grouped == flat, bit for bit.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, GroupedVelocitySolveIsBitIdenticalToFlatSolve) {
    const SolverScene source = MakeTwoIslandScene();
    ASSERT_EQ(source.islandBodies.size(), 2u) << "fixture must produce two islands";
    ASSERT_FALSE(SolverSettings{}.packIslandBodies)
        << "the shipped default must be the direct grouped variant (packing measured slower at this scale)";

    const int32_t iterationCounts[] = {1, 3, 8};
    size_t comparedFloats = 0;
    for (const int32_t iterations : iterationCounts) {
        for (const bool warmStart : {false, true}) {
            const SolveResult flat = RunVelocitySolve(source, SolveMode::Flat, iterations, warmStart);
            const SolveResult direct =
                RunVelocitySolve(source, SolveMode::GroupedDirect, iterations, warmStart);
            const SolveResult packed =
                RunVelocitySolve(source, SolveMode::GroupedPacked, iterations, warmStart);

            EXPECT_EQ(flat.constraintCount, direct.constraintCount) << "iterations=" << iterations;
            EXPECT_EQ(flat.constraintCount, packed.constraintCount) << "iterations=" << iterations;

            const size_t directBodyDifferences =
                CountDifferences(flat.snapshot.bodyBits, direct.snapshot.bodyBits);
            const size_t packedBodyDifferences =
                CountDifferences(flat.snapshot.bodyBits, packed.snapshot.bodyBits);
            const size_t directImpulseDifferences =
                CountDifferences(flat.snapshot.impulseBits, direct.snapshot.impulseBits);
            const size_t packedImpulseDifferences =
                CountDifferences(flat.snapshot.impulseBits, packed.snapshot.impulseBits);
            EXPECT_EQ(directBodyDifferences, 0u) << "iterations=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(packedBodyDifferences, 0u) << "iterations=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(directImpulseDifferences, 0u) << "iterations=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(packedImpulseDifferences, 0u) << "iterations=" << iterations << " warm=" << warmStart;

            comparedFloats += 2u * (flat.snapshot.bodyBits.size() + flat.snapshot.impulseBits.size());
            std::printf(
                "[solver7] velocity iters=%d warm=%d: flat/direct/packed %zu body floats + %zu contact "
                "impulses bit-identical\n",
                iterations, static_cast<int>(warmStart), flat.snapshot.bodyBits.size(),
                flat.snapshot.impulseBits.size());
        }
    }
    std::printf("[solver7] velocity three-mode bit-identity total: %zu float comparisons, 0 differences\n",
                comparedFloats);
}

TEST(SolverIslandTest, GroupedAndFlatMatchAHandComputedContactResponse) {
    // Two equal dynamic boxes head-on, no friction, e = 0, normal (1, 0):
    //   vRel.n = 0 - 2 = -2, K_n = 1 + 1 = 2 -> normalMass = 0.5
    //   lambda = -(-2 + 0) * 0.5 = 1 -> both bodies end at the common velocity 1.
    SolverScene scene;
    IDPool ids;
    scene.bodies.resize(2);
    scene.bodies[0] = MakeBody(ids.Allocate(), BodyType::Dynamic, Vec2{-0.5f, 0.0f}, Vec2{2.0f, 0.0f}, 0.0f);
    scene.bodies[1] = MakeBody(ids.Allocate(), BodyType::Dynamic, Vec2{0.5f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f);
    scene.contacts.push_back(MakePointContact(scene.bodies[0], scene.bodies[1], Vec2{0.0f, 0.0f},
                                              Vec2{1.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.0f));
    p2::BuildIslandPartition(scene.bodies, ids, scene.contacts, scene.islandBodies, scene.islandContacts);

    const SolveResult flat = RunVelocitySolve(scene, SolveMode::Flat, 1, /*warmStart=*/true);
    const SolveResult direct = RunVelocitySolve(scene, SolveMode::GroupedDirect, 1, /*warmStart=*/true);
    const SolveResult packed = RunVelocitySolve(scene, SolveMode::GroupedPacked, 1, /*warmStart=*/true);
    EXPECT_FLOAT_EQ(flat.scene.bodies[0].linearVelocity.x, 1.0f);
    EXPECT_FLOAT_EQ(flat.scene.bodies[1].linearVelocity.x, 1.0f);
    EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, direct.snapshot.bodyBits), 0u);
    EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, packed.snapshot.bodyBits), 0u);
    std::printf("[solver7] hand-computed head-on: flat=(%.1f, %.1f) direct=(%.1f, %.1f) packed=(%.1f, %.1f) "
                "bit-identical\n",
                static_cast<double>(flat.scene.bodies[0].linearVelocity.x),
                static_cast<double>(flat.scene.bodies[1].linearVelocity.x),
                static_cast<double>(direct.scene.bodies[0].linearVelocity.x),
                static_cast<double>(direct.scene.bodies[1].linearVelocity.x),
                static_cast<double>(packed.scene.bodies[0].linearVelocity.x),
                static_cast<double>(packed.scene.bodies[1].linearVelocity.x));
}

// ---------------------------------------------------------------------------
// Sleeping island: skipped in every mode, contact accumulators preserved.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, SleepingIslandIsSkippedByAllModes) {
    const SolverScene awakeSource = MakeTwoIslandScene();
    const SolverScene sleepingSource = MakeSceneWithSleepingSecondIsland();
    const BitSnapshot sleepingInitial = TakeSnapshot(sleepingSource);

    const SolveResult flatAwake = RunVelocitySolve(awakeSource, SolveMode::Flat, 8, true);
    const SolveResult flatSleeping = RunVelocitySolve(sleepingSource, SolveMode::Flat, 8, true);
    const SolveResult directSleeping = RunVelocitySolve(sleepingSource, SolveMode::GroupedDirect, 8, true);
    const SolveResult packedSleeping = RunVelocitySolve(sleepingSource, SolveMode::GroupedPacked, 8, true);

    // All modes skip the sleeping island identically.
    EXPECT_EQ(CountDifferences(flatSleeping.snapshot.bodyBits, directSleeping.snapshot.bodyBits), 0u);
    EXPECT_EQ(CountDifferences(flatSleeping.snapshot.impulseBits, directSleeping.snapshot.impulseBits), 0u);
    EXPECT_EQ(CountDifferences(flatSleeping.snapshot.bodyBits, packedSleeping.snapshot.bodyBits), 0u);
    EXPECT_EQ(CountDifferences(flatSleeping.snapshot.impulseBits, packedSleeping.snapshot.impulseBits), 0u);
    // Only island 0's two contacts are solved (flat skips both-asleep pairs,
    // grouped skips the whole sleeping island).
    EXPECT_EQ(flatSleeping.constraintCount, 2);
    EXPECT_EQ(directSleeping.constraintCount, 2);
    EXPECT_EQ(packedSleeping.constraintCount, 2);

    // Sleeping bodies and their persistent contact impulses are untouched...
    for (const int32_t bodyIndex : sleepingSource.islandBodies[1]) {
        const Body& sleepingBody = flatSleeping.scene.bodies[static_cast<size_t>(bodyIndex)];
        const size_t bitBase = static_cast<size_t>(bodyIndex) * 3;
        EXPECT_EQ(FloatBits(sleepingBody.linearVelocity.x), sleepingInitial.bodyBits[bitBase + 0]);
        EXPECT_EQ(FloatBits(sleepingBody.linearVelocity.y), sleepingInitial.bodyBits[bitBase + 1]);
        EXPECT_EQ(FloatBits(sleepingBody.angularVelocity), sleepingInitial.bodyBits[bitBase + 2]);
    }
    for (size_t contactIndex = 2; contactIndex < 4; ++contactIndex) {
        EXPECT_EQ(FloatBits(flatSleeping.scene.contacts[contactIndex].manifold.points[0].normalImpulse),
                  FloatBits(sleepingSource.contacts[contactIndex].manifold.points[0].normalImpulse));
        EXPECT_EQ(FloatBits(flatSleeping.scene.contacts[contactIndex].manifold.points[0].tangentImpulse),
                  FloatBits(sleepingSource.contacts[contactIndex].manifold.points[0].tangentImpulse));
    }
    // The awake island is unaffected by the other island's sleep state (islands
    // are independent), so its velocities match the all-awake run bit for bit.
    for (const int32_t bodyIndex : awakeSource.islandBodies[0]) {
        for (int32_t component = 0; component < 3; ++component) {
            const size_t bitIndex = static_cast<size_t>(bodyIndex) * 3 + static_cast<size_t>(component);
            EXPECT_EQ(flatSleeping.snapshot.bodyBits[bitIndex], flatAwake.snapshot.bodyBits[bitIndex]);
        }
    }

    std::printf("[solver7] sleeping island: flat/direct/packed all skip it (constraints=2); "
                "sleeping velocities/impulses bit-identical, awake island bit-identical to all-awake run\n");
}

// ---------------------------------------------------------------------------
// Partition semantics: empty partition = flat mode; Has/Clear; no partition.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, EmptyInstalledPartitionBehavesLikeFlatMode) {
    const SolverScene source = MakeTwoIslandScene();
    const SolveResult flat = RunVelocitySolve(source, SolveMode::Flat, 8, true);

    SolveResult installed;
    installed.scene = source;
    SolverSettings settings;
    ContactSolver solver;
    const std::vector<std::vector<int32_t>> emptyBodies;
    const std::vector<std::vector<int32_t>> emptyContacts;
    solver.SetIslandPartition(&emptyBodies, &emptyContacts);
    EXPECT_TRUE(solver.HasIslandPartition()); // installed, but documented as flat mode
    solver.Prepare(installed.scene.bodies, installed.scene.contacts, settings);
    solver.WarmStart(installed.scene.bodies, installed.scene.contacts, settings);
    solver.SolveVelocity(installed.scene.bodies, installed.scene.contacts, 8, settings);
    installed.snapshot = TakeSnapshot(installed.scene);

    EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, installed.snapshot.bodyBits), 0u);
    EXPECT_EQ(CountDifferences(flat.snapshot.impulseBits, installed.snapshot.impulseBits), 0u);
    std::printf("[solver7] installed-but-empty partition == flat mode: bit-identical\n");
}

TEST(SolverIslandTest, HasIslandPartitionAndClearSemantics) {
    const SolverScene source = MakeTwoIslandScene();
    ContactSolver solver;
    EXPECT_FALSE(solver.HasIslandPartition());
    solver.SetIslandPartition(&source.islandBodies, &source.islandContacts);
    EXPECT_TRUE(solver.HasIslandPartition());
    solver.ClearIslandPartition();
    EXPECT_FALSE(solver.HasIslandPartition());
    solver.SetIslandPartition(nullptr, nullptr);
    EXPECT_FALSE(solver.HasIslandPartition());
}

// ---------------------------------------------------------------------------
// Position layer (split impulse): grouped == flat, sleeping island untouched.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, GroupedPositionLayerIsBitIdenticalToFlat) {
    const SolverScene source = MakeTwoIslandScene();
    const SolveResult flat = RunPositionLayer(source, /*grouped=*/false);
    const SolveResult grouped = RunPositionLayer(source, /*grouped=*/true);

    EXPECT_EQ(CountDifferences(flat.snapshot.transformBits, grouped.snapshot.transformBits), 0u);
    EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, grouped.snapshot.bodyBits), 0u)
        << "real velocities untouched";
    // Real velocities were never modified by the position layer.
    for (size_t index = 0; index < source.bodies.size(); ++index) {
        EXPECT_EQ(FloatBits(flat.scene.bodies[index].linearVelocity.x),
                  FloatBits(source.bodies[index].linearVelocity.x));
        EXPECT_EQ(FloatBits(flat.scene.bodies[index].linearVelocity.y),
                  FloatBits(source.bodies[index].linearVelocity.y));
        EXPECT_EQ(FloatBits(flat.scene.bodies[index].angularVelocity),
                  FloatBits(source.bodies[index].angularVelocity));
        EXPECT_EQ(FloatBits(flat.scene.bodies[index].pseudoLinearVelocity.x), FloatBits(0.0f));
        EXPECT_EQ(FloatBits(flat.scene.bodies[index].pseudoAngularVelocity), FloatBits(0.0f));
    }
    std::printf("[solver7] position layer: %zu transform floats + %zu velocity floats bit-identical flat vs "
                "grouped\n",
                flat.snapshot.transformBits.size(), flat.snapshot.bodyBits.size());
}

TEST(SolverIslandTest, PositionLayerNeverPushesASleepingIsland) {
    const SolverScene sleepingSource = MakeSceneWithSleepingSecondIsland();
    const BitSnapshot initial = TakeSnapshot(sleepingSource);
    const SolveResult flat = RunPositionLayer(sleepingSource, /*grouped=*/false);
    const SolveResult grouped = RunPositionLayer(sleepingSource, /*grouped=*/true);

    EXPECT_EQ(CountDifferences(flat.snapshot.transformBits, grouped.snapshot.transformBits), 0u);
    for (const int32_t bodyIndex : sleepingSource.islandBodies[1]) {
        EXPECT_EQ(FloatBits(flat.scene.bodies[static_cast<size_t>(bodyIndex)].xf.p.x),
                  initial.transformBits[static_cast<size_t>(bodyIndex) * 4 + 0]);
        EXPECT_EQ(FloatBits(flat.scene.bodies[static_cast<size_t>(bodyIndex)].xf.p.y),
                  initial.transformBits[static_cast<size_t>(bodyIndex) * 4 + 1]);
    }
    // The awake island still got corrected (penetration above slop in the fixture).
    EXPECT_NE(FloatBits(flat.scene.bodies[1].xf.p.y), initial.transformBits[1 * 4 + 1]);
    std::printf("[solver7] sleeping island position layer: transforms unchanged in both modes\n");
}

// ---------------------------------------------------------------------------
// Dense-island three-mode equivalence: flat, grouped-direct (default) and
// grouped-packed must agree bit for bit on the 400-body / 760-contact island.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, ThreeSolveModesAreBitIdenticalOnADenseIsland) {
    const SolverScene scene = MakeLargeGridScene();
    ASSERT_EQ(scene.islandBodies.size(), 1u);
    ASSERT_EQ(scene.islandBodies[0].size(), static_cast<size_t>(kGridWidth * kGridHeight));
    ASSERT_FALSE(SolverSettings{}.packIslandBodies) << "shipped default must be grouped-direct";

    size_t comparedFloats = 0;
    for (const int32_t iterations : {1, 8}) {
        for (const bool warmStart : {false, true}) {
            const SolveResult flat = RunVelocitySolve(scene, SolveMode::Flat, iterations, warmStart);
            const SolveResult direct =
                RunVelocitySolve(scene, SolveMode::GroupedDirect, iterations, warmStart);
            const SolveResult packed =
                RunVelocitySolve(scene, SolveMode::GroupedPacked, iterations, warmStart);

            EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, direct.snapshot.bodyBits), 0u)
                << "iters=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(CountDifferences(flat.snapshot.bodyBits, packed.snapshot.bodyBits), 0u)
                << "iters=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(CountDifferences(flat.snapshot.impulseBits, direct.snapshot.impulseBits), 0u)
                << "iters=" << iterations << " warm=" << warmStart;
            EXPECT_EQ(CountDifferences(flat.snapshot.impulseBits, packed.snapshot.impulseBits), 0u)
                << "iters=" << iterations << " warm=" << warmStart;
            comparedFloats += 2u * (flat.snapshot.bodyBits.size() + flat.snapshot.impulseBits.size());
        }
    }
    std::printf("[solver7] dense island 400 bodies/760 contacts: flat vs grouped-direct vs grouped-packed "
                "%zu float comparisons, 0 differences\n",
                comparedFloats);
}

// ---------------------------------------------------------------------------
// Three-mode SolveVelocity timing (printed only; Release numbers are for the
// devlog, Debug is a smoke test). The default config must be the faster variant.
// ---------------------------------------------------------------------------

TEST(SolverIslandTest, SolveModeTimingComparisonIsPrinted) {
    constexpr int32_t kRepetitions = 40;
    const SolverScene scene = MakeLargeGridScene();
    const int32_t bodyCount = kGridWidth * kGridHeight;
    ASSERT_EQ(scene.islandBodies.size(), 1u);

    SolverSettings flatSettings;   // no partition -> flat order
    SolverSettings directSettings; // default: packIslandBodies == false
    SolverSettings packedSettings;
    packedSettings.packIslandBodies = true;
    ASSERT_FALSE(directSettings.packIslandBodies);

    SolverScene flatScene = scene;
    ContactSolver flatSolver;
    flatSolver.Prepare(flatScene.bodies, flatScene.contacts, flatSettings);

    SolverScene directScene = scene;
    ContactSolver directSolver;
    directSolver.SetIslandPartition(&directScene.islandBodies, &directScene.islandContacts);
    directSolver.Prepare(directScene.bodies, directScene.contacts, directSettings);

    SolverScene packedScene = scene;
    ContactSolver packedSolver;
    packedSolver.SetIslandPartition(&packedScene.islandBodies, &packedScene.islandContacts);
    packedSolver.Prepare(packedScene.bodies, packedScene.contacts, packedSettings);

    const auto timeSolve = [&](ContactSolver& solver, SolverScene& localScene, const SolverSettings& settings,
                               int32_t iterations) {
        const auto start = std::chrono::steady_clock::now();
        for (int32_t repetition = 0; repetition < kRepetitions; ++repetition) {
            solver.SolveVelocity(localScene.bodies, localScene.contacts, iterations, settings);
        }
        const auto end = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count();
    };

    for (const int32_t iterations : {8, 32}) {
        const double flatMilliseconds = timeSolve(flatSolver, flatScene, flatSettings, iterations);
        const double directMilliseconds = timeSolve(directSolver, directScene, directSettings, iterations);
        const double packedMilliseconds = timeSolve(packedSolver, packedScene, packedSettings, iterations);
        std::printf("[solver7] solve timing %d bodies / %d contacts, %d reps x %d iters: flat=%.3f ms "
                    "grouped-direct=%.3f ms grouped-packed=%.3f ms (direct/flat=%.2fx, packed/flat=%.2fx) "
                    "[%s build]\n",
                    bodyCount, static_cast<int>(scene.contacts.size()), kRepetitions, iterations,
                    flatMilliseconds, directMilliseconds, packedMilliseconds,
                    flatMilliseconds / std::max(1.0e-9, directMilliseconds),
                    flatMilliseconds / std::max(1.0e-9, packedMilliseconds),
#ifdef NDEBUG
                    "Release"
#else
                    "Debug"
#endif
        );
        // Timing is environment-dependent, so nothing is asserted about the ratios;
        // the solves must merely have run.
        EXPECT_GT(flatMilliseconds, 0.0);
        EXPECT_GT(directMilliseconds, 0.0);
        EXPECT_GT(packedMilliseconds, 0.0);
    }
}

// ---------------------------------------------------------------------------
// World-level A/B: with sleeping switched off, `useIslandSolving` must not change
// any physics bit. This exercises the full pipeline (World builds the canonical
// partition, hands it to the solver) rather than hand-built partitions.
// (World::UpdateSleep and Body::islandId bookkeeping are excluded by design:
// flat mode intentionally leaves islandId at -1 and sleeping is an island feature,
// not a solving-order feature.)
// ---------------------------------------------------------------------------

namespace {

using p2::World;

void BuildWorldABScene(World& world) {
    p2::BodyDef groundDef;
    groundDef.type = BodyType::Static;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    for (int32_t index = 0; index < 10; ++index) {
        p2::BodyDef boxDef;
        boxDef.type = BodyType::Dynamic;
        boxDef.position = Vec2{0.0f, 0.5f + static_cast<float>(index)};
        boxDef.density = 1.0f;
        const BodyHandle box = world.CreateBody(boxDef);
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(0.5f, 0.5f)));
    }
    const Vec2 circlePositions[] = {{-2.0f, 3.0f}, {2.0f, 5.0f}, {-2.0f, 7.0f}};
    const float circleRadii[] = {0.4f, 0.5f, 0.3f};
    for (int32_t index = 0; index < 3; ++index) {
        p2::BodyDef circleDef;
        circleDef.type = BodyType::Dynamic;
        circleDef.position = circlePositions[index];
        circleDef.density = 1.0f;
        const BodyHandle circle = world.CreateBody(circleDef);
        EXPECT_TRUE(world.AttachShape(circle, world.CreateCircleShape(circleRadii[index])));
    }
}

// Physics-only state: no islandId, no sleep timer. Contact manifold + impulses are
// included because the solver owns them.
std::vector<uint32_t> SnapshotWorldPhysics(const World& world) {
    std::vector<uint32_t> bits;
    for (const Body& body : world.GetBodies()) {
        if (!body.handle.IsValid()) {
            continue;
        }
        bits.push_back(FloatBits(body.xf.p.x));
        bits.push_back(FloatBits(body.xf.p.y));
        bits.push_back(FloatBits(body.xf.q.c));
        bits.push_back(FloatBits(body.xf.q.s));
        bits.push_back(FloatBits(body.linearVelocity.x));
        bits.push_back(FloatBits(body.linearVelocity.y));
        bits.push_back(FloatBits(body.angularVelocity));
        bits.push_back(FloatBits(body.pseudoLinearVelocity.x));
        bits.push_back(FloatBits(body.pseudoLinearVelocity.y));
        bits.push_back(FloatBits(body.pseudoAngularVelocity));
        bits.push_back(body.isAwake ? 1u : 0u);
    }
    for (const Contact& contact : world.GetContacts()) {
        bits.push_back(static_cast<uint32_t>(contact.bodyA.index));
        bits.push_back(static_cast<uint32_t>(contact.bodyB.index));
        bits.push_back(static_cast<uint32_t>(contact.manifold.pointCount));
        bits.push_back(FloatBits(contact.manifold.normal.x));
        bits.push_back(FloatBits(contact.manifold.normal.y));
        bits.push_back(FloatBits(contact.manifold.penetration));
        bits.push_back(contact.isPersistent ? 1u : 0u);
        for (int32_t pointIndex = 0; pointIndex < p2::kMaxManifoldPoints; ++pointIndex) {
            const p2::ContactPoint& point = contact.manifold.points[pointIndex];
            bits.push_back(FloatBits(point.localPointA.x));
            bits.push_back(FloatBits(point.localPointA.y));
            bits.push_back(FloatBits(point.localPointB.x));
            bits.push_back(FloatBits(point.localPointB.y));
            bits.push_back(FloatBits(point.normalImpulse));
            bits.push_back(FloatBits(point.tangentImpulse));
            bits.push_back(point.id.Key());
        }
    }
    return bits;
}

} // namespace

TEST(SolverIslandTest, WorldLevelIslandSolvingToggleIsBitIdenticalWhenSleepingIsOff) {
    World groupedWorld;
    World flatWorld;
    SolverSettings groupedSettings = groupedWorld.GetSolverSettings();
    groupedSettings.useIslandSolving = true;
    groupedSettings.enableSleeping = false;
    groupedWorld.SetSolverSettings(groupedSettings);
    SolverSettings flatSettings = flatWorld.GetSolverSettings();
    flatSettings.useIslandSolving = false;
    flatSettings.enableSleeping = false;
    flatWorld.SetSolverSettings(flatSettings);

    BuildWorldABScene(groupedWorld);
    BuildWorldABScene(flatWorld);

    int32_t firstMismatchStep = -1;
    size_t firstMismatchIndex = 0;
    size_t comparedFloats = 0;
    constexpr int32_t kSteps = 300;
    for (int32_t step = 0; step < kSteps; ++step) {
        groupedWorld.Step(kDt);
        flatWorld.Step(kDt);
        const std::vector<uint32_t> groupedBits = SnapshotWorldPhysics(groupedWorld);
        const std::vector<uint32_t> flatBits = SnapshotWorldPhysics(flatWorld);
        if (groupedBits.size() != flatBits.size()) {
            firstMismatchStep = step;
            firstMismatchIndex = std::min(groupedBits.size(), flatBits.size());
            break;
        }
        comparedFloats = groupedBits.size();
        for (size_t index = 0; index < groupedBits.size(); ++index) {
            if (groupedBits[index] != flatBits[index]) {
                firstMismatchStep = step;
                firstMismatchIndex = index;
                break;
            }
        }
        if (firstMismatchStep >= 0) {
            break;
        }
    }

    std::printf(
        "[solver7] world A/B (sleep off): steps=%d floats/step=%zu grouped-islands=%d flat-islands=%d "
        "first-mismatch step=%d index=%zu\n",
        kSteps, comparedFloats, groupedWorld.GetIslandCount(), flatWorld.GetIslandCount(), firstMismatchStep,
        firstMismatchIndex);
    EXPECT_EQ(firstMismatchStep, -1);
    EXPECT_GT(groupedWorld.GetIslandCount(), 0) << "grouped mode must build a non-empty partition";
    EXPECT_EQ(flatWorld.GetIslandCount(), 0);
    EXPECT_GT(comparedFloats, 0u);
}
