// Week 7 (P13): World-level island partition tests.
//
// The island partition is the connected-component decomposition of the
// dynamic-body contact graph (contract: Solver/Island.h):
//  - only DYNAMIC bodies are island members;
//  - static and kinematic bodies are BOUNDARIES: island contacts still act on
//    them, but they neither join an island nor connect two islands. In particular
//    a shared ground plane must NOT merge every grounded stack into one island;
//  - every dynamic body belongs to exactly one island (singletons included);
//  - the partition is canonical: islands ordered by smallest member body slot,
//    members ascending, contact indices ascending in global contact order. That
//    ordering is what makes the island-grouped solve bit-identical to the flat
//    solve (P13 "娑撳簼绗夐幐澶婄煗濮瑰倽袙 hash 娑撯偓閼?).
//
// What this file pins at the World level:
//  - three separated stacks on one ground -> exactly 3 islands;
//  - Body::islandId agrees with the partition and the ids are dense and canonical;
//  - a connector body bridging two stacks merges exactly those two islands;
//  - `useIslandSolving = false` leaves the partition empty but produces
//    BIT-IDENTICAL physics to the grouped solve over 300 steps (island
//    independence => Gauss-Seidel updates commute across islands);
//  - a 500-body touching chain (the 2000-body P13 requirement scaled down for
//    test time) builds as ONE island: the DFS is iterative, no stack overflow.
//
// [world7] printf lines are the measurements; comments quote them.

#include <gtest/gtest.h>

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::SolverSettings;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr int32_t kEquivalenceSteps = 300;

BodyDef MakeBodyDef(BodyType type, Vec2 position, float density = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.density = density;
    return def;
}

BodyHandle AddGroundPlane(World& world) {
    const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
    return ground;
}

BodyHandle AddBox(World& world, Vec2 position, float halfWidth = 0.5f, float halfHeight = 0.5f) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position));
    EXPECT_TRUE(world.AttachShape(body, world.CreateBoxShape(halfWidth, halfHeight)));
    return body;
}

BodyHandle AddCircle(World& world, Vec2 position, float radius = 0.4f) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position));
    EXPECT_TRUE(world.AttachShape(body, world.CreateCircleShape(radius)));
    return body;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

// Flattens the observable physics state into a float array: every live body's
// transform / velocities / pseudo velocities / awake flag, then every contact
// (pair identity, manifold, anchors, accumulated impulses, persistence, SAT
// cache). Used for the island-solving A/B memcmp.
//
// `Body::islandId` is deliberately NOT hashed: it is dispatch bookkeeping, not
// physics. The flat solve (useIslandSolving = false) sets every islandId to -1 by
// design, so including it would make the A/B memcmp fail on a field that by
// definition differs. The grouped run asserts islandId validity separately.
std::vector<float> SnapshotPhysicsState(const World& world) {
    std::vector<float> values;
    const std::vector<Body>& bodies = world.GetBodies();
    values.reserve(bodies.size() * 14U + world.GetContacts().size() * 32U);
    for (const Body& body : bodies) {
        if (!body.handle.IsValid()) {
            continue;
        }
        values.push_back(body.xf.p.x);
        values.push_back(body.xf.p.y);
        values.push_back(body.xf.q.c);
        values.push_back(body.xf.q.s);
        values.push_back(body.linearVelocity.x);
        values.push_back(body.linearVelocity.y);
        values.push_back(body.angularVelocity);
        values.push_back(body.pseudoLinearVelocity.x);
        values.push_back(body.pseudoLinearVelocity.y);
        values.push_back(body.pseudoAngularVelocity);
        values.push_back(body.force.x);
        values.push_back(body.force.y);
        values.push_back(body.torque);
        values.push_back(body.isAwake ? 1.0f : 0.0f);
    }
    for (const Contact& contact : world.GetContacts()) {
        values.push_back(static_cast<float>(contact.bodyA.index));
        values.push_back(static_cast<float>(contact.bodyB.index));
        values.push_back(static_cast<float>(contact.manifold.pointCount));
        values.push_back(contact.manifold.normal.x);
        values.push_back(contact.manifold.normal.y);
        values.push_back(contact.manifold.penetration);
        values.push_back(contact.isPersistent ? 1.0f : 0.0f);
        for (int32_t pointIndex = 0; pointIndex < p2::kMaxManifoldPoints; ++pointIndex) {
            const p2::ContactPoint& point = contact.manifold.points[pointIndex];
            values.push_back(point.localPointA.x);
            values.push_back(point.localPointA.y);
            values.push_back(point.localPointB.x);
            values.push_back(point.localPointB.y);
            values.push_back(point.normalImpulse);
            values.push_back(point.tangentImpulse);
            values.push_back(static_cast<float>(point.id.Key()));
        }
    }
    return values;
}

int32_t FindFirstBitMismatch(const std::vector<float>& a, const std::vector<float>& b) {
    const std::size_t common = std::min(a.size(), b.size());
    for (std::size_t index = 0; index < common; ++index) {
        if (std::memcmp(&a[index], &b[index], sizeof(float)) != 0) {
            return static_cast<int32_t>(index);
        }
    }
    return a.size() == b.size() ? -1 : static_cast<int32_t>(common);
}

TEST(WorldIslandsTest, SeparatedStacksOnASharedGroundAreSeparateIslands) {
    // Three 2-box stacks on ONE static ground plane, 4 m apart. The ground is a
    // boundary: it carries the island contacts but must not connect the islands,
    // otherwise every grounded stack would merge and sleeping could never engage.
    World world;
    world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    const BodyHandle ground = AddGroundPlane(world);
    std::vector<std::vector<BodyHandle>> stacks(3);
    for (int32_t stackIndex = 0; stackIndex < 3; ++stackIndex) {
        const float x = -4.0f + 4.0f * static_cast<float>(stackIndex);
        stacks[static_cast<std::size_t>(stackIndex)].push_back(AddBox(world, Vec2(x, 0.5f)));
        stacks[static_cast<std::size_t>(stackIndex)].push_back(AddBox(world, Vec2(x, 1.5f)));
    }

    StepFor(world, 120);
    ASSERT_EQ(world.GetIslandCount(), 3);
    const int32_t separatedIslandCount = world.GetIslandCount();
    for (std::size_t stackIndex = 0; stackIndex < stacks.size(); ++stackIndex) {
        const int32_t firstIsland = world.GetBody(stacks[stackIndex][0])->islandId;
        EXPECT_GE(firstIsland, 0);
        EXPECT_EQ(world.GetBody(stacks[stackIndex][1])->islandId, firstIsland);
        for (std::size_t other = 0; other < stacks.size(); ++other) {
            if (other == stackIndex) {
                continue;
            }
            EXPECT_NE(world.GetBody(stacks[other][0])->islandId, firstIsland)
                << "a shared static ground must not merge islands";
        }
    }
    // The ground is a boundary, never an island member.
    EXPECT_EQ(world.GetBody(ground)->islandId, -1);

    // Bridge exactly stack 0 and stack 1 with one long thin dynamic box resting on
    // both tops (stack tops are at y = 2.0; the connector is placed 0.05 m into
    // them so the contact is unambiguous). Two islands were merged, the third one
    // must stay independent: 2 islands.
    const BodyHandle connector = AddBox(world, Vec2(-2.0f, 1.95f), 2.0f, 0.1f);
    StepFor(world, 10);
    EXPECT_EQ(world.GetIslandCount(), 2);
    EXPECT_EQ(world.GetBody(connector)->islandId, world.GetBody(stacks[0][0])->islandId);
    EXPECT_EQ(world.GetBody(stacks[0][0])->islandId, world.GetBody(stacks[1][0])->islandId);
    EXPECT_NE(world.GetBody(stacks[2][0])->islandId, world.GetBody(stacks[0][0])->islandId);
    std::printf("[world7] islands: 3 separated stacks -> %d islands; connector bridges stacks 0+1 "
                "-> 2 islands (connector islandId = %d, stack2 islandId = %d)\n",
                static_cast<int>(separatedIslandCount), world.GetBody(connector)->islandId,
                world.GetBody(stacks[2][0])->islandId);
}

TEST(WorldIslandsTest, IslandIdsAreDenseCanonicalAndConsistent) {
    // Canonical order check: islands are ordered by their smallest member slot.
    // Scene order: ground(0), stack A boxes(1,2), stack B boxes(3,4), free balls
    // (5,6). Contacts after one step: the stacked boxes touch each other and the
    // ground; the free-falling balls touch nothing -> singleton islands.
    World world;
    const BodyHandle ground = AddGroundPlane(world);
    const BodyHandle stackABottom = AddBox(world, Vec2(-3.0f, 0.5f));
    const BodyHandle stackATop = AddBox(world, Vec2(-3.0f, 1.5f));
    const BodyHandle stackBBottom = AddBox(world, Vec2(3.0f, 0.5f));
    const BodyHandle stackBTop = AddBox(world, Vec2(3.0f, 1.5f));
    const BodyHandle ballA = AddCircle(world, Vec2(0.0f, 6.0f));
    const BodyHandle ballB = AddCircle(world, Vec2(0.0f, 9.0f), 0.3f);

    StepFor(world, 2);
    ASSERT_EQ(world.GetIslandCount(), 4);

    const BodyHandle dynamicBodies[] = {stackABottom, stackATop, stackBBottom, stackBTop, ballA, ballB};
    const int32_t expectedIslandIds[] = {0, 0, 1, 1, 2, 3};
    for (std::size_t index = 0; index < std::size(dynamicBodies); ++index) {
        const Body* body = world.GetBody(dynamicBodies[index]);
        ASSERT_NE(body, nullptr);
        EXPECT_EQ(body->islandId, expectedIslandIds[index]);
    }
    EXPECT_EQ(world.GetBody(ground)->islandId, -1);

    // Dense ids: exactly {0, 1, 2, 3} for 4 islands.
    std::vector<int32_t> seen(world.GetIslandCount(), 0);
    for (const Body& body : world.GetBodies()) {
        if (!body.handle.IsValid() || body.type != BodyType::Dynamic) {
            continue;
        }
        ASSERT_GE(body.islandId, 0);
        ASSERT_LT(body.islandId, world.GetIslandCount());
        ++seen[static_cast<std::size_t>(body.islandId)];
    }
    EXPECT_EQ(seen[0], 2); // stack A
    EXPECT_EQ(seen[1], 2); // stack B
    EXPECT_EQ(seen[2], 1); // ball A (singleton)
    EXPECT_EQ(seen[3], 1); // ball B (singleton)
    std::printf("[world7] island ids: 4 islands, sizes = %d/%d/%d/%d, ground islandId = -1\n",
                static_cast<int>(seen[0]), static_cast<int>(seen[1]), static_cast<int>(seen[2]),
                static_cast<int>(seen[3]));
}

TEST(WorldIslandsTest, IslandSolvingToggleIsBitIdenticalPhysics) {
    // P13 A/B requirement: the island-grouped solve must be bit-identical to the
    // flat Week 6 solve. Islands are independent components, so their
    // Gauss-Seidel updates commute; only the order INSIDE an island matters and it
    // is the global contact order. Sleeping is disabled on both sides so the
    // comparison isolates the solving order (sleeping is a separate feature and
    // needs a partition, i.e. it is off in flat mode by design).
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

    // Realistic scene: ground + ten-box tower + three circles that fall and roll.
    const auto buildScene = [](World& world, std::vector<BodyHandle>& bodies) {
        bodies.push_back(AddGroundPlane(world));
        for (int32_t index = 0; index < 10; ++index) {
            bodies.push_back(AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index))));
        }
        bodies.push_back(AddCircle(world, Vec2(-2.0f, 3.0f), 0.4f));
        bodies.push_back(AddCircle(world, Vec2(2.0f, 5.0f), 0.5f));
        bodies.push_back(AddCircle(world, Vec2(-2.0f, 7.0f), 0.3f));
    };
    std::vector<BodyHandle> groupedBodies;
    std::vector<BodyHandle> flatBodies;
    buildScene(groupedWorld, groupedBodies);
    buildScene(flatWorld, flatBodies);
    ASSERT_EQ(groupedBodies.size(), flatBodies.size());

    int32_t firstMismatchStep = -1;
    int32_t firstMismatchIndex = -1;
    std::size_t floatsPerStep = 0;
    for (int32_t step = 0; step < kEquivalenceSteps; ++step) {
        groupedWorld.Step(kDt);
        flatWorld.Step(kDt);
        const std::vector<float> groupedSnapshot = SnapshotPhysicsState(groupedWorld);
        const std::vector<float> flatSnapshot = SnapshotPhysicsState(flatWorld);
        if (groupedSnapshot.size() != flatSnapshot.size()) {
            firstMismatchStep = step;
            firstMismatchIndex = -1;
            break;
        }
        floatsPerStep = groupedSnapshot.size();
        const int32_t mismatch = FindFirstBitMismatch(groupedSnapshot, flatSnapshot);
        if (mismatch >= 0) {
            firstMismatchStep = step;
            firstMismatchIndex = mismatch;
            break;
        }
        if (groupedWorld.GetIslandCount() <= 0) {
            firstMismatchStep = step; // the grouped partition must be non-empty
            firstMismatchIndex = -2;
            break;
        }
        EXPECT_EQ(flatWorld.GetIslandCount(), 0);
    }

    std::printf("[world7] island-solve A/B: steps = %d, floats/step = %d, grouped islands = %d, "
                "flat islands = %d, first mismatch step = %d index = %d\n",
                static_cast<int>(kEquivalenceSteps), static_cast<int>(floatsPerStep),
                static_cast<int>(groupedWorld.GetIslandCount()), static_cast<int>(flatWorld.GetIslandCount()),
                static_cast<int>(firstMismatchStep), static_cast<int>(firstMismatchIndex));
    EXPECT_EQ(firstMismatchStep, -1);
    EXPECT_GT(groupedWorld.GetIslandCount(), 0);
    EXPECT_EQ(flatWorld.GetIslandCount(), 0);
    // Flat mode must not leave island ids behind on the bodies.
    for (const Body& body : flatWorld.GetBodies()) {
        if (body.handle.IsValid() && body.type == BodyType::Dynamic) {
            EXPECT_EQ(body.islandId, -1);
        }
    }
    // Grouped mode: every dynamic body is in exactly one island.
    for (const Body& body : groupedWorld.GetBodies()) {
        if (body.handle.IsValid() && body.type == BodyType::Dynamic) {
            EXPECT_GE(body.islandId, 0);
            EXPECT_LT(body.islandId, groupedWorld.GetIslandCount());
        }
    }
}

TEST(WorldIslandsTest, FiveHundredBodyChainIsOneIsland) {
    // P13 requires a 2000-body chain to build without recursion/overflow; 500 is
    // the test-time compromise (the DFS is iterative, so the failure mode - stack
    // overflow - would already show at 500 on this 1 MB-by-default stack).
    World world(Vec2(0.0f, 0.0f)); // no gravity: the chain only touches its neighbours
    constexpr int32_t kChainLength = 500;
    std::vector<BodyHandle> chain;
    chain.reserve(static_cast<std::size_t>(kChainLength));
    for (int32_t index = 0; index < kChainLength; ++index) {
        // 0.99 m pitch with 1 m boxes: each neighbour pair overlaps by 0.01 m, so
        // every link generates a contact.
        chain.push_back(AddBox(world, Vec2(0.99f * static_cast<float>(index), 0.0f)));
    }
    // Warm up: one step generates the contacts, the second builds the island.
    StepFor(world, 2);
    ASSERT_EQ(world.GetIslandCount(), 1);
    EXPECT_GE(world.GetContactCount(), kChainLength - 5); // near-neighbour contacts
    const int32_t islandId = world.GetBody(chain.front())->islandId;
    EXPECT_EQ(islandId, 0);
    for (const BodyHandle body : chain) {
        EXPECT_EQ(world.GetBody(body)->islandId, islandId);
    }
    std::printf("[world7] long chain: %d bodies -> %d island(s), %d contacts\n",
                static_cast<int>(kChainLength), static_cast<int>(world.GetIslandCount()),
                static_cast<int>(world.GetContactCount()));
}

} // namespace
