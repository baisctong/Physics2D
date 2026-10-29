// Week 5 (D7 / P09 / P10): World-level integration tests for the solver wiring.
//
// This file pins the Week 5 acceptance contract from HANDOFF section 5.1 on the
// ten-box scene - the exact scene the Week 4 baseline recorded as collapsing -
// so the before/after comparison is direct:
//
//   - the ten 1 m boxes stand for 10 simulated seconds at the DEFAULT iteration
//     counts (8 velocity, 3 position): max penetration < 0.01 m, top box near
//     y = 9.5 m, per-box tilt and lateral drift bounded, no NaN/Inf;
//   - two identically built Worlds stay bit-identical (memcmp of the extracted
//     physics state) after 600 steps;
//   - the warm-start factor comparison (0 / 0.5 / 0.85 / 0.9 / 0.95 / 1.0) prints
//     the measured collapse step of every factor and asserts the two ends of the
//     contract: the full seed (the shipped default) stands, a missing seed does
//     not survive 600 steps at 8 iterations;
//   - the split-impulse on/off comparison prints max penetration + residual
//     jitter, and a dedicated deep-overlap scene shows that split impulse removes
//     penetration without placing a correction velocity into the REAL velocity
//     state (the Baumgarte baseline does exactly that);
//   - disabling warm starting and setting warmStartFactor = 0 are equivalent.
//
// Scene convention (same as the Week 4 baseline test in test_world_sat.cpp):
// static ground plane at y = 0 (normal +y), ten dynamic 1 m x 1 m boxes (half
// extent 0.5 m, density 1, default friction 0.3) at y = 0.5, 1.5, ..., 9.5,
// default gravity (0, -10), fixed dt = 1/60 s, 600 steps = 10 s.
//
// The [world5] printf lines are the measurements this file asserts; the comments
// quote the same numbers so a regression is readable without re-running.
//
// Determinism note: the state snapshot is extracted field by field into a float
// array before memcmp. Body contains a std::string (debugName), so memcmp on the
// raw Body array would compare object representation (pointers), not physics.

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
constexpr int32_t kTowerBoxCount = 10;
constexpr int32_t kSimulationSteps = 600;   // 10 s at the fixed step
constexpr int32_t kRestingWindowSteps = 60; // last simulated second for jitter
constexpr float kBoxHalfExtent = 0.5f;      // 1 m x 1 m box -> one storey per box
constexpr float kStoreyHeight = 2.0f * kBoxHalfExtent;
constexpr float kTopBoxStartY = kBoxHalfExtent + static_cast<float>(kTowerBoxCount - 1) * kStoreyHeight;

// Acceptance thresholds (HANDOFF section 5.1). These are the milestone contract,
// NOT engine tunables: the engine constants live in Core/Settings.h.
constexpr float kMilestoneMaxPenetration = 0.01f; // "max penetration < 0.01 m"
constexpr float kMaxAllowedTilt = 0.05f;          // rad (~2.9 deg): visually upright
constexpr float kMaxAllowedDrift = 0.05f;         // m: per-box lateral wander
constexpr float kRestingSpeedTolerance = 0.05f;   // m/s: residual jitter budget
// Week 4 baseline used "top box below half the tower height" as the collapse
// probe; keeping the same definition makes the before/after directly comparable.
constexpr float kCollapseTopY = 5.0f;

BodyDef MakeBodyDef(BodyType type, Vec2 position) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.density = 1.0f;
    return def;
}

struct TowerScene {
    BodyHandle ground;
    std::vector<BodyHandle> boxes;
};

// Ground plane + `boxCount` unit boxes stacked with one storey of pitch.
TowerScene BuildTower(World& world, int32_t boxCount = kTowerBoxCount) {
    TowerScene scene;
    scene.ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(scene.ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
    scene.boxes.reserve(static_cast<std::size_t>(boxCount));
    for (int32_t index = 0; index < boxCount; ++index) {
        const float y = kBoxHalfExtent + static_cast<float>(index) * kStoreyHeight;
        const BodyHandle box = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, y)));
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(kBoxHalfExtent, kBoxHalfExtent)));
        scene.boxes.push_back(box);
    }
    return scene;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

float MaxContactPenetration(const World& world) {
    float maxPenetration = 0.0f;
    for (const Contact& contact : world.GetContacts()) {
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    return maxPenetration;
}

int32_t TotalContactPoints(const World& world) {
    int32_t points = 0;
    for (const Contact& contact : world.GetContacts()) {
        points += contact.manifold.pointCount;
    }
    return points;
}

bool IsFiniteBody(const World& world, BodyHandle body) {
    const Vec2 position = world.GetPosition(body);
    const Vec2 velocity = world.GetLinearVelocity(body);
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(world.GetAngle(body)) &&
           std::isfinite(velocity.x) && std::isfinite(velocity.y) &&
           std::isfinite(world.GetAngularVelocity(body));
}

// Aggregate measurements of one tower run. "max*" values are over the whole run;
// "final*" values are read after the last step (the resting state).
struct TowerMetrics {
    float maxPenetration = 0.0f;
    float maxTiltOverRun = 0.0f;
    float maxLateralOverRun = 0.0f;
    float finalTopY = 0.0f;
    float finalMaxTilt = 0.0f;
    float finalMaxLateral = 0.0f;
    float finalMaxSpeed = 0.0f;
    float jitterMaxSpeed = 0.0f; // max speed during the last simulated second
    int32_t finalContactCount = 0;
    int32_t finalContactPointCount = 0;
    int32_t collapseStep = -1; // first step with the top box below kCollapseTopY
    bool allFinite = true;
};

TowerMetrics RunTower(World& world, const TowerScene& scene, int32_t stepCount) {
    TowerMetrics metrics;
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
        metrics.maxPenetration = std::max(metrics.maxPenetration, MaxContactPenetration(world));
        if (metrics.collapseStep < 0 && world.GetPosition(scene.boxes.back()).y < kCollapseTopY) {
            metrics.collapseStep = step;
        }
        float stepMaxSpeed = 0.0f;
        for (const BodyHandle box : scene.boxes) {
            const Vec2 position = world.GetPosition(box);
            const Vec2 velocity = world.GetLinearVelocity(box);
            metrics.maxTiltOverRun = std::max(metrics.maxTiltOverRun, std::fabs(world.GetAngle(box)));
            metrics.maxLateralOverRun = std::max(metrics.maxLateralOverRun, std::fabs(position.x));
            stepMaxSpeed = std::max(stepMaxSpeed, p2::Length(velocity));
            metrics.allFinite = metrics.allFinite && IsFiniteBody(world, box);
        }
        if (step >= stepCount - kRestingWindowSteps) {
            metrics.jitterMaxSpeed = std::max(metrics.jitterMaxSpeed, stepMaxSpeed);
        }
    }

    metrics.finalTopY = world.GetPosition(scene.boxes.back()).y;
    metrics.finalContactCount = world.GetContactCount();
    metrics.finalContactPointCount = TotalContactPoints(world);
    for (const BodyHandle box : scene.boxes) {
        const Vec2 position = world.GetPosition(box);
        metrics.finalMaxTilt = std::max(metrics.finalMaxTilt, std::fabs(world.GetAngle(box)));
        metrics.finalMaxLateral = std::max(metrics.finalMaxLateral, std::fabs(position.x));
        metrics.finalMaxSpeed = std::max(metrics.finalMaxSpeed, p2::Length(world.GetLinearVelocity(box)));
    }
    return metrics;
}

void PrintTowerMetrics(const char* label, const TowerMetrics& metrics) {
    std::printf("[world5] %s: max penetration = %.6f m, top y = %.5f m (start %.5f), "
                "max |tilt| over run = %.6f rad, final max |tilt| = %.6f rad, "
                "final max |x| = %.6f m, final max speed = %.6f m/s, "
                "jitter max speed = %.6f m/s, contacts = %d, points = %d, collapse step = %d\n",
                label, static_cast<double>(metrics.maxPenetration), static_cast<double>(metrics.finalTopY),
                static_cast<double>(kTopBoxStartY), static_cast<double>(metrics.maxTiltOverRun),
                static_cast<double>(metrics.finalMaxTilt), static_cast<double>(metrics.finalMaxLateral),
                static_cast<double>(metrics.finalMaxSpeed), static_cast<double>(metrics.jitterMaxSpeed),
                static_cast<int>(metrics.finalContactCount), static_cast<int>(metrics.finalContactPointCount),
                static_cast<int>(metrics.collapseStep));
}

void PrintTowerState(const World& world, const TowerScene& scene) {
    for (std::size_t index = 0; index < scene.boxes.size(); ++index) {
        const Vec2 position = world.GetPosition(scene.boxes[index]);
        std::printf("[world5]   box %d: x = %+.5f, y = %.5f, angle = %+.6f rad\n", static_cast<int>(index),
                    static_cast<double>(position.x), static_cast<double>(position.y),
                    static_cast<double>(world.GetAngle(scene.boxes[index])));
    }
}

// Flattens the full physics state (transform, real/pseudo velocities, force
// accumulators, contact manifolds and accumulated impulses) into a float array.
// Two Worlds built and stepped with identical call sequences must match bit for
// bit; the engine reads no wall clock, RNG or unordered container.
std::vector<float> SnapshotPhysicsState(const World& world) {
    std::vector<float> values;
    const std::vector<Body>& bodies = world.GetBodies();
    values.reserve(bodies.size() * 13U + world.GetContacts().size() * 22U);
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
        // Split-impulse scratch state: SolvePosition must clear it every step, so
        // including it here also pins "no pseudo state leaks into the next step".
        values.push_back(body.pseudoLinearVelocity.x);
        values.push_back(body.pseudoLinearVelocity.y);
        values.push_back(body.pseudoAngularVelocity);
        values.push_back(body.force.x);
        values.push_back(body.force.y);
        values.push_back(body.torque);
    }
    for (const Contact& contact : world.GetContacts()) {
        values.push_back(static_cast<float>(contact.bodyA.index));
        values.push_back(static_cast<float>(contact.bodyB.index));
        values.push_back(static_cast<float>(contact.manifold.pointCount));
        values.push_back(contact.manifold.normal.x);
        values.push_back(contact.manifold.normal.y);
        values.push_back(contact.manifold.penetration);
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

// Returns the flat index of the first float whose bit pattern differs, or -1.
int32_t FindFirstBitMismatch(const std::vector<float>& a, const std::vector<float>& b) {
    const std::size_t common = std::min(a.size(), b.size());
    for (std::size_t index = 0; index < common; ++index) {
        if (std::memcmp(&a[index], &b[index], sizeof(float)) != 0) {
            return static_cast<int32_t>(index);
        }
    }
    return a.size() == b.size() ? -1 : static_cast<int32_t>(common);
}

TEST(WorldStabilityTest, TenBoxTowerStandsForTenSecondsAtDefaultIterations) {
    World world;
    const TowerScene scene = BuildTower(world);
    // Pin the setup: the milestone is about the DEFAULT iteration counts; any
    // caller-side override would invalidate the acceptance claim.
    ASSERT_EQ(world.GetSolverSettings().velocityIterations, p2::kVelocityIterations);
    ASSERT_EQ(world.GetSolverSettings().positionIterations, p2::kPositionIterations);

    const TowerMetrics metrics = RunTower(world, scene, kSimulationSteps);
    PrintTowerMetrics("milestone (default 8 vel / 3 pos, 600 steps)", metrics);
    PrintTowerState(world, scene);

    // MEASURED (Week 5 solver, default settings): max penetration 0.005223 m,
    // top y 9.47813 m (2.2 cm below the built 9.5 m = the slop band accumulated
    // over ten contacts), max |tilt| over run 0.004596 rad, final max |x|
    // 0.023825 m, final max speed 0.007216 m/s, contacts 10 / points 20.

    // Ten face-face contacts: ground-box0 plus nine box-box pairs. A face-face
    // manifold has two points; 18 is the floor that tolerates one contact point
    // being lost at a corner while the tower is still standing.
    EXPECT_EQ(metrics.finalContactCount, kTowerBoxCount);
    EXPECT_GE(metrics.finalContactPointCount, 2 * kTowerBoxCount - 2);

    // HANDOFF 5.1 milestone: max penetration < 0.01 m over the whole run.
    EXPECT_LT(metrics.maxPenetration, kMilestoneMaxPenetration);
    // Top box must stay at its built position (9.5 m) - a tower that sank or was
    // pushed up would fail the "vertex displacement bounded" clause.
    EXPECT_NEAR(metrics.finalTopY, kTopBoxStartY, kMaxAllowedDrift);
    EXPECT_LT(metrics.finalMaxTilt, kMaxAllowedTilt);
    EXPECT_LT(metrics.finalMaxLateral, kMaxAllowedDrift);
    EXPECT_LT(metrics.finalMaxSpeed, kRestingSpeedTolerance);
    // Collapse probe of the Week 4 baseline: must never fire in Week 5.
    EXPECT_EQ(metrics.collapseStep, -1);
    EXPECT_TRUE(metrics.allFinite);

    // Per-storey positions: each box keeps its own slot (no interpenetration /
    // reordering), which also proves the tower is not leaning as a rigid group.
    for (std::size_t index = 0; index < scene.boxes.size(); ++index) {
        const Vec2 position = world.GetPosition(scene.boxes[index]);
        const float expectedY = kBoxHalfExtent + static_cast<float>(index) * kStoreyHeight;
        EXPECT_NEAR(position.y, expectedY, kMaxAllowedDrift);
        EXPECT_LT(std::fabs(position.x), kMaxAllowedDrift);
        EXPECT_LT(std::fabs(world.GetAngle(scene.boxes[index])), kMaxAllowedTilt);
    }
}

TEST(WorldStabilityTest, TwoIdenticalTowersStayBitIdentical) {
    // Determinism (Section 7.4): identical call order -> identical float bits.
    // This catches hidden state (pointer-order iteration, unordered containers,
    // wall clock) anywhere in the new friction/warm-start/split-impulse path.
    // MEASURED: 343 state floats (13 bodies + 10 contacts: transforms, velocities,
    // pseudo velocities, forces, manifold data and accumulated impulses), no
    // mismatch.
    World worldA;
    World worldB;
    const TowerScene sceneA = BuildTower(worldA);
    const TowerScene sceneB = BuildTower(worldB);
    ASSERT_EQ(sceneA.boxes.size(), sceneB.boxes.size());

    RunTower(worldA, sceneA, kSimulationSteps);
    RunTower(worldB, sceneB, kSimulationSteps);

    ASSERT_EQ(worldA.GetContactCount(), worldB.GetContactCount());
    const std::vector<float> snapshotA = SnapshotPhysicsState(worldA);
    const std::vector<float> snapshotB = SnapshotPhysicsState(worldB);
    ASSERT_EQ(snapshotA.size(), snapshotB.size());
    const int32_t firstMismatch = FindFirstBitMismatch(snapshotA, snapshotB);
    if (firstMismatch >= 0) {
        std::printf("[world5] determinism: first differing float at flat index %d: A = %.9g, B = %.9g\n",
                    static_cast<int>(firstMismatch),
                    static_cast<double>(snapshotA[static_cast<std::size_t>(firstMismatch)]),
                    static_cast<double>(snapshotB[static_cast<std::size_t>(firstMismatch)]));
    }
    std::printf("[world5] determinism: %d state floats compared, first mismatch = %d\n",
                static_cast<int>(snapshotA.size()), static_cast<int>(firstMismatch));
    EXPECT_EQ(firstMismatch, -1);
}

TEST(WorldStabilityTest, WarmStartFactorComparison) {
    // P09 D4 warm-start trust knob, measured on the hardest scene: the 10-box
    // tower at the default 8 velocity iterations. A factor f seeds the solver with
    // f * last frame's accumulated impulse, so the solver has to re-resolve the
    // (1 - f) deficit every frame; for a tall stack that re-injects the
    // constraint-ordering bias that warm starting is supposed to suppress.
    // MEASURED (Week 5 solver, 600 steps, collapse step):
    //   f = 0.00 -> 258   (Week 4 behaviour: no inherited solution, cannot stand)
    //   f = 0.50 -> 314
    //   f = 0.85 -> 259
    //   f = 0.90 -> 283
    //   f = 0.95 -> 332
    //   f = 1.00 -> no collapse (shipped default; full warm starting, like Box2D)
    // The intermediate factors are deliberately NOT asserted on a specific frame:
    // the exact collapse frame is chaotic. What is asserted is the two ends of the
    // contract: no seed cannot stand, full seed stands at the default iterations.
    struct WarmStartRow {
        float factor;
        TowerMetrics metrics;
    };
    std::vector<WarmStartRow> rows;
    const float factors[] = {0.0f, 0.5f, 0.85f, 0.9f, 0.95f, 1.0f};
    for (const float factor : factors) {
        World world;
        const TowerScene scene = BuildTower(world);
        SolverSettings settings = world.GetSolverSettings();
        settings.warmStartFactor = factor;
        world.SetSolverSettings(settings);
        rows.push_back(WarmStartRow{factor, RunTower(world, scene, kSimulationSteps)});
    }

    std::printf("[world5] warmStartFactor comparison (10 boxes, 600 steps, default 8/3 iterations):\n");
    std::printf("[world5]   factor | max pen (m) | final max speed (m/s) | jitter speed (m/s) | "
                "final max |x| (m) | top y (m) | collapse step\n");
    for (const WarmStartRow& row : rows) {
        std::printf("[world5]   %6.2f | %11.6f | %21.6f | %18.6f | %16.6f | %9.5f | %d\n",
                    static_cast<double>(row.factor), static_cast<double>(row.metrics.maxPenetration),
                    static_cast<double>(row.metrics.finalMaxSpeed),
                    static_cast<double>(row.metrics.jitterMaxSpeed),
                    static_cast<double>(row.metrics.finalMaxLateral),
                    static_cast<double>(row.metrics.finalTopY), static_cast<int>(row.metrics.collapseStep));
    }

    // Row index of factor == 1.0 (the shipped default, see Settings.h).
    const std::size_t fullSeed = static_cast<std::size_t>(std::size(rows)) - 1U;
    EXPECT_FLOAT_EQ(rows[fullSeed].factor, 1.0f);

    // Full warm starting (the default) must satisfy the milestone: the 10-box
    // tower stands for 10 s at the default iteration counts.
    EXPECT_LT(rows[fullSeed].metrics.maxPenetration, kMilestoneMaxPenetration);
    EXPECT_EQ(rows[fullSeed].metrics.collapseStep, -1);
    EXPECT_NEAR(rows[fullSeed].metrics.finalTopY, kTopBoxStartY, kMaxAllowedDrift);
    EXPECT_LT(rows[fullSeed].metrics.finalMaxTilt, kMaxAllowedTilt);
    EXPECT_LT(rows[fullSeed].metrics.finalMaxLateral, kMaxAllowedDrift);

    // Missing/partial warm starting cannot hold the tower for 600 steps at the
    // default 8 iterations (W4's root cause: no inherited solution -> the PGS
    // ordering bias is re-converged from zero every frame).
    EXPECT_GE(rows[0].metrics.collapseStep, 0); // factor 0.0
    EXPECT_GE(rows[1].metrics.collapseStep, 0); // factor 0.5
    EXPECT_GT(rows[0].metrics.maxPenetration, rows[fullSeed].metrics.maxPenetration);
}

TEST(WorldStabilityTest, SplitImpulseOnOffComparison) {
    // Split impulse (on) vs the Baumgarte-bias baseline (off). Both stabilise
    // penetration, but Baumgarte puts the correction into the REAL velocity state
    // through the velocity constraint; that mechanism difference is pinned
    // deterministically by the deep-overlap test below, while this table records
    // the tower-level data for the devlog.
    // MEASURED (Week 5 solver, 600 steps, default warm-start factor 1.0):
    //   split impulse on : max penetration 0.00522 m, jitter 0.0248 m/s, stands.
    //   split impulse off: max penetration 0.00527 m, jitter 0.0282 m/s, stands.
    // With full warm starting both arms keep the tower upright, so the tower-level
    // difference is small; the energy-injection difference is isolated by the
    // deep-overlap test below. Only the split-impulse arm is asserted on the
    // milestone; the Baumgarte row is data for the devlog.
    struct SplitImpulseRow {
        bool enabled;
        TowerMetrics metrics;
    };
    std::vector<SplitImpulseRow> rows;
    for (const bool enabled : {true, false}) {
        World world;
        const TowerScene scene = BuildTower(world);
        SolverSettings settings = world.GetSolverSettings();
        settings.useSplitImpulse = enabled;
        world.SetSolverSettings(settings);
        rows.push_back(SplitImpulseRow{enabled, RunTower(world, scene, kSimulationSteps)});
    }

    std::printf("[world5] split impulse comparison (10 boxes, 600 steps, default 8/3 iterations):\n");
    std::printf("[world5]   split impulse | max pen (m) | jitter speed (m/s) | final max speed (m/s) | "
                "top y (m) | collapse step\n");
    for (const SplitImpulseRow& row : rows) {
        std::printf("[world5]   %-13s | %11.6f | %18.6f | %22.6f | %9.5f | %d\n",
                    row.enabled ? "on" : "off (Baumgarte)", static_cast<double>(row.metrics.maxPenetration),
                    static_cast<double>(row.metrics.jitterMaxSpeed),
                    static_cast<double>(row.metrics.finalMaxSpeed),
                    static_cast<double>(row.metrics.finalTopY), static_cast<int>(row.metrics.collapseStep));
    }

    // Split impulse on: the milestone penetration budget and a standing tower.
    EXPECT_LT(rows[0].metrics.maxPenetration, kMilestoneMaxPenetration);
    EXPECT_EQ(rows[0].metrics.collapseStep, -1);
    EXPECT_LT(rows[0].metrics.jitterMaxSpeed, kRestingSpeedTolerance);
}

TEST(WorldStabilityTest, SplitImpulseDoesNotInjectEnergyIntoRealVelocities) {
    // The DoD item "split impulse 开启时不向真实速度注入能量", pinned with a scene
    // where the mechanism difference is deterministic instead of chaotic:
    //
    //   a 1 m box starts with its bottom 0.10 m inside the ground (centre y = 0.4,
    //   resting height 0.5), zero velocity, gravity on, e = 0.
    //
    // Split impulse: the velocity solver only removes the approach velocity, and
    // the overlap is removed by the pseudo-velocity layer (never written back to
    // the real velocities). The box may not acquire a real upward velocity.
    //
    // Baumgarte (useSplitImpulse = false): the overlap becomes a separating
    // velocity target in the SAME velocity constraint,
    //   target = (beta/dt) * max(0, penetration - slop)
    //          = (0.2/0.0166667) * (0.10 - 0.005) = 1.14 m/s (hand computed),
    // so the box is kicked upward with a real velocity - that is the energy
    // injection split impulse exists to avoid.
    struct EnergyRow {
        bool enabled;
        float maxUpwardVelocity;
        float finalY;
        float finalSpeed;
    };
    std::vector<EnergyRow> rows;
    for (const bool enabled : {true, false}) {
        World world;
        const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
        EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
        const BodyHandle box = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.4f)));
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(kBoxHalfExtent, kBoxHalfExtent)));
        SolverSettings settings = world.GetSolverSettings();
        settings.useSplitImpulse = enabled;
        world.SetSolverSettings(settings);

        float maxUpwardVelocity = 0.0f;
        for (int32_t step = 0; step < 240; ++step) {
            world.Step(kDt);
            maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(box).y);
        }
        rows.push_back(EnergyRow{enabled, maxUpwardVelocity, world.GetPosition(box).y,
                                 p2::Length(world.GetLinearVelocity(box))});
        std::printf("[world5] deep overlap (0.10 m), split impulse %-3s: max upward velocity = %.6f m/s, "
                    "final y = %.5f m, final speed = %.6f m/s\n",
                    enabled ? "on" : "off", static_cast<double>(maxUpwardVelocity),
                    static_cast<double>(world.GetPosition(box).y),
                    static_cast<double>(p2::Length(world.GetLinearVelocity(box))));
    }

    // Split impulse manufactures no upward velocity: the correction is carried by
    // the pseudo-velocity layer only.
    EXPECT_LT(rows[0].maxUpwardVelocity, 0.01f);
    // Baumgarte converts the same overlap into real velocity (hand-computed
    // target 1.14 m/s); assert the kick is clearly present.
    EXPECT_GT(rows[1].maxUpwardVelocity, 0.5f);
    // Both variants eventually rest in the slop band (penetration handled either
    // way; split impulse just does it without touching the real velocity state).
    EXPECT_NEAR(rows[0].finalY, kBoxHalfExtent - p2::kLinearSlop, 0.01f);
    EXPECT_NEAR(rows[1].finalY, kBoxHalfExtent - p2::kLinearSlop, 0.01f);
}

TEST(WorldStabilityTest, DisablingWarmStartingEqualsZeroFactor) {
    // Contract: `useWarmStarting = false` must be the same physics as
    // `warmStartFactor = 0` (both seed the accumulators with zero). Anything that
    // branches differently between the two paths would be hidden state.
    World disabledWorld;
    World zeroFactorWorld;
    const TowerScene disabledScene = BuildTower(disabledWorld);
    const TowerScene zeroFactorScene = BuildTower(zeroFactorWorld);
    ASSERT_EQ(disabledScene.boxes.size(), zeroFactorScene.boxes.size());

    SolverSettings disabledSettings = disabledWorld.GetSolverSettings();
    disabledSettings.useWarmStarting = false;
    disabledWorld.SetSolverSettings(disabledSettings);

    SolverSettings zeroFactorSettings = zeroFactorWorld.GetSolverSettings();
    zeroFactorSettings.warmStartFactor = 0.0f; // useWarmStarting stays true
    zeroFactorWorld.SetSolverSettings(zeroFactorSettings);

    StepFor(disabledWorld, kSimulationSteps);
    StepFor(zeroFactorWorld, kSimulationSteps);

    const std::vector<float> disabledSnapshot = SnapshotPhysicsState(disabledWorld);
    const std::vector<float> zeroFactorSnapshot = SnapshotPhysicsState(zeroFactorWorld);
    ASSERT_EQ(disabledSnapshot.size(), zeroFactorSnapshot.size());
    const int32_t firstMismatch = FindFirstBitMismatch(disabledSnapshot, zeroFactorSnapshot);
    std::printf("[world5] useWarmStarting=false vs warmStartFactor=0: %d state floats compared, "
                "first mismatch = %d (contact counts %d / %d)\n",
                static_cast<int>(disabledSnapshot.size()), static_cast<int>(firstMismatch),
                static_cast<int>(disabledWorld.GetContactCount()),
                static_cast<int>(zeroFactorWorld.GetContactCount()));
    EXPECT_EQ(firstMismatch, -1);
    EXPECT_EQ(disabledWorld.GetContactCount(), zeroFactorWorld.GetContactCount());
}

} // namespace
