// Week 7 (P13 / Section 6.5): whole-island atomic sleeping and waking.
//
// Contract pinned here at the World level:
//  - qualification: a dynamic island sleeps only when EVERY member has stayed
//    below kSleepLinearTolerance / kSleepAngularTolerance for longer than
//    kSleepTimeThreshold. One moving (or allowSleep=false) member keeps the whole
//    island awake - sleeping is atomic, never per body;
//  - freezing: a sleeping body keeps its pose, loses its real and pseudo
//    velocities, and its contacts (manifold + accumulated impulses) are reused
//    unchanged, so further steps are BIT-IDENTICAL while it sleeps;
//  - skipping: sleeping bodies keep their broad-phase proxies (needed for the
//    proximity wake / contact reuse) but skip the AABB refresh, the narrow phase
//    and the solve;
//  - waking: an external impulse / SetLinearVelocity / SetTransform / ApplyForce
//    on ANY member wakes the WHOLE island (condition 1); an awake dynamic or
//    kinematic body whose fat AABB touches a sleeping island wakes it through the
//    broad phase (condition 2) and the O(N^2) fallback does the same inside
//    CreateContactForPair;
//  - a 12-domino chain wakes and topples domino by domino - no domino is skipped.
//
// [world7] printf lines are the measured data; comments quote them.

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
#include <chrono>
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
constexpr int32_t kTowerBoxCount = 10;
constexpr int32_t kFrozenSteps = 100; // "sleep, then 100 further steps unchanged"
constexpr int32_t kSettleMaxSteps = 900;
constexpr int32_t kRestingWindowSteps = 60;

BodyDef MakeBodyDef(BodyType type, Vec2 position, bool allowSleep = true, float density = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.allowSleep = allowSleep;
    def.density = density;
    return def;
}

BodyHandle AddGroundPlane(World& world) {
    const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
    return ground;
}

BodyHandle AddBox(World& world, Vec2 position, float halfWidth = 0.5f, float halfHeight = 0.5f,
                  bool allowSleep = true) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position, allowSleep));
    EXPECT_TRUE(world.AttachShape(body, world.CreateBoxShape(halfWidth, halfHeight)));
    return body;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

struct TowerScene {
    BodyHandle ground;
    std::vector<BodyHandle> boxes;
};

TowerScene BuildTower(World& world, int32_t boxCount = kTowerBoxCount, bool allowSleep = true) {
    TowerScene scene;
    scene.ground = AddGroundPlane(world);
    scene.boxes.reserve(static_cast<std::size_t>(boxCount));
    for (int32_t index = 0; index < boxCount; ++index) {
        scene.boxes.push_back(
            AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index)), 0.5f, 0.5f, allowSleep));
    }
    return scene;
}

int32_t CountDynamicBodies(const World& world) {
    int32_t count = 0;
    for (const Body& body : world.GetBodies()) {
        if (body.handle.IsValid() && body.type == BodyType::Dynamic) {
            ++count;
        }
    }
    return count;
}

// Steps until every dynamic body is asleep; returns the step count or -1.
int32_t SettleUntilSleeping(World& world, int32_t maxSteps = kSettleMaxSteps) {
    const int32_t dynamicCount = CountDynamicBodies(world);
    for (int32_t step = 0; step < maxSteps; ++step) {
        world.Step(kDt);
        if (world.GetSleepingBodyCount() == dynamicCount && world.GetAwakeBodyCount() == 0) {
            return step + 1;
        }
    }
    return -1;
}

// Flattens the observable physics state: every live body's hot state plus the
// complete contact list (manifold + accumulated impulses). aabb/proxyId are
// deliberately excluded from the FROZEN comparison - Synchronize keeps recomputing
// the same AABB and the proxy id is engine bookkeeping, not physics; positions,
// rotations, velocities, pseudo velocities, contact impulses and island ids are
// all included.
std::vector<float> SnapshotPhysicsState(const World& world) {
    std::vector<float> values;
    const std::vector<Body>& bodies = world.GetBodies();
    values.reserve(bodies.size() * 15U + world.GetContacts().size() * 32U);
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
        values.push_back(body.isAwake ? 1.0f : 0.0f);
        values.push_back(static_cast<float>(body.islandId));
    }
    for (const Contact& contact : world.GetContacts()) {
        values.push_back(static_cast<float>(contact.bodyA.index));
        values.push_back(static_cast<float>(contact.bodyB.index));
        values.push_back(static_cast<float>(contact.manifold.pointCount));
        values.push_back(contact.manifold.normal.x);
        values.push_back(contact.manifold.normal.y);
        values.push_back(contact.manifold.penetration);
        values.push_back(contact.friction);
        values.push_back(contact.restitution);
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

// Exact-zero check for the frozen velocities: UpdateSleep must clear the real and
// pseudo velocities, not merely let them decay.
bool HasZeroVelocity(const Body& body) {
    return body.linearVelocity == Vec2{0.0f, 0.0f} && body.angularVelocity == 0.0f &&
           body.pseudoLinearVelocity == Vec2{0.0f, 0.0f} && body.pseudoAngularVelocity == 0.0f;
}

bool AllBoxesAsleep(const World& world, const TowerScene& scene) {
    for (const BodyHandle box : scene.boxes) {
        const Body* body = world.GetBody(box);
        if (body == nullptr || body->isAwake) {
            return false;
        }
    }
    return true;
}

bool AllBoxesAwake(const World& world, const TowerScene& scene) {
    for (const BodyHandle box : scene.boxes) {
        const Body* body = world.GetBody(box);
        if (body == nullptr || !body->isAwake) {
            return false;
        }
    }
    return true;
}

TEST(WorldSleepTest, TenBoxTowerSleepsAtomicallyAndStaysFrozen) {
    // The W7 integration milestone: the ten-box tower settles, the WHOLE island
    // goes to sleep, and 100 further steps are bit-identical (poses, velocities,
    // pseudo velocities, contact manifolds and accumulated impulses).
    //
    // Sleep onset is measured at step 859, not at the 600-step "stable" mark:
    // the resting jitter of the point-by-point PGS spikes periodically above
    // kSleepLinearTolerance (W5 measured jitter max 0.0248 m/s vs the 0.01 m/s
    // threshold), so the island needs a genuinely quiet 0.5 s window before it
    // qualifies. kSleepLinearTolerance deliberately stays at the design/Box2D
    // value (0.01 m/s) - lowering the bar to make the tower sleep at 600 steps
    // would hide the real residual and is not a fix. The correct fix is the
    // Week 8 2x2 block solver for two-point contacts; this longer window is the
    // honest current-solver result and the 900-step settle budget documents it.
    World world;
    world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    const TowerScene scene = BuildTower(world);

    const int32_t sleepStep = SettleUntilSleeping(world);
    ASSERT_GE(sleepStep, 0) << "the ten-box tower did not fall asleep within " << kSettleMaxSteps << " steps";
    EXPECT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    EXPECT_TRUE(AllBoxesAsleep(world, scene));
    for (const BodyHandle box : scene.boxes) {
        const Body* body = world.GetBody(box);
        ASSERT_NE(body, nullptr);
        EXPECT_TRUE(HasZeroVelocity(*body));
    }

    // The sleeping bodies keep their broad-phase proxies: that is what lets the
    // broad phase still report their contacts and the proximity-wake plan work.
    ASSERT_NE(world.GetBroadPhase(), nullptr);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), kTowerBoxCount + 1); // ground + boxes
    for (const BodyHandle box : scene.boxes) {
        const Body* body = world.GetBody(box);
        ASSERT_NE(body, nullptr);
        EXPECT_GE(body->proxyId, 0);
    }

    const std::vector<float> frozenSnapshot = SnapshotPhysicsState(world);
    const int32_t contactCountAtSleep = world.GetContactCount();
    const float topYAtSleep = world.GetPosition(scene.boxes.back()).y;
    StepFor(world, kFrozenSteps);
    ASSERT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    const std::vector<float> afterSnapshot = SnapshotPhysicsState(world);
    ASSERT_EQ(frozenSnapshot.size(), afterSnapshot.size());
    const int32_t firstMismatch = FindFirstBitMismatch(frozenSnapshot, afterSnapshot);
    if (firstMismatch >= 0) {
        std::printf("[world7] frozen-state mismatch at float %d: before = %.9g, after = %.9g\n",
                    static_cast<int>(firstMismatch),
                    static_cast<double>(frozenSnapshot[static_cast<std::size_t>(firstMismatch)]),
                    static_cast<double>(afterSnapshot[static_cast<std::size_t>(firstMismatch)]));
    }
    EXPECT_EQ(firstMismatch, -1);
    EXPECT_EQ(world.GetContactCount(), contactCountAtSleep);
    EXPECT_EQ(world.GetPosition(scene.boxes.back()).y, topYAtSleep);

    std::printf("[world7] tower sleep: slept at step %d, sleeping = %d, awake = %d, contacts = %d, "
                "top y = %.5f, %d frozen steps bit-identical = %s\n",
                static_cast<int>(sleepStep), static_cast<int>(world.GetSleepingBodyCount()),
                static_cast<int>(world.GetAwakeBodyCount()), static_cast<int>(contactCountAtSleep),
                static_cast<double>(topYAtSleep), static_cast<int>(kFrozenSteps),
                firstMismatch < 0 ? "yes" : "NO");
}

TEST(WorldSleepTest, SleepingDisabledKeepsEveryBodyAwake) {
    // The A/B control: with enableSleeping = false the same tower never sleeps and
    // keeps paying for the continuous solve. The Step time difference is printed
    // (debug build, so the absolute numbers matter less than the direction).
    World sleepingWorld;
    sleepingWorld.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    const TowerScene sleepingScene = BuildTower(sleepingWorld);
    const int32_t sleepStep = SettleUntilSleeping(sleepingWorld, kSettleMaxSteps);
    ASSERT_GE(sleepStep, 0);
    ASSERT_TRUE(AllBoxesAsleep(sleepingWorld, sleepingScene));

    World awakeWorld;
    awakeWorld.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    SolverSettings awakeSettings = awakeWorld.GetSolverSettings();
    awakeSettings.enableSleeping = false;
    awakeWorld.SetSolverSettings(awakeSettings);
    const TowerScene awakeScene = BuildTower(awakeWorld);
    StepFor(awakeWorld, sleepStep); // same simulated time, but still fully dynamic
    ASSERT_EQ(awakeWorld.GetSleepingBodyCount(), 0);
    ASSERT_EQ(awakeWorld.GetAwakeBodyCount(), kTowerBoxCount);
    EXPECT_TRUE(AllBoxesAwake(awakeWorld, awakeScene));

    // Time the settled phase: the sleeping world does no solve, the awake world
    // still runs warm start + 8 iterations + the split-impulse layer on 10 bodies.
    const auto timeSteps = [](World& timedWorld, int32_t steps) {
        const auto start = std::chrono::steady_clock::now();
        StepFor(timedWorld, steps);
        const auto end = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::micro>(end - start).count() / steps;
    };
    constexpr int32_t kTimedSteps = 300;
    const double sleepingMicros = timeSteps(sleepingWorld, kTimedSteps);
    const double awakeMicros = timeSteps(awakeWorld, kTimedSteps);
    const double sleepingMicrosFloor = std::max(sleepingMicros, 1.0e-6);
    for (const BodyHandle box : sleepingScene.boxes) {
        EXPECT_FALSE(sleepingWorld.IsAwake(box));
    }
    for (const BodyHandle box : awakeScene.boxes) {
        EXPECT_TRUE(awakeWorld.IsAwake(box));
    }
    std::printf("[world7] sleeping on/off: sleeping = %d bodies, awake = %d bodies; "
                "settled Step time = %.1f us/step (asleep) vs %.1f us/step (awake), speedup = %.2fx\n",
                static_cast<int>(sleepingWorld.GetSleepingBodyCount()),
                static_cast<int>(awakeWorld.GetSleepingBodyCount()), sleepingMicros, awakeMicros,
                awakeMicros / sleepingMicrosFloor);
}

TEST(WorldSleepTest, ImpulseWakesTheWholeIslandAndItResponds) {
    World world;
    world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    const TowerScene scene = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_TRUE(AllBoxesAsleep(world, scene));

    const BodyHandle target = scene.boxes[5];
    const Vec2 before = world.GetPosition(target);
    world.ApplyLinearImpulse(target, Vec2(0.3f, 0.0f), world.GetPosition(target));

    // Atomic wake: every member is awake immediately, before any step runs.
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_TRUE(AllBoxesAwake(world, scene));
    // Only the target received the impulse; the other members restart from zero.
    for (std::size_t index = 0; index < scene.boxes.size(); ++index) {
        const Body* body = world.GetBody(scene.boxes[index]);
        ASSERT_NE(body, nullptr);
        if (static_cast<int32_t>(index) == 5) {
            EXPECT_GT(body->linearVelocity.x, 0.0f);
        } else {
            EXPECT_TRUE(HasZeroVelocity(*body));
        }
    }
    StepFor(world, 10);
    EXPECT_GT(world.GetPosition(target).x, before.x + 1.0e-3f);
    EXPECT_GT(world.GetContactCount(), 0);
    std::printf("[world7] impulse wake: sleeping -> %d, all awake = %s, target dx after 10 steps "
                "= %.5f m\n",
                static_cast<int>(world.GetSleepingBodyCount()), AllBoxesAwake(world, scene) ? "yes" : "no",
                static_cast<double>(world.GetPosition(target).x - before.x));
}

TEST(WorldSleepTest, SetLinearVelocityWakesTheWholeIsland) {
    World world;
    const TowerScene scene = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_TRUE(AllBoxesAsleep(world, scene));

    const BodyHandle target = scene.boxes[3];
    world.SetLinearVelocity(target, Vec2(0.5f, 0.0f));
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_TRUE(AllBoxesAwake(world, scene));
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(target).x, 0.5f);
    const float startX = world.GetPosition(target).x;
    StepFor(world, 5);
    EXPECT_GT(world.GetPosition(target).x, startX);
    std::printf("[world7] SetLinearVelocity wake: all awake = %s, dx after 5 steps = %.5f m\n",
                AllBoxesAwake(world, scene) ? "yes" : "no",
                static_cast<double>(world.GetPosition(target).x - startX));
}

TEST(WorldSleepTest, SetTransformWakesTheWholeIsland) {
    World world;
    const TowerScene scene = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_TRUE(AllBoxesAsleep(world, scene));

    const BodyHandle target = scene.boxes[7];
    const Vec2 targetPosition = world.GetPosition(target) + Vec2(0.5f, 0.2f);
    world.SetTransform(target, targetPosition, 0.1f);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_TRUE(AllBoxesAwake(world, scene));
    EXPECT_EQ(world.GetPosition(target), targetPosition);
    StepFor(world, 1);
    // The teleport must not be rolled back: the woken body keeps simulating from
    // the new pose (gravity alone is enough to make its position differ a step
    // later, so this only pins "the teleport stuck and the island is alive").
    EXPECT_TRUE(world.GetPosition(target) != targetPosition);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    std::printf("[world7] SetTransform wake: all awake = %s, teleported to (%.3f, %.3f)\n",
                AllBoxesAwake(world, scene) ? "yes" : "no", static_cast<double>(world.GetPosition(target).x),
                static_cast<double>(world.GetPosition(target).y));
}

TEST(WorldSleepTest, ApplyForceWakesTheWholeIsland) {
    World world;
    const TowerScene scene = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_TRUE(AllBoxesAsleep(world, scene));

    const BodyHandle target = scene.boxes[2];
    // F = 10 N on a 1 kg box gives a = 10 m/s^2 for one step -> dv = g*dt worth of
    // horizontal velocity, clearly above the sleep tolerance after the step.
    world.ApplyForce(target, Vec2(10.0f, 0.0f), world.GetPosition(target));
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_TRUE(AllBoxesAwake(world, scene));
    StepFor(world, 1);
    EXPECT_GT(world.GetLinearVelocity(target).x, 0.01f);
    std::printf("[world7] ApplyForce wake: all awake = %s, target vx after 1 step = %.4f m/s\n",
                AllBoxesAwake(world, scene) ? "yes" : "no",
                static_cast<double>(world.GetLinearVelocity(target).x));
}

TEST(WorldSleepTest, AwakeBodyFatAabbProximityWakesSleepingIsland) {
    // P13 wake condition 2, tested on both pair sources:
    //  - installed broad phase: the proximity wake lives in UpdateBroadPhase and
    //    runs before the narrow phase, so a body that merely ENTERS the sleeping
    //    island's fat AABB (no geometric contact yet) wakes the whole island;
    //  - O(N^2) fallback: no pair list exists, so CreateContactForPair performs the
    //    identical check.
    // The probe is a 0.4 m box placed at x = 0.75: its tight AABB starts at 0.55,
    // the settled tower's fat AABB ends at ~0.6 -> the FAT bounds overlap by ~0.05
    // while the shapes stay 0.05 m apart (verified below: no contact with the
    // probe). That isolates proximity wake from contact wake.
    for (const bool useBroadPhase : {true, false}) {
        World world;
        if (useBroadPhase) {
            world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
        }
        const TowerScene scene = BuildTower(world);
        ASSERT_GE(SettleUntilSleeping(world), 0)
            << "tower did not sleep (broad phase = " << (useBroadPhase ? "on" : "off") << ")";
        ASSERT_TRUE(AllBoxesAsleep(world, scene));
        const int32_t contactCountBeforeProbe = world.GetContactCount();

        const BodyHandle probe = AddBox(world, Vec2(0.75f, 9.5f), 0.2f, 0.2f);
        world.Step(kDt);

        EXPECT_TRUE(AllBoxesAwake(world, scene))
            << "proximity wake failed (broad phase = " << (useBroadPhase ? "on" : "off") << ")";
        EXPECT_EQ(world.GetSleepingBodyCount(), 0);
        // The probe overlaps no tower geometry yet: the wake was purely the fat
        // AABB proximity, not a contact.
        EXPECT_EQ(world.GetContactCount(), contactCountBeforeProbe);
        for (const Contact& contact : world.GetContacts()) {
            EXPECT_NE(contact.bodyA, probe);
            EXPECT_NE(contact.bodyB, probe);
        }
        std::printf("[world7] proximity wake (broad phase %s): tower woke with probe at (%.3f, %.3f), "
                    "contacts = %d (unchanged from %d)\n",
                    useBroadPhase ? "on" : "off", static_cast<double>(world.GetPosition(probe).x),
                    static_cast<double>(world.GetPosition(probe).y),
                    static_cast<int>(world.GetContactCount()), static_cast<int>(contactCountBeforeProbe));
    }
}

TEST(WorldSleepTest, TwelveDominoChainWakesAndTopplesEveryDomino) {
    // A sleeping row of dominoes: each piece is its own island (only the static
    // ground touches it), so a push must wake them one at a time through contact /
    // proximity wake as the wave travels - no domino may be skipped.
    //
    // Geometry note (measured): 0.12 m wide pieces (half width 0.06) never reach
    // the sleep thresholds in this solver - their resting jitter periodically
    // spikes to ~0.015 m/s against kSleepLinearTolerance = 0.01 m/s (the
    // point-by-point PGS leaves more residual torque on low-inertia pieces), so
    // the row would stay awake forever. A 0.30 m wide piece (half width 0.15)
    // qualifies after 32 steps (measured) and still reads/behaves as a domino
    // (aspect 1:3.3); the 12/12 chain works with the top-edge impulse below. The
    // thin-piece residual is a block-solver improvement item, NOT a sleeping-rule
    // workaround - documented as a known W7 limitation.
    World world;
    world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    AddGroundPlane(world);
    constexpr int32_t kDominoCount = 12;
    constexpr float kDominoHalfWidth = 0.15f;
    constexpr float kDominoHalfHeight = 0.5f;
    constexpr float kDominoSpacing = 0.65f;
    // Measured top-edge impulse sweep (spacing 0.65): 0.20-0.30 N*s all give
    // 12/12 awake and toppled; 0.25 is the middle of the working band.
    constexpr float kPushImpulse = 0.25f;
    std::vector<BodyHandle> dominoes;
    dominoes.reserve(static_cast<std::size_t>(kDominoCount));
    for (int32_t index = 0; index < kDominoCount; ++index) {
        dominoes.push_back(AddBox(world, Vec2(kDominoSpacing * static_cast<float>(index), kDominoHalfHeight),
                                  kDominoHalfWidth, kDominoHalfHeight));
    }
    const int32_t sleepStep = SettleUntilSleeping(world);
    ASSERT_GE(sleepStep, 0) << "domino row did not sleep";
    ASSERT_EQ(world.GetSleepingBodyCount(), kDominoCount);
    ASSERT_EQ(world.GetAwakeBodyCount(), 0);
    ASSERT_EQ(world.GetIslandCount(), kDominoCount); // each domino is a singleton island

    // Push near the top edge: a pure angular velocity would be braked by the
    // ground contact at the bottom edge, so the impulse is applied at 0.95 of the
    // half height. This wakes the first island (a single body) and creates the
    // torque that tips it onto its neighbour.
    const Vec2 topPoint = world.GetPosition(dominoes.front()) + Vec2(0.0f, kDominoHalfHeight * 0.95f);
    world.ApplyLinearImpulse(dominoes.front(), Vec2(kPushImpulse, 0.0f), topPoint);
    EXPECT_EQ(world.GetSleepingBodyCount(), kDominoCount - 1); // only the pushed island woke

    std::vector<uint8_t> everAwake(static_cast<std::size_t>(kDominoCount), 0);
    std::vector<int32_t> firstWakeStep(static_cast<std::size_t>(kDominoCount), -1);
    int32_t maxAwake = 0;
    constexpr int32_t kChainSteps = 500;
    for (int32_t step = 0; step < kChainSteps; ++step) {
        world.Step(kDt);
        int32_t awakeNow = 0;
        for (std::size_t index = 0; index < dominoes.size(); ++index) {
            if (world.IsAwake(dominoes[index])) {
                ++awakeNow;
                if (everAwake[index] == 0) {
                    firstWakeStep[index] = step;
                }
                everAwake[index] = 1;
            }
        }
        maxAwake = std::max(maxAwake, awakeNow);
    }

    int32_t wokenCount = 0;
    int32_t toppledCount = 0;
    for (std::size_t index = 0; index < dominoes.size(); ++index) {
        wokenCount += everAwake[index] != 0 ? 1 : 0;
        if (std::fabs(world.GetAngle(dominoes[index])) > 0.5f) {
            ++toppledCount;
        }
    }
    // Chain propagation: every domino must have been woken by its predecessor.
    EXPECT_EQ(wokenCount, kDominoCount);
    EXPECT_EQ(maxAwake, kDominoCount);
    // The wake wave should be monotone along the row (the chain may take a step or
    // two per piece; ties are fine, e.g. two neighbours waking in the same frame).
    for (std::size_t index = 1; index < dominoes.size(); ++index) {
        EXPECT_GE(firstWakeStep[index], firstWakeStep[index - 1])
            << "wake wave skipped backwards at domino " << index;
    }
    // Physical outcome: the wave topples the row (a domino may re-sleep once it is
    // lying still, so the final awake count is not asserted - only the topple).
    EXPECT_GE(toppledCount, kDominoCount - 2);
    std::printf("[world7] dominoes: slept after %d steps; push woke 1; %d/%d woken, max "
                "simultaneously awake = %d, %d/%d toppled after %d steps, final awake = %d\n",
                static_cast<int>(sleepStep), static_cast<int>(wokenCount), static_cast<int>(kDominoCount),
                static_cast<int>(maxAwake), static_cast<int>(toppledCount), static_cast<int>(kDominoCount),
                static_cast<int>(kChainSteps), static_cast<int>(world.GetAwakeBodyCount()));
    std::printf("[world7] domino first-wake steps:");
    for (std::size_t index = 0; index < firstWakeStep.size(); ++index) {
        std::printf(" %d", static_cast<int>(firstWakeStep[index]));
    }
    std::printf("\n");
}

TEST(WorldSleepTest, NonSleepableBodyKeepsItsIslandAwake) {
    // allowSleep = false is the per-body control: a single non-sleepable member
    // keeps the WHOLE island awake (partial sleeping would break atomicity).
    // Body::allowSleep is set through BodyDef at creation - the public API is
    // frozen and has no setter, which is fine: allowSleep is a construction-time
    // decision like the body type.
    World world;
    AddGroundPlane(world);
    std::vector<BodyHandle> boxes;
    boxes.reserve(kTowerBoxCount);
    for (int32_t index = 0; index < kTowerBoxCount; ++index) {
        const bool canSleep = (index != 4); // one non-sleepable member in the stack
        boxes.push_back(AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index)), 0.5f, 0.5f, canSleep));
    }

    StepFor(world, kSettleMaxSteps); // long enough that a normal island would sleep
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), kTowerBoxCount);
    for (const BodyHandle box : boxes) {
        EXPECT_TRUE(world.IsAwake(box));
    }
    std::printf("[world7] allowSleep=false on one member: sleeping = %d, awake = %d "
                "(whole island kept awake)\n",
                static_cast<int>(world.GetSleepingBodyCount()), static_cast<int>(world.GetAwakeBodyCount()));
}

TEST(WorldSleepTest, DestroyBodyBetweenStepsLeavesAStalePartitionThatMustNotCrash) {
    // The island partition is a per-step snapshot, but the public API allows
    // DestroyBody/impulses BETWEEN steps. A body destroyed after Step k leaves its
    // slot in m_islandBodyIndices until BuildIslands runs again; WakeIslandOfBody
    // (called by the next external impulse) and UpdateSleep must tolerate the dead
    // member instead of indexing it (out-of-bounds / dead-slot writes).
    World world;
    world.SetBroadPhase(std::make_unique<p2::BroadPhaseBruteForce>());
    const TowerScene scene = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_TRUE(AllBoxesAsleep(world, scene));

    const BodyHandle doomed = scene.boxes[5];
    world.DestroyBody(doomed);
    ASSERT_FALSE(world.IsValid(doomed));
    // The stale partition still lists the dead slot. Waking through a live member
    // must skip it and wake the other members (no crash, no dead-slot write).
    world.ApplyLinearImpulse(scene.boxes[4], Vec2(0.05f, 0.0f), world.GetPosition(scene.boxes[4]));
    for (std::size_t index = 0; index < scene.boxes.size(); ++index) {
        if (static_cast<int32_t>(index) == 5) {
            continue; // destroyed
        }
        EXPECT_TRUE(world.IsAwake(scene.boxes[index]));
    }
    // The next Step rebuilds the partition without the dead slot.
    world.Step(kDt);
    EXPECT_EQ(world.GetBodyCount(), kTowerBoxCount); // ground + 9 remaining boxes
    EXPECT_GT(world.GetIslandCount(), 0);
    std::printf("[world7] stale island partition after DestroyBody: wake + step survived, "
                "bodies = %d, islands = %d\n",
                static_cast<int>(world.GetBodyCount()), static_cast<int>(world.GetIslandCount()));
}

} // namespace
