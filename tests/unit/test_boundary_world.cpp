// Week 8 (delivery week): World boundary / contract audit.
//
// The Week 1-7 suites already pin the *happy paths* (and parts of the error
// contract: shape validation, attached-shape destruction, body init NaN). This
// file deliberately tests the edges that the delivery checklist asks for and that
// the other suites do NOT already cover, so the project can truthfully claim
// "every known crash point has an assert or a documented behaviour":
//
//  1. lifecycle totality: DestroyBody twice, stale handles after slot reuse,
//     Clear() with a live broad phase + island partition + sleeping bodies, and
//     "old handles never alias the new body that reuses their slot";
//  2. Step input contract: illegal dt / iteration counts assert in Debug and are
//     refused (or clamped to a defined zero-iteration solve) in Release;
//  3. sleep edges: WakeBody (the external wake entry point) wakes the WHOLE
//     island, sleeping bodies keep their broad-phase proxies, and turning
//     `enableSleeping` off after the scene settled wakes every body again;
//  4. broad-phase edges: switching implementations while an island is asleep
//     rebuilds its proxies without waking it, and no/brute-force/SAP stay
//     bit-identical across a scene with create/destroy churn (not just a static
//     scene as the Week 6 test used);
//  5. core-state invariants: after every step islandId is in range, sleepTime is
//     finite/non-negative, pseudo velocities are cleared, sleeping dynamic bodies
//     have zero velocity - the same states World::CheckSane now scans (Week 8
//     addition in src/Core/World.cpp);
//  6. (Release only) the IslandSleep500 sleeping-on/sleeping-off Step-time
//     measurement, printed with the [boundary8] prefix so docs/profiling.md can
//     cite a command that reproduces it (`--gtest_filter=WorldBoundaryProfilingTest.*`).
//
// Timing numbers printed here are only quotable from a Release build; a Debug
// run of the same test is a smoke test (the printout says which build it was).

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::BroadPhaseBruteForce;
using p2::BroadPhaseSAP;
using p2::Contact;
using p2::ShapeHandle;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr int32_t kTowerBoxCount = 5;    // one tower = one island (ground is a boundary)
constexpr int32_t kSettleMaxSteps = 600; // 5-box towers settle around frame ~105

BodyDef MakeBodyDef(BodyType type, Vec2 position, bool allowSleep = true) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.allowSleep = allowSleep;
    def.density = 1.0f;
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

// A vertical stack of exactly touching 1 m boxes (same initial state as the
// IslandSleep500 towers and the Week 5/7 stability suites).
std::vector<BodyHandle> BuildTower(World& world, int32_t boxCount = kTowerBoxCount) {
    std::vector<BodyHandle> boxes;
    boxes.reserve(static_cast<std::size_t>(boxCount));
    for (int32_t index = 0; index < boxCount; ++index) {
        boxes.push_back(AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index))));
    }
    return boxes;
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
    if (dynamicCount == 0) {
        return -1;
    }
    for (int32_t step = 0; step < maxSteps; ++step) {
        world.Step(kDt);
        if (world.GetSleepingBodyCount() == dynamicCount && world.GetAwakeBodyCount() == 0) {
            return step + 1;
        }
    }
    return -1;
}

void AppendFloatBits(std::vector<uint32_t>& bits, float value) {
    bits.push_back(std::bit_cast<uint32_t>(value));
}

// Every physics float a World owns (body hot state + contact manifold including
// the accumulated impulses). Two Worlds running the same scene must produce
// identical vectors every step - that is the Week 6 "switching pair sources must
// not change the physics" contract checked under lifecycle churn below.
std::vector<uint32_t> SnapshotWorldPhysics(const World& world) {
    std::vector<uint32_t> bits;
    bits.reserve(world.GetBodies().size() * 10U + world.GetContacts().size() * 14U);
    for (const Body& body : world.GetBodies()) {
        if (!body.handle.IsValid()) {
            continue;
        }
        AppendFloatBits(bits, body.xf.p.x);
        AppendFloatBits(bits, body.xf.p.y);
        AppendFloatBits(bits, body.xf.q.c);
        AppendFloatBits(bits, body.xf.q.s);
        AppendFloatBits(bits, body.linearVelocity.x);
        AppendFloatBits(bits, body.linearVelocity.y);
        AppendFloatBits(bits, body.angularVelocity);
        AppendFloatBits(bits, body.pseudoLinearVelocity.x);
        AppendFloatBits(bits, body.pseudoLinearVelocity.y);
        AppendFloatBits(bits, body.pseudoAngularVelocity);
    }
    for (const Contact& contact : world.GetContacts()) {
        bits.push_back(static_cast<uint32_t>(contact.bodyA.index));
        bits.push_back(static_cast<uint32_t>(contact.bodyB.index));
        bits.push_back(static_cast<uint32_t>(contact.manifold.pointCount));
        AppendFloatBits(bits, contact.manifold.normal.x);
        AppendFloatBits(bits, contact.manifold.normal.y);
        AppendFloatBits(bits, contact.manifold.penetration);
        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            const p2::ContactPoint& point = contact.manifold.points[pointIndex];
            AppendFloatBits(bits, point.normalImpulse);
            AppendFloatBits(bits, point.tangentImpulse);
        }
    }
    return bits;
}

bool HasZeroVelocity(const Body& body) {
    return body.linearVelocity == Vec2{0.0f, 0.0f} && body.angularVelocity == 0.0f &&
           body.pseudoLinearVelocity == Vec2{0.0f, 0.0f} && body.pseudoAngularVelocity == 0.0f;
}

// ---------------------------------------------------------------------------
// 1. Lifecycle totality
// ---------------------------------------------------------------------------

TEST(WorldLifecycleBoundaryTest, DoubleDestroyAndStaleHandlesAreTotal) {
    World world(Vec2(0.0f, -10.0f));
    world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    const BodyHandle body = AddBox(world, Vec2(0.5f, 3.0f));
    ASSERT_TRUE(world.IsValid(body));
    const int32_t slotIndex = body.index;
    const uint32_t generation = body.generation;
    StepFor(world, 5);
    ASSERT_EQ(world.GetBodyCount(), 1);
    ASSERT_EQ(world.GetShapeCount(), 1);

    world.DestroyBody(body);
    EXPECT_FALSE(world.IsValid(body));
    EXPECT_EQ(world.GetBody(body), nullptr);
    EXPECT_EQ(world.GetBodyCount(), 0);
    // The body owns its shape: destruction releases both slots.
    EXPECT_EQ(world.GetShapeCount(), 0);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);

    // Destruction is idempotent (documented cleanup-path contract).
    world.DestroyBody(body);
    EXPECT_EQ(world.GetBodyCount(), 0);

    // Stale read accessors stay total: they return neutral defaults instead of
    // touching the released slot (UI panels poll handles for one frame).
    EXPECT_EQ(world.GetPosition(body), Vec2(0.0f, 0.0f));
    EXPECT_EQ(world.GetTransform(body).p, Vec2(0.0f, 0.0f));
    EXPECT_EQ(world.GetLinearVelocity(body), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetAngle(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetMass(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetInvMass(body), 0.0f);
    EXPECT_EQ(world.GetBodyType(body), BodyType::Static);
    EXPECT_FALSE(world.IsAwake(body));
    EXPECT_EQ(world.GetUserData(body), nullptr);

    // Stale mutating calls are no-ops (no GetBodyMutable assert fires on a
    // destroyed handle: the validity check runs first). This is the documented
    // "stale handles must safely fail" invariant.
    world.SetTransform(body, Vec2(9.0f, 9.0f), 1.0f);
    world.SetLinearVelocity(body, Vec2(5.0f, 5.0f));
    world.SetAngularVelocity(body, 3.0f);
    world.SetFriction(body, 0.9f);
    world.SetRestitution(body, 0.9f);
    world.ApplyForce(body, Vec2(100.0f, 100.0f), Vec2(0.0f, 0.0f));
    world.ApplyLinearImpulse(body, Vec2(100.0f, 100.0f), Vec2(0.0f, 0.0f));
    world.WakeBody(body);
    world.Step(kDt); // a world with only dead slots must step without crashing
    EXPECT_EQ(world.GetBodyCount(), 0);

    // Slot reuse: the new body gets the same index with a new generation, so the
    // old handle can never alias (ABA) it.
    const BodyHandle reused = AddBox(world, Vec2(-1.0f, 3.0f));
    ASSERT_TRUE(world.IsValid(reused));
    EXPECT_EQ(reused.index, slotIndex);
    EXPECT_NE(reused.generation, generation);
    EXPECT_EQ(reused.generation, generation + 1U);
    EXPECT_FALSE(world.IsValid(body));
    world.DestroyBody(body); // stale handle: must NOT destroy the reused body
    EXPECT_TRUE(world.IsValid(reused));
    EXPECT_EQ(world.GetBodyCount(), 1);
    const Vec2 reusedPosition = world.GetPosition(reused);

    world.SetTransform(body, Vec2(20.0f, 20.0f), 0.0f); // stale setter: no effect on `reused`
    EXPECT_EQ(world.GetPosition(reused), reusedPosition);
}

TEST(WorldLifecycleBoundaryTest, ClearDropsBodiesShapesProxiesAndIslandState) {
    World world;
    world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    AddGroundPlane(world);
    const std::vector<BodyHandle> boxes = BuildTower(world);
    const ShapeHandle spare = world.CreateCircleShape(0.25f);
    ASSERT_TRUE(world.IsValidShape(spare));
    ASSERT_EQ(world.GetBodyCount(), kTowerBoxCount + 1);
    ASSERT_EQ(world.GetShapeCount(), kTowerBoxCount + 2);

    // Sleep the tower first: Clear must also drop the solver's island partition
    // (non-owning pointers) and the broad-phase proxies of sleeping bodies.
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_EQ(world.GetAwakeBodyCount(), 0);

    world.Clear();
    EXPECT_EQ(world.GetBodyCount(), 0);
    EXPECT_EQ(world.GetShapeCount(), 0);
    EXPECT_EQ(world.GetContactCount(), 0);
    EXPECT_EQ(world.GetIslandCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);
    for (const BodyHandle box : boxes) {
        EXPECT_FALSE(world.IsValid(box));
        EXPECT_EQ(world.GetBody(box), nullptr);
    }
    EXPECT_FALSE(world.IsValidShape(spare));
    EXPECT_EQ(world.GetShape(spare), nullptr);

    // Stale destruction after Clear is a no-op, and Clear itself is idempotent.
    world.DestroyBody(boxes.front());
    world.DestroyShape(spare);
    world.Clear();
    EXPECT_EQ(world.GetBodyCount(), 0);

    // The world is immediately reusable and the slot space restarts at 0/1.
    const BodyHandle rebuilt = AddBox(world, Vec2(0.0f, 4.0f));
    ASSERT_TRUE(world.IsValid(rebuilt));
    EXPECT_EQ(rebuilt.index, 0);
    EXPECT_EQ(rebuilt.generation, 1U);
    StepFor(world, 10);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 1);
    EXPECT_EQ(world.GetBodyCount(), 1);
}

// ---------------------------------------------------------------------------
// 2. Step input contract (assert in Debug, defined behaviour in Release)
// ---------------------------------------------------------------------------

class WorldStepBoundaryDeathTest : public p2test::DeathTestFixture {};

TEST_F(WorldStepBoundaryDeathTest, ZeroDeltaTimeAsserts) {
    World world;
    EXPECT_DEATH(world.Step(0.0f), "dt must be finite and > 0");
}

TEST_F(WorldStepBoundaryDeathTest, NegativeAndNonFiniteDeltaTimeAsserts) {
    World world;
    EXPECT_DEATH(world.Step(-kDt), "dt must be finite and > 0");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    EXPECT_DEATH(world.Step(nan), "dt must be finite and > 0");
    EXPECT_DEATH(world.Step(infinity), "dt must be finite and > 0");
}

TEST_F(WorldStepBoundaryDeathTest, NegativeIterationCountsAssert) {
    World world;
    EXPECT_DEATH(world.Step(kDt, -1, 0), "iteration counts must be non-negative");
    EXPECT_DEATH(world.Step(kDt, 0, -2), "iteration counts must be non-negative");
}

#ifdef NDEBUG
TEST(WorldStepBoundaryTest, InvalidStepInputIsRefusedInReleaseBuilds) {
    // assert() is compiled out with NDEBUG, so the Release fallback must keep the
    // call total: illegal dt returns without touching the state, negative
    // iteration counts are clamped to zero (a defined "no solve iterations").
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(1.0f, 2.0f)));
    ASSERT_TRUE(world.IsValid(body));
    const Vec2 positionBefore = world.GetPosition(body);

    world.Step(0.0f);
    world.Step(-kDt);
    world.Step(std::numeric_limits<float>::quiet_NaN());
    world.Step(std::numeric_limits<float>::infinity());
    EXPECT_EQ(world.GetPosition(body), positionBefore);
    EXPECT_EQ(world.GetLinearVelocity(body), Vec2(0.0f, 0.0f));

    // Clamped iterations: the step itself still runs (gravity integrator), but the
    // solver budget is a defined zero instead of a negative count.
    world.Step(kDt, -5, -5);
    EXPECT_TRUE(std::isfinite(world.GetPosition(body).y));
    EXPECT_EQ(world.GetLinearVelocity(body), Vec2(0.0f, 0.0f)); // gravity is zero here
    EXPECT_EQ(world.GetPosition(body).x, positionBefore.x);
    world.CheckSane(); // the release fallback must not create NaN/Inf state

    // A per-call override of exactly zero is legal in every build and equals the
    // "no solver iterations" configuration (the API only requires non-negative).
    world.Step(kDt, 0, 0);
    EXPECT_TRUE(std::isfinite(world.GetPosition(body).y));
}
#endif // NDEBUG

// ---------------------------------------------------------------------------
// 3. Sleep boundary (proxy retention, whole-island wake, runtime toggle)
// ---------------------------------------------------------------------------

TEST(WorldSleepBoundaryTest, WakeBodyIsWholeIslandAndProxiesSurviveSleep) {
    World world;
    world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    const BodyHandle ground = AddGroundPlane(world);
    const std::vector<BodyHandle> boxes = BuildTower(world);

    const int32_t sleepStep = SettleUntilSleeping(world);
    ASSERT_GE(sleepStep, 0) << "5-box tower did not sleep within " << kSettleMaxSteps << " steps";
    EXPECT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    EXPECT_EQ(world.GetIslandCount(), 1); // the tower is one island; ground is a boundary

    // Sleeping bodies keep their broad-phase proxies (the proximity-wake and the
    // frozen-contact reuse both depend on it) and freeze with exact zero velocity.
    ASSERT_NE(world.GetBroadPhase(), nullptr);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), kTowerBoxCount + 1);
    EXPECT_GE(world.GetBody(ground)->proxyId, 0);
    for (const BodyHandle box : boxes) {
        const Body* body = world.GetBody(box);
        ASSERT_NE(body, nullptr);
        EXPECT_GE(body->proxyId, 0);
        EXPECT_TRUE(HasZeroVelocity(*body));
        EXPECT_FLOAT_EQ(body->sleepTime, 0.0f);
        EXPECT_GE(body->islandId, 0);
        EXPECT_LT(body->islandId, world.GetIslandCount());
    }
    world.CheckSane(); // Week 8: the validator accepts the frozen island state

    // WakeBody is the public external wake entry point: it must wake the WHOLE
    // island, not just the addressed body (a half-awake island would clip).
    world.WakeBody(boxes[2]);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), kTowerBoxCount);
    for (const BodyHandle box : boxes) {
        EXPECT_TRUE(world.IsAwake(box));
        EXPECT_FLOAT_EQ(world.GetBody(box)->sleepTime, 0.0f); // fresh qualification window
    }
    StepFor(world, 1);
    world.CheckSane();
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), kTowerBoxCount + 1);

    // The tower can fall asleep again, and a second settle keeps proxy count exact
    // (no proxy leak across sleep/wake cycles).
    EXPECT_GE(SettleUntilSleeping(world), 0);
    EXPECT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), kTowerBoxCount + 1);

    std::printf("[boundary8] 5-box tower: first sleep step = %d, proxies kept = %d, "
                "WakeBody woke %d bodies (whole island)\n",
                static_cast<int>(sleepStep), static_cast<int>(world.GetBroadPhase()->GetProxyCount()),
                static_cast<int>(kTowerBoxCount));
}

TEST(WorldSleepBoundaryTest, DisablingSleepingAfterSettleWakesTheIslandAgain) {
    // UpdateSleep documents that `enableSleeping = false` is a true sleeping-off
    // switch, not a permanent freeze: bodies already asleep must be woken when the
    // flag is cleared at runtime (testbed checkbox contract).
    World world;
    world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    AddGroundPlane(world);
    const std::vector<BodyHandle> boxes = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);

    p2::SolverSettings settings = world.GetSolverSettings();
    settings.enableSleeping = false;
    world.SetSolverSettings(settings);
    StepFor(world, 1);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), kTowerBoxCount);
    for (const BodyHandle box : boxes) {
        EXPECT_TRUE(world.IsAwake(box));
    }
    StepFor(world, 60);
    EXPECT_EQ(world.GetSleepingBodyCount(), 0); // never re-sleeps while disabled
    world.CheckSane();

    // Re-enable: the tower must be able to sleep again (state was not corrupted).
    settings.enableSleeping = true;
    world.SetSolverSettings(settings);
    EXPECT_GE(SettleUntilSleeping(world), 0);
    EXPECT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
}

// ---------------------------------------------------------------------------
// 4. Broad-phase edges
// ---------------------------------------------------------------------------

TEST(WorldBroadPhaseBoundaryTest, SwitchingBroadPhaseWhileAsleepPreservesSleepAndProxies) {
    World world;
    world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    AddGroundPlane(world);
    const std::vector<BodyHandle> boxes = BuildTower(world);
    ASSERT_GE(SettleUntilSleeping(world), 0);
    ASSERT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount);
    const int32_t contactsBeforeSwitch = world.GetContactCount();
    ASSERT_GT(contactsBeforeSwitch, 0);

    // Switch while the island is asleep: the old implementation's proxies are
    // released immediately (before any step) and the next step re-inserts one
    // proxy per shaped body, including the sleeping ones.
    world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    ASSERT_NE(world.GetBroadPhase(), nullptr);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);
    EXPECT_EQ(world.GetBroadPhasePairCount(), 0);
    StepFor(world, 1);

    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), kTowerBoxCount + 1);
    EXPECT_EQ(world.GetSleepingBodyCount(), kTowerBoxCount); // a switch must not wake sleepers
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    EXPECT_EQ(world.GetContactCount(), contactsBeforeSwitch); // frozen contacts reused
    for (const BodyHandle box : boxes) {
        EXPECT_FALSE(world.IsAwake(box));
        EXPECT_GE(world.GetBody(box)->proxyId, 0);
    }
    world.CheckSane();

    // The island is still a functioning sleeper: waking it works after the switch.
    world.WakeBody(boxes.front());
    EXPECT_EQ(world.GetSleepingBodyCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), kTowerBoxCount);

    std::printf("[boundary8] broad-phase switch while asleep: proxies restored = %d, "
                "contacts preserved = %d\n",
                static_cast<int>(world.GetBroadPhase()->GetProxyCount()),
                static_cast<int>(contactsBeforeSwitch));
}

TEST(WorldBroadPhaseBoundaryTest, PairSourcesStayBitIdenticalAcrossLifecycleChurn) {
    // Week 6 proved no-broad-phase / BruteForce / SAP are bit-identical on a fixed
    // scene. This extends it to LIFECYCLE CHURN: create bodies, destroy one while
    // it is in contact, create a replacement that reuses the slot, and keep
    // stepping. A proxy bookkeeping slip (stale proxy id, stale reverse mapping)
    // would show up here as a different contact set/physics.
    const auto buildScene = [](World& world) {
        AddGroundPlane(world);
        std::vector<BodyHandle> boxes = BuildTower(world, 6);
        // Two circles dropping next to the tower make the contact/trajectory
        // pattern asymmetric, so an ordering slip cannot hide behind symmetry.
        AddBox(world, Vec2(2.0f, 4.0f));
        const BodyHandle ballA = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(-2.0f, 5.0f)));
        EXPECT_TRUE(world.AttachShape(ballA, world.CreateCircleShape(0.4f)));
        const BodyHandle ballB = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(2.5f, 6.0f)));
        EXPECT_TRUE(world.AttachShape(ballB, world.CreateCircleShape(0.3f)));
        return boxes;
    };

    World fallbackWorld;
    World bruteForceWorld;
    bruteForceWorld.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    World sapWorld;
    sapWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());

    std::vector<BodyHandle> fallbackBoxes = buildScene(fallbackWorld);
    std::vector<BodyHandle> bruteForceBoxes = buildScene(bruteForceWorld);
    std::vector<BodyHandle> sapBoxes = buildScene(sapWorld);
    ASSERT_EQ(fallbackBoxes.size(), bruteForceBoxes.size());
    ASSERT_EQ(fallbackBoxes.size(), sapBoxes.size());

    // Phase 1: settle with the tower interacting with the circles.
    for (int32_t step = 0; step < 40; ++step) {
        fallbackWorld.Step(kDt);
        bruteForceWorld.Step(kDt);
        sapWorld.Step(kDt);
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(bruteForceWorld))
            << "fallback vs BruteForce diverged at step " << step;
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(sapWorld))
            << "fallback vs SAP diverged at step " << step;
    }

    // Phase 2: destroy the top tower box (in contact with the one below it) in
    // each world, then keep stepping.
    fallbackWorld.DestroyBody(fallbackBoxes.back());
    bruteForceWorld.DestroyBody(bruteForceBoxes.back());
    sapWorld.DestroyBody(sapBoxes.back());
    for (int32_t step = 0; step < 30; ++step) {
        fallbackWorld.Step(kDt);
        bruteForceWorld.Step(kDt);
        sapWorld.Step(kDt);
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(bruteForceWorld))
            << "fallback vs BruteForce diverged after destroy, step " << step;
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(sapWorld))
            << "fallback vs SAP diverged after destroy, step " << step;
    }

    // Phase 3: create a falling box that reuses the freed slot, then keep stepping.
    fallbackBoxes.pop_back();
    bruteForceBoxes.pop_back();
    sapBoxes.pop_back();
    fallbackBoxes.push_back(AddBox(fallbackWorld, Vec2(0.2f, 8.0f)));
    bruteForceBoxes.push_back(AddBox(bruteForceWorld, Vec2(0.2f, 8.0f)));
    sapBoxes.push_back(AddBox(sapWorld, Vec2(0.2f, 8.0f)));
    for (int32_t step = 0; step < 130; ++step) {
        fallbackWorld.Step(kDt);
        bruteForceWorld.Step(kDt);
        sapWorld.Step(kDt);
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(bruteForceWorld))
            << "fallback vs BruteForce diverged after reuse, step " << step;
        ASSERT_EQ(SnapshotWorldPhysics(fallbackWorld), SnapshotWorldPhysics(sapWorld))
            << "fallback vs SAP diverged after reuse, step " << step;
    }

    // Proxy counts are now consistent with the body/shape counts of each source.
    EXPECT_EQ(bruteForceWorld.GetBroadPhase()->GetProxyCount(), bruteForceWorld.GetBodyCount());
    EXPECT_EQ(sapWorld.GetBroadPhase()->GetProxyCount(), sapWorld.GetBodyCount());
    EXPECT_EQ(fallbackWorld.GetBroadPhasePairCount(), 0); // no broad phase by design

    std::printf("[boundary8] lifecycle churn: 200 steps x {none, BruteForce, SAP} bit-identical; "
                "bodies = %d\n",
                static_cast<int>(fallbackWorld.GetBodyCount()));
}

// ---------------------------------------------------------------------------
// 5. Core-state invariants (the states World::CheckSane now scans, Week 8)
// ---------------------------------------------------------------------------

TEST(WorldStateInvariantTest, IslandSleepBookkeepingIsConsistentAfterEveryStep) {
    World world;
    world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    AddGroundPlane(world);
    const std::vector<BodyHandle> leftTower = BuildTower(world, 3);
    std::vector<BodyHandle> rightTower;
    for (int32_t index = 0; index < 3; ++index) {
        rightTower.push_back(AddBox(world, Vec2(4.0f, 0.5f + static_cast<float>(index))));
    }
    ASSERT_EQ(CountDynamicBodies(world), 6);

    const auto validateInvariants = [&world]() {
        const int32_t islandCount = world.GetIslandCount();
        for (const Body& body : world.GetBodies()) {
            if (!body.handle.IsValid()) {
                continue;
            }
            EXPECT_GE(body.islandId, -1);
            EXPECT_LT(body.islandId, islandCount);
            EXPECT_TRUE(std::isfinite(body.sleepTime));
            EXPECT_GE(body.sleepTime, 0.0f);
            if (body.type == BodyType::Dynamic && !body.isAwake) {
                // A sleeping dynamic body is exactly frozen (UpdateSleep clears all
                // four velocity channels; see the W7 whole-island sleep contract).
                EXPECT_TRUE(HasZeroVelocity(body));
            }
            // The position layer integrates and then clears the pseudo velocities
            // inside a full Step, so no pseudo state may leak between steps.
            EXPECT_EQ(body.pseudoLinearVelocity, Vec2(0.0f, 0.0f));
            EXPECT_FLOAT_EQ(body.pseudoAngularVelocity, 0.0f);
        }
    };

    // Run until both towers sleep, validating after every step (CheckSane scans
    // the same states and must not fire on a healthy world).
    int32_t sleepStep = -1;
    for (int32_t step = 0; step < kSettleMaxSteps; ++step) {
        world.Step(kDt);
        world.CheckSane();
        validateInvariants();
        if (world.GetSleepingBodyCount() == 6 && world.GetAwakeBodyCount() == 0) {
            sleepStep = step + 1;
            break;
        }
    }
    ASSERT_GE(sleepStep, 0) << "two 3-box towers did not sleep within " << kSettleMaxSteps << " steps";
    EXPECT_EQ(world.GetIslandCount(), 2); // two independent towers
    EXPECT_EQ(world.GetSleepingBodyCount(), 6);

    // Waking one tower must leave the other asleep (islands are independent) and
    // reset the sleep timer of exactly the woken island.
    world.WakeBody(rightTower[1]);
    EXPECT_EQ(world.GetAwakeBodyCount(), 3);
    EXPECT_EQ(world.GetSleepingBodyCount(), 3);
    EXPECT_EQ(world.GetBody(leftTower[0])->islandId, world.GetBody(leftTower[2])->islandId);
    EXPECT_EQ(world.GetBody(rightTower[0])->islandId, world.GetBody(rightTower[2])->islandId);
    EXPECT_NE(world.GetBody(leftTower[0])->islandId, world.GetBody(rightTower[0])->islandId);
    for (const BodyHandle box : rightTower) {
        EXPECT_TRUE(world.IsAwake(box));
        EXPECT_FLOAT_EQ(world.GetBody(box)->sleepTime, 0.0f);
    }
    for (const BodyHandle box : leftTower) {
        EXPECT_FALSE(world.IsAwake(box));
        EXPECT_TRUE(HasZeroVelocity(*world.GetBody(box)));
    }
    world.Step(kDt);
    world.CheckSane();
    validateInvariants();

    std::printf("[boundary8] two-island invariants: slept at step %d, islands = %d, "
                "independent island wake keeps the other tower asleep\n",
                static_cast<int>(sleepStep), static_cast<int>(world.GetIslandCount()));
}

// ---------------------------------------------------------------------------
// 6. Release-only profiling measurement (quoted by docs/profiling.md)
// ---------------------------------------------------------------------------

#ifdef NDEBUG
namespace {

// Same construction as testbed/demos/IslandSleep500.cpp: a static ground plane
// plus 100 columns of 5 exactly stacked 1 m boxes (pitch 1.25 m), so every column
// is its own island. 501 bodies total.
void BuildIslandSleep500Scene(World& world) {
    AddGroundPlane(world);
    constexpr int32_t kColumns = 100;
    constexpr int32_t kStackHeight = 5;
    constexpr float kHalfExtent = 0.5f;
    constexpr float kPitch = 1.25f;
    for (int32_t column = 0; column < kColumns; ++column) {
        const float centerX = (static_cast<float>(column) - static_cast<float>(kColumns - 1) * 0.5f) * kPitch;
        for (int32_t stacked = 0; stacked < kStackHeight; ++stacked) {
            AddBox(world, Vec2(centerX, kHalfExtent + 2.0f * kHalfExtent * static_cast<float>(stacked)),
                   kHalfExtent, kHalfExtent);
        }
    }
}

double TimeOneStepMilliseconds(World& world) {
    const auto start = std::chrono::steady_clock::now();
    world.Step(kDt);
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

TEST(WorldBoundaryProfilingTest, SleepingOnVsOffStepTimingIsPrinted) {
    // Mirrors the IslandSleep500 testbed smoke windows (warm-up 10 frames, early
    // window frames 30-89) so the two measurements are comparable. The numbers are
    // only quotable from a Release build; no timing value is asserted (machine
    // dependent) - only that the scenario behaves as documented.
    constexpr int32_t kFrames = 600;
    constexpr int32_t kWarmupFrames = 10;
    constexpr int32_t kEarlyWindowStart = 30;
    constexpr int32_t kEarlyWindowEnd = 90;
    constexpr int32_t kBoxCount = 500;

    World sleepingWorld;
    sleepingWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    BuildIslandSleep500Scene(sleepingWorld);
    ASSERT_EQ(CountDynamicBodies(sleepingWorld), kBoxCount);

    double earlySum = 0.0;
    int32_t earlyCount = 0;
    double awakeSum = 0.0;
    int32_t awakeCount = 0;
    double asleepSum = 0.0;
    int32_t asleepCount = 0;
    int32_t firstAllAsleepFrame = -1;
    for (int32_t frame = 1; frame <= kFrames; ++frame) {
        const double stepMilliseconds = TimeOneStepMilliseconds(sleepingWorld);
        const bool allAsleep = sleepingWorld.GetSleepingBodyCount() == kBoxCount;
        if (allAsleep && firstAllAsleepFrame < 0) {
            firstAllAsleepFrame = frame;
        }
        if (frame > kWarmupFrames) {
            if (frame >= kEarlyWindowStart && frame < kEarlyWindowEnd) {
                earlySum += stepMilliseconds;
                ++earlyCount;
            }
            if (allAsleep) {
                asleepSum += stepMilliseconds;
                ++asleepCount;
            } else {
                awakeSum += stepMilliseconds;
                ++awakeCount;
            }
        }
    }
    ASSERT_GT(firstAllAsleepFrame, 0);
    EXPECT_EQ(sleepingWorld.GetSleepingBodyCount(), kBoxCount);
    ASSERT_GT(awakeCount, 0);
    ASSERT_GT(asleepCount, 0);
    ASSERT_GT(earlyCount, 0);

    // Sleeping-off control: identical scene, same simulated time, `enableSleeping`
    // false. It must never sleep, so its window is the awake baseline.
    World awakeWorld;
    awakeWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    p2::SolverSettings awakeSettings = awakeWorld.GetSolverSettings();
    awakeSettings.enableSleeping = false;
    awakeWorld.SetSolverSettings(awakeSettings);
    BuildIslandSleep500Scene(awakeWorld);
    ASSERT_EQ(CountDynamicBodies(awakeWorld), kBoxCount);
    double awakeOffSum = 0.0;
    int32_t awakeOffCount = 0;
    for (int32_t frame = 1; frame <= kFrames; ++frame) {
        const double stepMilliseconds = TimeOneStepMilliseconds(awakeWorld);
        if (frame > kWarmupFrames) {
            awakeOffSum += stepMilliseconds;
            ++awakeOffCount;
        }
    }
    ASSERT_GT(awakeOffCount, 0);
    EXPECT_EQ(awakeWorld.GetSleepingBodyCount(), 0);

    const double earlyMilliseconds = earlySum / static_cast<double>(earlyCount);
    const double awakeMilliseconds = awakeSum / static_cast<double>(awakeCount);
    const double asleepMilliseconds = asleepSum / static_cast<double>(asleepCount);
    const double awakeOffMilliseconds = awakeOffSum / static_cast<double>(awakeOffCount);
    std::printf("[boundary8] islandsleep500 Release: first_all_asleep_frame=%d "
                "early_avg_step_ms=%.4f awake_avg_step_ms=%.4f asleep_avg_step_ms=%.4f "
                "speedup=%.2fx sleeping_off_avg_step_ms=%.4f\n",
                static_cast<int>(firstAllAsleepFrame), earlyMilliseconds, awakeMilliseconds,
                asleepMilliseconds, awakeMilliseconds / std::max(asleepMilliseconds, 1.0e-9),
                awakeOffMilliseconds);
}
#endif // NDEBUG

} // namespace
