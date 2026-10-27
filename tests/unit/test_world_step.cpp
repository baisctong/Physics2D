// Week 1 D6: p2::World lifecycle, stable accessors and the two live integration
// phases (1 IntegrateForces, 7 IntegratePositions, P05 / Section 4.6).
//
// Every numeric expectation is hand-computed in a comment next to the assertion.
// The fixed step is dt = kFixedTimeStep = 1/60 s. EXPECT_FLOAT_EQ allows a few
// ULPs of accumulation noise; EXPECT_NEAR is used where Rot::GetAngle goes
// through atan2 or where a hand-written analytic value has several operations.

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <limits>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;

BodyDef MakeBody(BodyType type, Vec2 position, float mass = 1.0f, float inertia = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.mass = mass;
    def.inertia = inertia;
    return def;
}

TEST(WorldTest, CreateBodyExposesState) {
    World world(Vec2(0.0f, 0.0f));
    BodyDef def = MakeBody(BodyType::Dynamic, Vec2(1.0f, 2.0f), 2.0f, 4.0f);
    def.angle = 0.5f;
    def.linearVelocity = Vec2(3.0f, 4.0f);
    def.angularVelocity = 2.0f;
    def.allowSleep = false;
    def.isSensor = true;
    def.friction = 0.4f;
    def.restitution = 0.6f;
    def.debugName = "dynamic";
    int payload = 7;
    def.userData = &payload;

    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));
    EXPECT_TRUE(handle.IsValid());
    EXPECT_EQ(handle.index, 0);
    EXPECT_EQ(handle.generation, 1U); // fresh slot from IDPool

    EXPECT_EQ(world.GetBodyCount(), 1);
    EXPECT_EQ(world.GetAwakeBodyCount(), 1);
    ASSERT_NE(world.GetBody(handle), nullptr);
    EXPECT_EQ(world.GetBody(handle)->debugName, "dynamic");

    EXPECT_EQ(world.GetPosition(handle), Vec2(1.0f, 2.0f));
    EXPECT_FLOAT_EQ(world.GetTransform(handle).p.x, 1.0f);
    EXPECT_FLOAT_EQ(world.GetTransform(handle).p.y, 2.0f);
    EXPECT_NEAR(world.GetAngle(handle), 0.5f, 1.0e-6f); // atan2 round trip
    EXPECT_EQ(world.GetLinearVelocity(handle), Vec2(3.0f, 4.0f));
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 2.0f);
    EXPECT_FLOAT_EQ(world.GetMass(handle), 2.0f);
    EXPECT_FLOAT_EQ(world.GetInvMass(handle), 0.5f); // 1 / 2
    EXPECT_EQ(world.GetBodyType(handle), BodyType::Dynamic);
    EXPECT_TRUE(world.IsAwake(handle));
    EXPECT_EQ(world.GetUserData(handle), static_cast<void*>(&payload));
    // Reserved statistics report zero until the relevant weeks land.
    EXPECT_EQ(world.GetIslandCount(), 0);
    EXPECT_EQ(world.GetContactCount(), 0);
}

TEST(WorldTest, CreateBodyRejectsInvalidDefinitions) {
    World world(Vec2(0.0f, 0.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const BodyDef base = MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0f, 1.0f);

    const auto expectRejected = [&world](const BodyDef& def) {
        EXPECT_FALSE(world.IsValid(world.CreateBody(def)));
    };

    BodyDef bad = base;
    bad.position = Vec2(nan, 0.0f);
    expectRejected(bad); // non-finite transform
    bad = base;
    bad.position = Vec2(0.0f, inf);
    expectRejected(bad);
    bad = base;
    bad.angle = nan;
    expectRejected(bad);
    bad = base;
    bad.linearVelocity = Vec2(inf, 0.0f);
    expectRejected(bad);
    bad = base;
    bad.angularVelocity = nan;
    expectRejected(bad);
    bad = base;
    bad.mass = 0.0f;
    expectRejected(bad); // dynamic mass must be > 0
    bad = base;
    bad.mass = -1.0f;
    expectRejected(bad);
    bad = base;
    bad.inertia = 0.0f;
    expectRejected(bad); // dynamic inertia must be > 0
    bad = base;
    bad.density = nan;
    expectRejected(bad);
    bad = base;
    bad.friction = -0.1f;
    expectRejected(bad);
    bad = base;
    bad.restitution = nan;
    expectRejected(bad);

    // Rejections must not consume a slot: the world is still empty ...
    EXPECT_EQ(world.GetBodyCount(), 0);
    // ... and the pool is healthy, so a valid definition is accepted afterwards.
    EXPECT_TRUE(world.IsValid(world.CreateBody(base)));

    // Static/kinematic bodies ignore their mass properties, so zero/negative
    // values are not a rejection criterion for them (the type makes them inert).
    BodyDef staticDef = MakeBody(BodyType::Static, Vec2(0.0f, 0.0f));
    staticDef.mass = 0.0f;
    staticDef.inertia = -3.0f;
    EXPECT_TRUE(world.IsValid(world.CreateBody(staticDef)));
}

TEST(WorldTest, StaticBodyIgnoresGravity) {
    World world; // default gravity (0, -10)
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Static, Vec2(0.0f, 5.0f)));
    ASSERT_TRUE(world.IsValid(handle));
    // Static bodies have infinite mass and are born asleep.
    EXPECT_FALSE(world.IsAwake(handle));
    EXPECT_FLOAT_EQ(world.GetMass(handle), 0.0f);
    EXPECT_FLOAT_EQ(world.GetInvMass(handle), 0.0f);

    for (int32_t step = 0; step < 60; ++step) {
        world.Step(kDt);
    }
    // One simulated second at g = -10 m/s^2: velocity stays 0 and the body does
    // not move a millimetre.
    EXPECT_EQ(world.GetPosition(handle), Vec2(0.0f, 5.0f));
    EXPECT_EQ(world.GetLinearVelocity(handle), Vec2(0.0f, 0.0f));
}

TEST(WorldTest, KinematicBodyMovesByVelocityOnly) {
    World world; // default gravity (0, -10)
    BodyDef def = MakeBody(BodyType::Kinematic, Vec2(0.0f, 0.0f));
    def.linearVelocity = Vec2(1.0f, 0.0f);
    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));
    EXPECT_TRUE(world.IsAwake(handle));
    EXPECT_FLOAT_EQ(world.GetInvMass(handle), 0.0f);

    world.Step(kDt);
    // x = v * dt = 1 * (1/60) = 0.0166667 m; gravity is ignored (infinite mass).
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, 0.0f);
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, 0.0f);

    world.Step(kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, 2.0f * kDt);
}

TEST(WorldTest, DynamicBodyFallsUnderGravity) {
    World world; // default gravity (0, -10)
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 10.0f), 2.0f, 2.0f));
    ASSERT_TRUE(world.IsValid(handle));

    world.Step(kDt);
    // Phase 1: v = 0 + (-10 + 0/2) * dt = -10 * dt = -0.1666667 m/s.
    // Phase 7 (semi-implicit): y = 10 + v * dt = 10 - 10 * dt^2 = 9.9972222 m.
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, -10.0f * kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, 10.0f - 10.0f * kDt * kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, 0.0f);
}

TEST(WorldTest, FreeFallMatchesSemiImplicitAnalyticSolution) {
    World world; // default gravity (0, -10)
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 10.0f)));
    ASSERT_TRUE(world.IsValid(handle));

    // Semi-implicit Euler after n steps:
    //   v_n = -g * n * dt
    //   y_n = 10 - g * dt^2 * (1 + 2 + ... + n) = 10 - g * dt^2 * n(n+1)/2
    constexpr int32_t kStepCount = 6;
    for (int32_t step = 0; step < kStepCount; ++step) {
        world.Step(kDt);
    }
    // v_6 = -10 * 6/60 = -1 m/s.
    EXPECT_NEAR(world.GetLinearVelocity(handle).y, -1.0f, 1.0e-6f);
    // y_6 = 10 - 10 * (1/3600) * (6*7/2) = 10 - 210/3600 = 9.9416667 m.
    EXPECT_NEAR(world.GetPosition(handle).y, 9.9416667f, 1.0e-5f);
}

TEST(WorldTest, StepIntegratesForcesBeforePositions) {
    World world(Vec2(0.0f, 0.0f)); // gravity off: isolate the applied force
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f, 2.0f));
    ASSERT_TRUE(world.IsValid(handle));

    // F = (2, 0) applied at the centre of mass -> a = F/m = 1 m/s^2, no torque.
    world.ApplyForce(handle, Vec2(2.0f, 0.0f), Vec2(0.0f, 0.0f));
    world.Step(kDt);

    // Phase 1 first: v = a * dt = dt = 0.0166667 m/s.
    // Phase 7 uses that NEW velocity: x = v * dt = dt^2 = 0.00027778 m.
    // Explicit Euler (position integrated before velocity) would give x = 0, so
    // this assertion pins the documented phase order.
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).x, kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, kDt * kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, 0.0f);
}

TEST(WorldTest, AppliedForceIsConsumedOnce) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f, 2.0f));
    ASSERT_TRUE(world.IsValid(handle));

    world.ApplyForce(handle, Vec2(2.0f, 0.0f), Vec2(0.0f, 0.0f)); // a = 1 m/s^2
    world.Step(kDt);
    // Force accumulators are cleared at the end of phase 1, so the second step
    // sees no force: velocity stays at a * dt = dt, and the body keeps moving at
    // constant speed (x = dt^2 + dt^2 = 2 * dt^2).
    world.Step(kDt);
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).x, kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, 2.0f * kDt * kDt);
}

TEST(WorldTest, ForceAtOffsetPointCreatesTorque) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f, 2.0f));
    ASSERT_TRUE(world.IsValid(handle));

    // F = (0, 2) at world point (1, 0): r = (1, 0), so
    // torque = Cross(r, F) = 1*2 - 0*0 = 2 N*m (counter-clockwise, Section 4.2).
    world.ApplyForce(handle, Vec2(0.0f, 2.0f), Vec2(1.0f, 0.0f));
    world.Step(kDt);

    // w = invI * torque * dt = 0.5 * 2 * dt = dt = 0.0166667 rad/s.
    // angle = w * dt = dt^2 = 0.00027778 rad (far below kMaxRotation).
    // Linear: a = F/m = (0, 1) -> v.y = dt and y = dt^2.
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), kDt);
    EXPECT_NEAR(world.GetAngle(handle), kDt * kDt, 1.0e-6f);
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, kDt * kDt);
}

TEST(WorldTest, ApplyLinearImpulseUsesSection42CrossConvention) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f, 2.0f));
    ASSERT_TRUE(world.IsValid(handle));

    // J = (4, 0) through the centre of mass: dv = J/m = (2, 0), r = 0 -> no spin.
    world.ApplyLinearImpulse(handle, Vec2(4.0f, 0.0f), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).x, 2.0f);
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, 0.0f);
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 0.0f);

    // J = (0, 4) at (1, 0): dv = (0, 2) and
    // dw = invI * Cross(r, J) = 0.5 * (1*4 - 0*0) = 2 rad/s.
    world.ApplyLinearImpulse(handle, Vec2(0.0f, 4.0f), Vec2(1.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, 2.0f);
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 2.0f);

    world.Step(kDt);
    // Position integrates the new velocity: x = 2 * dt, y = 2 * dt.
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, 2.0f * kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, 2.0f * kDt);
}

TEST(WorldTest, LinearDisplacementIsClampedPerStep) {
    World world(Vec2(0.0f, 0.0f)); // gravity off to keep the motion along +x
    BodyDef def = MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f));
    def.linearVelocity = Vec2(1.0e6f, 0.0f);
    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));

    world.Step(kDt);
    // Unclamped the body would travel v*dt = 1e6/60 = 16666.7 m in one step.
    // The tunnelling guard (Section 6.6) limits the displacement to
    // kMaxTranslation = 2 m while keeping the direction of travel.
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, p2::kMaxTranslation);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, 0.0f);
    EXPECT_LE(p2::Length(world.GetPosition(handle)), p2::kMaxTranslation);

    // The clamp is per step, not a velocity clamp: the velocity is untouched, so
    // the next step travels another kMaxTranslation.
    world.Step(kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, 2.0f * p2::kMaxTranslation);
}

TEST(WorldTest, AngularDisplacementIsClampedPerStep) {
    World world(Vec2(0.0f, 0.0f));
    BodyDef def = MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f));
    def.angularVelocity = 1.0e6f;
    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));

    world.Step(kDt);
    // Unclamped: dAngle = 1e6/60 = 16666.7 rad. The clamp allows at most
    // kMaxRotation = pi/2 = 1.5707963 rad per step.
    EXPECT_NEAR(world.GetAngle(handle), p2::kMaxRotation, 1.0e-5f);
}

TEST(WorldTest, FixedRotationIgnoresTorque) {
    World world(Vec2(0.0f, 0.0f));
    BodyDef def = MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f, 2.0f);
    def.fixedRotation = true;
    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));

    world.ApplyForce(handle, Vec2(0.0f, 2.0f), Vec2(1.0f, 0.0f)); // torque = 2
    world.Step(kDt);
    // fixedRotation zeroes invInertia even for a dynamic body: no spin, while
    // the linear response is unaffected (a = (0, 1) -> v.y = dt, y = dt^2).
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 0.0f);
    EXPECT_FLOAT_EQ(world.GetAngle(handle), 0.0f);
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).y, kDt);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).y, kDt * kDt);
}

TEST(WorldTest, KinematicBodyIgnoresForces) {
    World world(Vec2(0.0f, 0.0f));
    BodyDef def = MakeBody(BodyType::Kinematic, Vec2(0.0f, 0.0f));
    def.linearVelocity = Vec2(1.0f, 0.0f);
    const BodyHandle handle = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(handle));

    world.ApplyForce(handle, Vec2(1000.0f, 0.0f), Vec2(0.0f, 0.0f));
    world.Step(kDt);
    // invMass = 0: the force is discarded and only the def velocity moves the
    // body, so x = 1 * dt.
    EXPECT_FLOAT_EQ(world.GetLinearVelocity(handle).x, 1.0f);
    EXPECT_FLOAT_EQ(world.GetPosition(handle).x, kDt);
}

TEST(WorldTest, StaticBodyRejectsForcesAndImpulses) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Static, Vec2(1.0f, 2.0f)));
    ASSERT_TRUE(world.IsValid(handle));

    world.ApplyForce(handle, Vec2(50.0f, 50.0f), Vec2(3.0f, 3.0f));
    world.ApplyLinearImpulse(handle, Vec2(50.0f, 50.0f), Vec2(3.0f, 3.0f));
    world.Step(kDt);
    EXPECT_EQ(world.GetPosition(handle), Vec2(1.0f, 2.0f));
    EXPECT_EQ(world.GetLinearVelocity(handle), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 0.0f);
}

TEST(WorldTest, DestroyBodyInvalidatesHandleAndReusesSlot) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle first = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(3.0f, 4.0f)));
    ASSERT_TRUE(world.IsValid(first));
    const int32_t firstIndex = first.index;

    world.DestroyBody(first);
    EXPECT_FALSE(world.IsValid(first));
    EXPECT_EQ(world.GetBody(first), nullptr);
    EXPECT_EQ(world.GetBodyCount(), 0);
    // Stale read accessors return neutral defaults instead of touching dead data.
    EXPECT_EQ(world.GetPosition(first), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetMass(first), 0.0f);
    EXPECT_FALSE(world.IsAwake(first));
    EXPECT_EQ(world.GetBodyType(first), BodyType::Static);

    // Destroying a stale handle is a documented no-op (idempotent cleanup).
    world.DestroyBody(first);
    EXPECT_EQ(world.GetBodyCount(), 0);

    // Reuse: the next body takes the same slot with a new generation, so the old
    // handle still fails IsValid (the ABA case).
    const BodyHandle second = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(7.0f, 8.0f)));
    ASSERT_TRUE(world.IsValid(second));
    EXPECT_EQ(second.index, firstIndex);
    EXPECT_NE(second.generation, first.generation);
    EXPECT_FALSE(world.IsValid(first));
    EXPECT_EQ(world.GetPosition(second), Vec2(7.0f, 8.0f));
    EXPECT_EQ(world.GetBodies().size(), 1U); // slot reused, not appended
}

TEST(WorldTest, BodyAndAwakeCounts) {
    World world;
    const BodyHandle staticBody = world.CreateBody(MakeBody(BodyType::Static, Vec2(0.0f, 0.0f)));
    const BodyHandle kinematicBody = world.CreateBody(MakeBody(BodyType::Kinematic, Vec2(0.0f, 0.0f)));
    const BodyHandle dynamicBody = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f)));

    EXPECT_TRUE(world.IsValid(staticBody));
    EXPECT_TRUE(world.IsValid(kinematicBody));
    EXPECT_TRUE(world.IsValid(dynamicBody));
    EXPECT_EQ(world.GetBodyCount(), 3);
    // Static is born asleep; kinematic and dynamic are awake.
    EXPECT_EQ(world.GetAwakeBodyCount(), 2);

    world.DestroyBody(kinematicBody);
    EXPECT_EQ(world.GetBodyCount(), 2);
    EXPECT_EQ(world.GetAwakeBodyCount(), 1);
}

TEST(WorldTest, ClearRemovesBodiesAndResetsSlotSpace) {
    World world;
    const BodyHandle first = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    const BodyHandle second = world.CreateBody(MakeBody(BodyType::Static, Vec2(1.0f, 0.0f)));
    ASSERT_TRUE(world.IsValid(first));
    ASSERT_TRUE(world.IsValid(second));

    world.Clear();
    EXPECT_EQ(world.GetBodyCount(), 0);
    EXPECT_EQ(world.GetAwakeBodyCount(), 0);
    EXPECT_FALSE(world.IsValid(first));
    EXPECT_FALSE(world.IsValid(second));
    EXPECT_TRUE(world.GetBodies().empty());

    // The world is immediately usable again and the slot space restarts at 0/1.
    const BodyHandle third = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(1.0f, 1.0f)));
    EXPECT_TRUE(world.IsValid(third));
    EXPECT_EQ(third.index, 0);
    EXPECT_EQ(third.generation, 1U);
}

TEST(WorldTest, StepIterationOverrideMatchesDefaultSettings) {
    // Week 1 has no solver phases, so the per-call iteration override must not
    // change the result (the counts are reserved for Weeks 3-5).
    const BodyDef def = MakeBody(BodyType::Dynamic, Vec2(0.0f, 5.0f));
    World defaultWorld;
    World overrideWorld;
    const BodyHandle defaultHandle = defaultWorld.CreateBody(def);
    const BodyHandle overrideHandle = overrideWorld.CreateBody(def);
    ASSERT_TRUE(defaultWorld.IsValid(defaultHandle));
    ASSERT_TRUE(overrideWorld.IsValid(overrideHandle));

    defaultWorld.Step(kDt);
    overrideWorld.Step(kDt, 2, 1);
    EXPECT_EQ(defaultWorld.GetPosition(defaultHandle), overrideWorld.GetPosition(overrideHandle));
    EXPECT_EQ(defaultWorld.GetLinearVelocity(defaultHandle), overrideWorld.GetLinearVelocity(overrideHandle));
}

TEST(WorldTest, TransformAndVelocitySettersRoundTrip) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    ASSERT_TRUE(world.IsValid(handle));

    world.SetTransform(handle, Vec2(1.0f, 2.0f), 0.25f);
    EXPECT_EQ(world.GetPosition(handle), Vec2(1.0f, 2.0f));
    EXPECT_NEAR(world.GetAngle(handle), 0.25f, 1.0e-6f);

    world.SetLinearVelocity(handle, Vec2(3.0f, 4.0f));
    EXPECT_EQ(world.GetLinearVelocity(handle), Vec2(3.0f, 4.0f));
    world.SetAngularVelocity(handle, 5.0f);
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(handle), 5.0f);

    int payload = 0;
    world.SetUserData(handle, &payload);
    EXPECT_EQ(world.GetUserData(handle), static_cast<void*>(&payload));
}

TEST(WorldTest, GravityAndSolverSettingsAccessors) {
    World world;
    EXPECT_EQ(world.GetGravity(), Vec2(0.0f, p2::kDefaultGravityY));
    world.SetGravity(Vec2(0.0f, -20.0f));
    EXPECT_EQ(world.GetGravity(), Vec2(0.0f, -20.0f));

    p2::SolverSettings settings;
    settings.velocityIterations = 3;
    settings.positionIterations = 2;
    world.SetSolverSettings(settings);
    EXPECT_EQ(world.GetSolverSettings().velocityIterations, 3);
    EXPECT_EQ(world.GetSolverSettings().positionIterations, 2);
}

TEST(WorldTest, CheckSaneAcceptsHealthyState) {
    // The debug validator must be a no-op for healthy bodies. The failure path
    // (NaN/Inf -> assert) is covered by WorldDeathTest below.
    World world;
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 5.0f)));
    ASSERT_TRUE(world.IsValid(handle));
    world.Step(kDt);
    world.CheckSane();
    SUCCEED();
}

// Debug-only death tests: the fixture skips them when the host forbids the
// temp-file write that GoogleTest's death-test machinery needs (TestSupport.h),
// so sandboxed builds stay green while CI still runs the assertions. The fixture
// name is unique across the unit-test tree because gtest requires one fixture
// class per test suite (test_debug_asserts.cpp has its own death fixtures).
class WorldAssertDeathTest : public p2test::DeathTestFixture {};

TEST_F(WorldAssertDeathTest, StepDetectsNonFiniteBodyState) {
    // Inject an overflow through the public API only: a 1e-30 kg body hit by a
    // 1e38 N force. a = F/m overflows to inf, the position update turns inf*0
    // into NaN, and CheckSane (debug builds) must catch it and abort.
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle handle = world.CreateBody(MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0e-30f, 1.0f));
    ASSERT_TRUE(world.IsValid(handle));
    world.ApplyForce(handle, Vec2(1.0e38f, 0.0f), Vec2(0.0f, 0.0f));
    EXPECT_DEATH(world.Step(kDt), "CheckSane");
}

TEST_F(WorldAssertDeathTest, BodyInitializeRejectsNonFiniteInput) {
    BodyDef def = MakeBody(BodyType::Dynamic, Vec2(0.0f, 0.0f));
    def.position = Vec2(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    p2::Body body;
    EXPECT_DEATH(body.Initialize(def), "Body::Initialize");
}

} // namespace
