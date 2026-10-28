// Week 2 D6: temporary contact response behaviour tests (falling ball on a plane).
//
// Week 2 has no impulse solver: SolvePositionConstraints() is a deliberately
// TEMPORARY position-level hack (push out along the normal + direct normal
// velocity projection). These tests pin its observable behaviour, not a
// physically exact solution:
//   - e = 0: the ball settles, |vy| ~ 0 and the steady penetration is the
//     analytic fixed point of the position correction (see below);
//   - e > 0: the ball bounces, and the rebound peak is below the drop height
//     (energy is removed, never added);
//   - the static ground never moves;
//   - two identical worlds produce bit-identical trajectories (determinism).
//
// Steady-state penetration derivation (one fixed step):
//   phase 1: v = -g * dt
//   phase 7: y -= g * dt^2
//   phase 8: y += (pen - kLinearSlop), pen measured at the start of the step
// so at the fixed point pen* = kLinearSlop + g * dt^2. With g = 10, dt = 1/60:
//   pen* = 0.005 + 10/3600 = 0.0077778 m, i.e. y* = 0.5 - pen* = 0.4922222 m.

#include <gtest/gtest.h>

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr float kBallRadius = 0.5f;
constexpr float kGravityMagnitude = 10.0f; // matches kDefaultGravityY

struct FallingScene {
    BodyHandle ground;
    BodyHandle ball;
};

// Static ground plane at y = 0 plus one dynamic ball dropped from `height`.
FallingScene MakeFallingScene(World& world, float restitution, float height = 3.0f) {
    FallingScene scene;

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    scene.ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(scene.ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));

    BodyDef ballDef;
    ballDef.type = BodyType::Dynamic;
    ballDef.position = Vec2(0.0f, height);
    ballDef.density = 1.0f;
    ballDef.restitution = restitution;
    scene.ball = world.CreateBody(ballDef);
    EXPECT_TRUE(world.AttachShape(scene.ball, world.CreateCircleShape(kBallRadius)));
    return scene;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

float ContactPenetration(const World& world) {
    if (world.GetContactCount() == 0) {
        return -1.0f;
    }
    return world.GetContacts()[0].manifold.penetration;
}

TEST(WorldResponseTest, ElasticityZeroBallSettlesAtRest) {
    World world; // default gravity (0, -10)
    const FallingScene scene = MakeFallingScene(world, 0.0f);
    ASSERT_TRUE(world.IsValid(scene.ball));

    StepFor(world, 180); // 3 simulated seconds

    // Resting contact: the ball must not fall through and must stop.
    EXPECT_LT(std::fabs(world.GetLinearVelocity(scene.ball).y), 0.05f);
    ASSERT_EQ(world.GetContactCount(), 1);
    EXPECT_LT(ContactPenetration(world), 0.01f);
    const float restingHeight = world.GetPosition(scene.ball).y;
    EXPECT_GT(restingHeight, 0.48f);
    EXPECT_LT(restingHeight, kBallRadius);

    // Last second of simulated time: a settled body must not drift.
    const float heightAtTwoSeconds = world.GetPosition(scene.ball).y;
    StepFor(world, 60);
    const float heightAtThreeSeconds = world.GetPosition(scene.ball).y;
    EXPECT_LT(std::fabs(heightAtThreeSeconds - heightAtTwoSeconds), 1.0e-3f);

    // The static ground never moves (infinite mass).
    EXPECT_EQ(world.GetPosition(scene.ground), Vec2(0.0f, 0.0f));
    EXPECT_EQ(world.GetLinearVelocity(scene.ground), Vec2(0.0f, 0.0f));
}

TEST(WorldResponseTest, SteadyPenetrationMatchesTheAnalyticFixedPoint) {
    World world;
    const FallingScene scene = MakeFallingScene(world, 0.0f);
    StepFor(world, 240); // 4 s: well past the transient of the initial drop
    ASSERT_EQ(world.GetContactCount(), 1);

    const float expectedPenetration = p2::kLinearSlop + kGravityMagnitude * kDt * kDt;
    EXPECT_NEAR(ContactPenetration(world), expectedPenetration, 1.0e-4f);
    EXPECT_NEAR(world.GetPosition(scene.ball).y, kBallRadius - expectedPenetration, 1.0e-4f);
}

TEST(WorldResponseTest, ElasticityZeroNeverBounces) {
    World world;
    const FallingScene scene = MakeFallingScene(world, 0.0f);

    // Track the largest upward velocity after the first contact: with e = 0 the
    // normal-velocity projection targets 0, so the ball may never rebound.
    float maxUpwardVelocity = 0.0f;
    bool contactSeen = false;
    for (int32_t step = 0; step < 240; ++step) {
        world.Step(kDt);
        contactSeen = contactSeen || world.GetContactCount() > 0;
        if (contactSeen) {
            maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(scene.ball).y);
        }
    }
    EXPECT_TRUE(contactSeen);
    EXPECT_LT(maxUpwardVelocity, 0.01f);
}

TEST(WorldResponseTest, ElasticityBouncesWithLowerPeakThanTheDropHeight) {
    World world;
    const float dropHeight = 3.0f;
    const FallingScene scene = MakeFallingScene(world, 0.8f, dropHeight);
    ASSERT_TRUE(world.IsValid(scene.ball));

    // Ideal impact speed from free fall to the tangent height:
    //   v = sqrt(2 * g * (dropHeight - radius)) = sqrt(2 * 10 * 2.5) = 7.071 m/s.
    // The rebound target is e * |vn| = 0.8 * 7.071 = 5.66 m/s, so a clearly
    // positive upward velocity must appear after the first contact.
    float maxUpwardVelocity = 0.0f;
    float peakHeightAfterContact = 0.0f;
    bool contactSeen = false;
    for (int32_t step = 0; step < 300; ++step) {
        world.Step(kDt);
        if (world.GetContactCount() > 0) {
            contactSeen = true;
        }
        if (contactSeen) {
            maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(scene.ball).y);
            peakHeightAfterContact = std::max(peakHeightAfterContact, world.GetPosition(scene.ball).y);
        }
    }
    EXPECT_TRUE(contactSeen);
    EXPECT_GT(maxUpwardVelocity, 1.0f);
    // Bounce peak must be above the ground and strictly below the drop height:
    // the temporary response is lossy (energy is removed, never injected).
    EXPECT_GT(peakHeightAfterContact, kBallRadius + 0.1f);
    EXPECT_LT(peakHeightAfterContact, dropHeight);
    // The ground is still static after all impacts.
    EXPECT_EQ(world.GetPosition(scene.ground), Vec2(0.0f, 0.0f));
}

TEST(WorldResponseTest, RestitutionThresholdStopsTheSmallBounces) {
    World world;
    const FallingScene scene = MakeFallingScene(world, 0.8f);

    // Each bounce keeps e = 0.8 of the impact speed: 5.66, 4.53, ..., 0.95 m/s.
    // Below kRestitutionVelocityThreshold (1 m/s) the bounce is suppressed, so
    // 15 simulated seconds are plenty to settle without a death spiral of
    // ever-smaller hops.
    StepFor(world, 900);
    EXPECT_LT(std::fabs(world.GetLinearVelocity(scene.ball).y), 0.05f);
    EXPECT_GE(world.GetContactCount(), 1);
    EXPECT_GT(world.GetPosition(scene.ball).y, 0.48f);
}

TEST(WorldResponseTest, DeepPenetrationCorrectionIsCappedPerStep) {
    World world(Vec2(0.0f, 0.0f)); // gravity off: isolate the position correction
    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    const BodyHandle ground = world.CreateBody(groundDef);
    ASSERT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));

    BodyDef ballDef;
    ballDef.type = BodyType::Dynamic;
    ballDef.position = Vec2(0.0f, -5.0f); // 5.5 m inside the solid half-space
    ballDef.density = 1.0f;
    const BodyHandle ball = world.CreateBody(ballDef);
    ASSERT_TRUE(world.AttachShape(ball, world.CreateCircleShape(kBallRadius)));

    world.Step(kDt);
    // pen = 0.5 - (-5) = 5.5 m; the raw correction would teleport the ball out in
    // one step, so kMaxLinearCorrection caps it at 0.2 m (Section 6.6 style).
    EXPECT_NEAR(world.GetPosition(ball).y, -5.0f + p2::kMaxLinearCorrection, 1.0e-6f);
}

TEST(WorldResponseTest, TwoIdenticalWorldsStayBitIdentical) {
    // Determinism (Section 7.4): the same scene, built with the same call order,
    // must produce bit-identical body states. Any hidden state (pointer-order
    // iteration, unordered containers, wall-clock input) would show up here.
    const auto buildScene = [](World& world) {
        std::vector<BodyHandle> bodies;
        BodyDef groundDef;
        groundDef.type = BodyType::Static;
        const BodyHandle ground = world.CreateBody(groundDef);
        const p2::ShapeHandle groundPlane = world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f);
        EXPECT_TRUE(world.AttachShape(ground, groundPlane));
        bodies.push_back(ground);

        const Vec2 dropPositions[2] = {Vec2(-0.9f, 2.5f), Vec2(0.9f, 1.5f)};
        const float restitutions[2] = {0.0f, 0.6f};
        for (int32_t index = 0; index < 2; ++index) {
            BodyDef ballDef;
            ballDef.type = BodyType::Dynamic;
            ballDef.position = dropPositions[index];
            ballDef.density = 1.0f;
            ballDef.restitution = restitutions[index];
            const BodyHandle ball = world.CreateBody(ballDef);
            const p2::ShapeHandle ballCircle = world.CreateCircleShape(0.4f);
            EXPECT_TRUE(world.AttachShape(ball, ballCircle));
            bodies.push_back(ball);
        }
        return bodies;
    };

    World worldA;
    World worldB;
    const std::vector<BodyHandle> bodiesA = buildScene(worldA);
    const std::vector<BodyHandle> bodiesB = buildScene(worldB);
    ASSERT_EQ(bodiesA.size(), bodiesB.size());

    StepFor(worldA, 300);
    StepFor(worldB, 300);
    EXPECT_EQ(worldA.GetContactCount(), worldB.GetContactCount());

    for (std::size_t index = 0; index < bodiesA.size(); ++index) {
        const Vec2 positionA = worldA.GetPosition(bodiesA[index]);
        const Vec2 positionB = worldB.GetPosition(bodiesB[index]);
        // Bitwise comparison: exactly the determinism guarantee the regression
        // tool (Week 6+) will rely on.
        EXPECT_EQ(std::memcmp(&positionA, &positionB, sizeof(Vec2)), 0);
        const float angleA = worldA.GetAngle(bodiesA[index]);
        const float angleB = worldB.GetAngle(bodiesB[index]);
        EXPECT_EQ(std::memcmp(&angleA, &angleB, sizeof(float)), 0);
    }
}

} // namespace
