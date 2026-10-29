// Week 2/3 fall-on-plane response behaviour tests.
//
// Week 3 replaced the temporary velocity projection with the Sequential Impulse
// solver (phase 6). The observable behaviour these tests pin:
//   - e = 0: the ball settles, |vy| ~ 0 and the steady penetration is exactly the
//     slop band (see the derivation below);
//   - e > 0: the ball bounces, the rebound peak is below the drop height, and
//     low-speed contact below the restitution threshold does not bounce;
//   - the static ground never moves;
//   - two identical worlds produce bit-identical trajectories (determinism).
//
// Steady-state penetration derivation with the real solver (one fixed step):
//   phase 1: v = -g * dt
//   phase 6: the solver drives the normal velocity to 0 (e = 0)
//   phase 7: y does not move (v == 0)
//   phase 8: y += (pen - kLinearSlop), pen measured at the start of the step
// so at the fixed point pen* = kLinearSlop = 0.005 m exactly, i.e.
// y* = 0.5 - pen* = 0.495 m. (Week 2's temporary projection reached
// slop + g*dt^2 = 0.00778 m because it integrated the gravity velocity first.)

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

    // With the velocity solver the gravity increment is removed before positions
    // integrate, so the fixed point is exactly the slop band (see file header).
    const float expectedPenetration = p2::kLinearSlop;
    EXPECT_NEAR(ContactPenetration(world), expectedPenetration, 1.0e-4f);
    EXPECT_NEAR(world.GetPosition(scene.ball).y, kBallRadius - expectedPenetration, 1.0e-4f);
}

TEST(WorldResponseTest, ElasticityZeroNeverBounces) {
    World world;
    const FallingScene scene = MakeFallingScene(world, 0.0f);

    // Track the largest upward velocity after the first contact: with e = 0 the
    // solver drives the relative normal velocity to 0, so the ball may not rebound.
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
    // the restitution constraint removes energy (e < 1), it never injects it.
    EXPECT_GT(peakHeightAfterContact, kBallRadius + 0.1f);
    EXPECT_LT(peakHeightAfterContact, dropHeight);
    // The ground is still static after all impacts.
    EXPECT_EQ(world.GetPosition(scene.ground), Vec2(0.0f, 0.0f));
}

TEST(WorldResponseTest, LowSpeedContactIsAbsorbedWithoutBounce) {
    // Restitution threshold contract (Section 6.4): an approach slower than
    // kRestitutionVelocityThreshold (1 m/s) must be absorbed, not bounced.
    //
    // Week 3 measured a low-speed limit cycle (~0.9 m/s) instead of a hard stop
    // because the TEMPORARY position push-out lifted the ball by up to
    // (penetration - slop) on every impact. Week 5 removed that hack: the
    // split-impulse position layer corrects overlap without touching the real
    // velocity, so the velocity-level threshold contract is all that remains to
    // pin here: a slow contact is absorbed.
    World world;
    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    const BodyHandle ground = world.CreateBody(groundDef);
    ASSERT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));

    BodyDef ballDef;
    ballDef.type = BodyType::Dynamic;
    ballDef.position = Vec2(0.0f, kBallRadius - 0.01f); // already touching/penetrating
    ballDef.linearVelocity = Vec2(0.0f, -0.5f);         // 0.5 m/s < threshold 1 m/s
    ballDef.density = 1.0f;
    ballDef.restitution = 0.8f;
    const BodyHandle ball = world.CreateBody(ballDef);
    ASSERT_TRUE(world.AttachShape(ball, world.CreateCircleShape(kBallRadius)));

    world.Step(kDt);
    // The solver drives the normal velocity to zero; it must not flip it upward.
    EXPECT_GE(world.GetLinearVelocity(ball).y, -0.01f);
    EXPECT_LE(world.GetLinearVelocity(ball).y, 0.01f);

    // And the body stays resting: no repeated micro-bounces from this state.
    float maxUpwardVelocity = 0.0f;
    for (int32_t step = 0; step < 120; ++step) {
        world.Step(kDt);
        maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(ball).y);
    }
    EXPECT_LT(maxUpwardVelocity, 0.05f);
    EXPECT_NEAR(world.GetPosition(ball).y, kBallRadius - p2::kLinearSlop, 0.01f);
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
