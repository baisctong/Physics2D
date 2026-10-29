// Week 3 D6/P08: integration tests for the Sequential Impulse solver wired into
// World::Step (phases: UpdateContacts -> PrepareConstraints -> WarmStart ->
// SolveVelocityConstraints -> IntegratePositions -> split-impulse position layer).
//
// The tests use only the public World/Body API so they pin the observable
// physical contract, not the solver internals:
//   - equal-mass head-on: e = 0 is perfectly inelastic (both bodies leave with
//     the momentum-weighted common velocity v/2); e = 1 exchanges velocities;
//   - momentum is conserved to float accuracy in every collision;
//   - energy never increases for e = 0;
//   - a resting body's penetration is the slop band (kLinearSlop), because the
//     velocity solver removes the gravity increment before positions integrate;
//   - restitution is the ratio of rebound/approach speed for e = 0.8;
//   - the restitution threshold suppresses low-speed micro-bounces;
//   - the whole pipeline stays bit-deterministic.
//
// Discrete-time note: contacts are generated in phase 3 (before positions
// integrate) and solved in the same step, so a head-on collision resolves within
// one step. EXPECT_NEAR tolerances below are sized for a handful of float
// operations plus the number of solver iterations, not for simulation drift.

#include <gtest/gtest.h>

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr float kRadius = 0.5f;
constexpr float kGravity = 10.0f; // magnitude of kDefaultGravityY

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

// Dynamic circle with an explicit mass (via density) and material.
BodyHandle AddCircle(World& world, Vec2 position, float density, float restitution = 0.0f,
                     Vec2 velocity = Vec2(0.0f, 0.0f)) {
    BodyDef def;
    def.type = BodyType::Dynamic;
    def.position = position;
    def.linearVelocity = velocity;
    def.density = density;
    def.restitution = restitution;
    const BodyHandle body = world.CreateBody(def);
    EXPECT_TRUE(world.AttachShape(body, world.CreateCircleShape(kRadius)));
    return body;
}

BodyHandle AddGround(World& world) {
    BodyDef def;
    def.type = BodyType::Static;
    const BodyHandle body = world.CreateBody(def);
    EXPECT_TRUE(world.AttachShape(body, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
    return body;
}

float KineticEnergy(const World& world, const std::vector<BodyHandle>& bodies) {
    float energy = 0.0f;
    for (BodyHandle body : bodies) {
        const p2::Body* data = world.GetBody(body);
        if (data == nullptr || !data->IsDynamic()) {
            continue;
        }
        const Vec2 velocity = data->linearVelocity;
        energy += 0.5f * data->mass * p2::Dot(velocity, velocity);
        energy += 0.5f * data->inertia * data->angularVelocity * data->angularVelocity;
    }
    return energy;
}

TEST(WorldImpulseTest, EqualMassHeadOnIsInelasticAtZeroRestitution) {
    // Perfectly inelastic collision of equal masses: momentum conservation gives
    // the common velocity (m*v0)/(2m) = v0/2 = 1 m/s for v0 = 2 m/s. The relative
    // normal velocity is driven to zero, which is exactly e = 0.
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle bodyA = AddCircle(world, Vec2(-0.5f, 0.0f), 1.0f, 0.0f, Vec2(2.0f, 0.0f));
    const BodyHandle bodyB = AddCircle(world, Vec2(0.5f, 0.0f), 1.0f, 0.0f);

    world.Step(kDt);
    const float velocityA = world.GetLinearVelocity(bodyA).x;
    const float velocityB = world.GetLinearVelocity(bodyB).x;
    EXPECT_NEAR(velocityA, 1.0f, 1.0e-4f);
    EXPECT_NEAR(velocityB, 1.0f, 1.0e-4f);
    // Momentum before == after: m*2 == m*1 + m*1.
    const float momentumAfter = world.GetMass(bodyA) * velocityA + world.GetMass(bodyB) * velocityB;
    EXPECT_NEAR(momentumAfter, 2.0f * world.GetMass(bodyA), 1.0e-4f);
}

TEST(WorldImpulseTest, EqualMassElasticHeadOnSwapsVelocities) {
    // e = 1 target: relative normal velocity reverses (-v0). For equal masses
    // both momentum and energy are conserved, so the velocities exchange: the
    // incoming body stops and the resting body leaves at v0.
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle bodyA = AddCircle(world, Vec2(-0.5f, 0.0f), 1.0f, 1.0f, Vec2(2.0f, 0.0f));
    const BodyHandle bodyB = AddCircle(world, Vec2(0.5f, 0.0f), 1.0f, 1.0f);

    world.Step(kDt);
    EXPECT_NEAR(world.GetLinearVelocity(bodyA).x, 0.0f, 1.0e-3f);
    EXPECT_NEAR(world.GetLinearVelocity(bodyB).x, 2.0f, 1.0e-3f);
}

TEST(WorldImpulseTest, MassRatioCollisionConservesMomentumAndLosesEnergy) {
    // Same radius, density 1 vs 3 -> mB = 3 * mA. A head-on e = 0 collision ends
    // with the common momentum-weighted velocity:
    //   v' = mA * v0 / (mA + mB) = 3/4 = 0.75 m/s for v0 = 3 m/s.
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle bodyA = AddCircle(world, Vec2(-0.5f, 0.0f), 1.0f, 0.0f, Vec2(3.0f, 0.0f));
    const BodyHandle bodyB = AddCircle(world, Vec2(0.5f, 0.0f), 3.0f, 0.0f);
    ASSERT_NEAR(world.GetMass(bodyB), 3.0f * world.GetMass(bodyA), 1.0e-5f);

    const std::vector<BodyHandle> bodies{bodyA, bodyB};
    const float momentumBefore = world.GetMass(bodyA) * 3.0f;
    const float energyBefore = KineticEnergy(world, bodies);

    world.Step(kDt);
    const float momentumAfter = world.GetMass(bodyA) * world.GetLinearVelocity(bodyA).x +
                                world.GetMass(bodyB) * world.GetLinearVelocity(bodyB).x;
    EXPECT_NEAR(momentumAfter, momentumBefore, 1.0e-4f);
    EXPECT_NEAR(world.GetLinearVelocity(bodyA).x, 0.75f, 1.0e-4f);
    EXPECT_NEAR(world.GetLinearVelocity(bodyB).x, 0.75f, 1.0e-4f);
    // Inelastic collision: kinetic energy must not increase (it is lost to the
    // constraint's inelastic closure). e = 0 -> strictly less energy.
    const float energyAfter = KineticEnergy(world, bodies);
    EXPECT_LE(energyAfter, energyBefore + 1.0e-5f);
    EXPECT_LT(energyAfter, energyBefore);
}

TEST(WorldImpulseTest, BallSettlesOnGroundWithSlopPenetration) {
    // With the velocity solver, the resting state is:
    //   phase 1 adds -g*dt to v; phase 6 drives the normal velocity to 0;
    //   phase 7 integrates (no motion); phase 8 removes the overlap above the
    //   slop band with the split-impulse pseudo-velocity layer (beta per step).
    // The fixed point of that correction is penetration = kLinearSlop = 0.005 m
    // exactly (better than Week 2's slop + g*dt^2 = 0.00778 m), and the position
    // is constant.
    World world;
    const BodyHandle ground = AddGround(world);
    const BodyHandle ball = AddCircle(world, Vec2(0.0f, 3.0f), 1.0f);

    StepFor(world, 180); // 3 simulated seconds
    EXPECT_LT(std::fabs(world.GetLinearVelocity(ball).y), 0.02f);
    ASSERT_GE(world.GetContactCount(), 1);
    const float penetration = world.GetContacts()[0].manifold.penetration;
    EXPECT_LE(penetration, p2::kLinearSlop * 1.5f);
    EXPECT_NEAR(penetration, p2::kLinearSlop, p2::kLinearSlop * 0.2f);

    const float heightAtTwoSeconds = world.GetPosition(ball).y;
    StepFor(world, 60);
    const float heightAtThreeSeconds = world.GetPosition(ball).y;
    EXPECT_LT(std::fabs(heightAtThreeSeconds - heightAtTwoSeconds), 1.0e-3f);
    EXPECT_EQ(world.GetPosition(ground), Vec2(0.0f, 0.0f));
}

TEST(WorldImpulseTest, ThreeBallChainCascadesVelocityToTheLastBall) {
    // Newton's cradle, staged with small gaps so each impact resolves on its own
    // frame: ball1 hits ball2 (elastic, e = 1 -> velocities exchange), ball2 then
    // hits ball3 and stops. End state: ball3 carries the incoming 2 m/s.
    //
    // Why gaps: a restitution bias is computed once per frame in Prepare(). In a
    // chain that is already touching, the downstream contact sees vn0 ~ 0 and
    // gets no bias, so the solver would converge to the mixed elastic/inelastic
    // solution instead of the ideal cradle. The staged cascade is the correct way
    // to test momentum transfer with this Weekend-3 solver.
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle first = AddCircle(world, Vec2(-1.5f, 0.0f), 1.0f, 1.0f, Vec2(2.0f, 0.0f));
    const BodyHandle middle = AddCircle(world, Vec2(0.0f, 0.0f), 1.0f, 1.0f);
    const BodyHandle last = AddCircle(world, Vec2(1.6f, 0.0f), 1.0f, 1.0f);

    StepFor(world, 120); // 2 s: both impacts happen well inside this window
    EXPECT_NEAR(world.GetLinearVelocity(last).x, 2.0f, 1.0e-3f);
    EXPECT_NEAR(world.GetLinearVelocity(first).x, 0.0f, 1.0e-3f);
    EXPECT_NEAR(world.GetLinearVelocity(middle).x, 0.0f, 1.0e-3f);
    // Elastic equal-mass impacts conserve momentum: total p = m * 2 m/s.
    const float momentum = world.GetMass(first) * world.GetLinearVelocity(first).x +
                           world.GetMass(middle) * world.GetLinearVelocity(middle).x +
                           world.GetMass(last) * world.GetLinearVelocity(last).x;
    EXPECT_NEAR(momentum, 2.0f * world.GetMass(first), 1.0e-3f);
}

TEST(WorldImpulseTest, MoreIterationsReduceTheResidualNormalVelocity) {
    // Two stacked balls on the ground: the two contacts are coupled, so a single
    // velocity iteration cannot satisfy both. Analytic one-pass result (equal
    // masses, gravity step -a with a = g*dt): contact (ground, ball1) zeroes ball1
    // (v1 = 0), then contact (ball1, ball2) moves -a back onto ball1 and onto
    // ball2 (v1 = -a, v2 = 0). With enough iterations both converge to 0.
    const auto buildStack = [](World& world) {
        struct Stack {
            BodyHandle ground;
            BodyHandle lower;
            BodyHandle upper;
        };
        Stack stack;
        stack.ground = AddGround(world);
        stack.lower = AddCircle(world, Vec2(0.0f, 0.5f), 1.0f);
        stack.upper = AddCircle(world, Vec2(0.0f, 1.5f), 1.0f);
        return stack;
    };

    World oneIteration;
    World eightIterations;
    const auto stackOne = buildStack(oneIteration);
    const auto stackEight = buildStack(eightIterations);

    oneIteration.Step(kDt, 1, 3);
    eightIterations.Step(kDt, 8, 3);

    const float residualOne = std::fabs(oneIteration.GetLinearVelocity(stackOne.lower).y);
    const float residualEight = std::fabs(eightIterations.GetLinearVelocity(stackEight.lower).y);
    std::printf("[convergence] residual |v_lower.y|: 1 iter = %.6f, 8 iters = %.6f (analytic 1-pass %.6f)\n",
                static_cast<double>(residualOne), static_cast<double>(residualEight),
                static_cast<double>(0.5f * kGravity * kDt));
    EXPECT_GT(residualOne, residualEight);
    EXPECT_LT(residualEight, 0.01f);
    // Analytic one-pass residual (equal masses; the dynamic-dynamic effective mass
    // is K = 1 + 1 = 2, so its normal mass is 1/2):
    //   contact (ground, lower): drives lower to 0;
    //   contact (lower, upper): lambda = (g*dt) * (1/2) -> lower = -g*dt/2.
    // Each further pass halves the error, so 8 iterations leave ~g*dt/256 = 6.5e-4.
    EXPECT_NEAR(residualOne, 0.5f * kGravity * kDt, 1.0e-3f);
    EXPECT_LT(residualEight, residualOne / 50.0f);

    // Sanity: with enough iterations the whole stack stays put (velocities are
    // driven to zero and the position phase only removes slop).
    StepFor(eightIterations, 60);
    EXPECT_LT(std::fabs(eightIterations.GetLinearVelocity(stackEight.upper).y), 0.02f);
}

TEST(WorldImpulseTest, ZeroRestitutionDoesNotBounceFromFiveMetres) {
    World world;
    AddGround(world);
    const BodyHandle ball = AddCircle(world, Vec2(0.0f, 5.0f), 1.0f, 0.0f);

    bool contactSeen = false;
    float maxUpwardVelocity = 0.0f;
    for (int32_t step = 0; step < 240; ++step) {
        world.Step(kDt);
        contactSeen = contactSeen || world.GetContactCount() > 0;
        if (contactSeen) {
            maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(ball).y);
        }
    }
    EXPECT_TRUE(contactSeen);
    // e = 0 drives the normal velocity to exactly zero; no rebound.
    EXPECT_LT(maxUpwardVelocity, 0.05f);
}

TEST(WorldImpulseTest, RestitutionPointEightReboundsNearTheAnalyticPeak) {
    // Drop from centre height 5 m onto the ground (contact at centre 0.5 m):
    //   continuous impact speed v = sqrt(2 * g * (5 - 0.5)) = sqrt(90) = 9.487 m/s
    //   rebound speed v' = e * v = 0.8 * 9.487 = 7.590 m/s
    //   energy-height ratio e^2 = 0.64, so the analytic apex is
    //     0.5 + 0.64 * 4.5 = 3.38 m  (the task's "0.64 * 5 = 3.2 m" ignores the
    //     0.5 m contact offset).
    //
    // Week 5 measures apex 3.4633 m (impact 9.5000 m/s before the contact frame,
    // rebound 7.7333 m/s, ratio 0.8140). The remaining gap above the analytic
    // apex is the discrete-time impact-velocity bias, not position-layer energy
    // injection: the restitution bias is captured in Prepare after the gravity
    // step, so the rebound target is e * (impact + g*dt) = 0.8 * 9.667 = 7.733
    // m/s, and sqrt(2) discretisation of the apex costs another ~0.02 m. The
    // split-impulse position layer never writes into the real velocity (pinned by
    // SplitImpulseDoesNotInjectEnergyIntoRealVelocities in test_world_stability);
    // Week 3's TEMPORARY push-out, which lifted the ball before the rebound and
    // added up to (penetration - slop) of potential energy, is gone.
    // The test pins the fundamental contract (rebound/impact ratio = e and a
    // bounded, sub-drop apex) with the 3.0-3.6 m band.
    World world;
    AddGround(world);
    const BodyHandle ball = AddCircle(world, Vec2(0.0f, 5.0f), 1.0f, 0.8f);

    bool contactSeen = false;
    float impactSpeed = 0.0f;
    float peakHeightAfterContact = 0.0f;
    float maxUpwardVelocity = 0.0f;
    for (int32_t step = 0; step < 300; ++step) {
        world.Step(kDt);
        const float velocity = world.GetLinearVelocity(ball).y;
        if (world.GetContactCount() > 0) {
            contactSeen = true;
        }
        if (!contactSeen) {
            impactSpeed = std::max(impactSpeed, -velocity);
        } else {
            peakHeightAfterContact = std::max(peakHeightAfterContact, world.GetPosition(ball).y);
            maxUpwardVelocity = std::max(maxUpwardVelocity, velocity);
        }
    }
    EXPECT_TRUE(contactSeen);
    std::printf("[world5] restitution 0.8: impact speed = %.4f m/s, rebound speed = %.4f m/s, "
                "rebound/impact = %.4f, apex = %.4f m (analytic 3.38 m)\n",
                static_cast<double>(impactSpeed), static_cast<double>(maxUpwardVelocity),
                static_cast<double>(maxUpwardVelocity / impactSpeed),
                static_cast<double>(peakHeightAfterContact));
    EXPECT_GT(impactSpeed, 9.0f);
    EXPECT_GT(maxUpwardVelocity, 5.0f); // clear rebound, not a positional artefact
    // Restitution semantics: rebound / impact speed == e (to within discrete error).
    EXPECT_NEAR(maxUpwardVelocity / impactSpeed, 0.8f, 0.08f);
    EXPECT_GT(peakHeightAfterContact, 3.0f);
    EXPECT_LT(peakHeightAfterContact, 3.6f);
    // A rebound can never exceed the drop height (energy is not injected).
    EXPECT_LT(peakHeightAfterContact, 5.0f);
}

TEST(WorldImpulseTest, RestitutionThresholdSuppressesLowSpeedBounce) {
    // Drop from 4 cm above the ground: impact speed sqrt(2 * 10 * 0.04) = 0.894
    // m/s is below kRestitutionVelocityThreshold (1 m/s), so even with e = 0.8
    // the restitution bias is zero and the ball must not bounce at all.
    World world;
    AddGround(world);
    const BodyHandle ball = AddCircle(world, Vec2(0.0f, 0.5f + 0.04f), 1.0f, 0.8f);

    bool contactSeen = false;
    float maxUpwardVelocity = 0.0f;
    for (int32_t step = 0; step < 180; ++step) {
        world.Step(kDt);
        contactSeen = contactSeen || world.GetContactCount() > 0;
        if (contactSeen) {
            maxUpwardVelocity = std::max(maxUpwardVelocity, world.GetLinearVelocity(ball).y);
        }
    }
    EXPECT_TRUE(contactSeen);
    EXPECT_LT(maxUpwardVelocity, 0.05f);
}

TEST(WorldImpulseTest, StaticGroundNeverMoves) {
    World world;
    const BodyHandle ground = AddGround(world);
    const BodyHandle ball = AddCircle(world, Vec2(0.0f, 5.0f), 1.0f, 0.8f);
    ASSERT_TRUE(world.IsValid(ball));

    StepFor(world, 300);
    EXPECT_EQ(world.GetPosition(ground), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetAngle(ground), 0.0f);
    EXPECT_EQ(world.GetLinearVelocity(ground), Vec2(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world.GetAngularVelocity(ground), 0.0f);
}

TEST(WorldImpulseTest, TwoIdenticalWorldsStayBitIdenticalWithTheSolver) {
    // Determinism through the new phases (Prepare/iterations are pure functions of
    // the contact list): identical call order -> identical float bits.
    const auto buildScene = [](World& world) {
        std::vector<BodyHandle> bodies;
        bodies.push_back(AddGround(world));
        bodies.push_back(AddCircle(world, Vec2(-0.9f, 3.0f), 1.0f, 0.0f));
        bodies.push_back(AddCircle(world, Vec2(0.9f, 2.0f), 1.0f, 0.5f));
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
        EXPECT_EQ(std::memcmp(&positionA, &positionB, sizeof(Vec2)), 0);
        const float angleA = worldA.GetAngle(bodiesA[index]);
        const float angleB = worldB.GetAngle(bodiesB[index]);
        EXPECT_EQ(std::memcmp(&angleA, &angleB, sizeof(float)), 0);
    }
}

} // namespace
