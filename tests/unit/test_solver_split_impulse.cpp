// Week 5 P10 — split-impulse position layer tests.
//
// Covered contracts (every expectation is hand-derived in the comments):
//  - the pseudo-velocity layer NEVER reads or writes the real velocities
//    (bitwise comparison before/after), and clears the pseudo state afterwards;
//  - a single position iteration moves the pair apart by exactly
//    beta * (penetration - slop) along the normal (zero-lever case);
//  - with a lever arm the correction splits into a linear and an angular part and
//    still removes exactly beta of the excess at the contact anchor;
//  - positionIterations == 0 performs no correction at all;
//  - useSplitImpulse == false skips the position layer and consumes the
//    equivalent correction through the Baumgarte bias in Prepare;
//  - a 30-box tower keeps its penetration below the slop band with split impulse
//    and the comparison run (Baumgarte) is printed for the devlog.

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"
#include "physics2d/Solver/ContactSolver.h"

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::ContactSolver;
using p2::SolverSettings;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr float kTolerance = 1.0e-6f;

Body MakeBody(int32_t index, BodyType type, Vec2 position, Vec2 velocity = Vec2{0.0f, 0.0f},
              float angularVelocity = 0.0f, float mass = 1.0f, float inertia = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.linearVelocity = velocity;
    def.angularVelocity = angularVelocity;
    def.mass = mass;
    def.inertia = inertia;
    Body body;
    body.Initialize(def);
    body.handle = BodyHandle{index, 1u};
    return body;
}

Body MakeDynamic(int32_t index, Vec2 position, Vec2 velocity = Vec2{0.0f, 0.0f}, float angularVelocity = 0.0f,
                 float mass = 1.0f, float inertia = 1.0f) {
    return MakeBody(index, BodyType::Dynamic, position, velocity, angularVelocity, mass, inertia);
}

Body MakeStatic(int32_t index, Vec2 position = Vec2{0.0f, 0.0f}) {
    return MakeBody(index, BodyType::Static, position);
}

uint32_t FloatBits(float value) noexcept {
    return std::bit_cast<uint32_t>(value);
}

void ExpectVecBitsEqual(Vec2 a, Vec2 b) {
    EXPECT_EQ(FloatBits(a.x), FloatBits(b.x));
    EXPECT_EQ(FloatBits(a.y), FloatBits(b.y));
}

void ExpectTransformBitsEqual(const p2::Transform& a, const p2::Transform& b) {
    ExpectVecBitsEqual(a.p, b.p);
    EXPECT_EQ(FloatBits(a.q.c), FloatBits(b.q.c));
    EXPECT_EQ(FloatBits(a.q.s), FloatBits(b.q.s));
}

// dynamic A at (0, bodyY) above static B at the origin, contact normal (0, -1)
// pointing from A to B (down), shared anchor at world (0, 0). Returns both bodies
// and the one-point contact.
struct AnchoredContactScene {
    std::vector<Body> bodies;
    std::vector<Contact> contacts;
};

AnchoredContactScene MakeAnchoredContactScene(float bodyY, float penetrationGap = 0.0f,
                                              Vec2 velocity = Vec2{0.0f, 0.0f},
                                              float angularVelocity = 0.0f) {
    AnchoredContactScene scene;
    scene.bodies.push_back(MakeDynamic(0, Vec2{0.0f, bodyY}, velocity, angularVelocity, 1.0f, 1.0f));
    scene.bodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));

    Contact contact;
    contact.bodyA = scene.bodies[0].handle;
    contact.bodyB = scene.bodies[1].handle;
    contact.manifold.normal = Vec2{0.0f, -1.0f};
    // The narrow phase stores the same world anchor in both local frames; that
    // invariant is what makes "separation_now = Dot(worldB - worldA, n)" zero at
    // contact generation time.
    contact.manifold.penetration = penetrationGap;
    contact.manifold.pointCount = 1;
    contact.manifold.points[0].localPointA = Vec2{0.0f, -bodyY}; // anchor (0,0) in A's frame
    contact.manifold.points[0].localPointB = Vec2{0.0f, 0.0f};
    contact.manifold.points[0].id.type = p2::kContactTypeCircleCircle;
    scene.contacts.push_back(contact);
    return scene;
}

float AnchorSeparation(const AnchoredContactScene& scene) {
    const Body& bodyA = scene.bodies[0];
    const Body& bodyB = scene.bodies[1];
    const Vec2 worldA = p2::Mul(bodyA.xf, scene.contacts[0].manifold.points[0].localPointA);
    const Vec2 worldB = p2::Mul(bodyB.xf, scene.contacts[0].manifold.points[0].localPointB);
    return p2::Dot(worldB - worldA, scene.contacts[0].manifold.normal);
}

struct TowerMeasurements {
    float maxPenetration = 0.0f;
    float maxSpeed = 0.0f;
    float maxAbsAngle = 0.0f;
};

// Ground + 30 unit boxes. `useSplitImpulse` selects the position-correction
// mechanism: the pseudo-velocity layer (on) or the Baumgarte bias in the velocity
// layer (off). Measured over the last second of `stepCount`.
//
// `velocityIterations` is explicit because a 30:1 column is an unstable
// equilibrium whose support has to be re-established every frame: at the default
// 8 iterations the top of the column sags faster than the position layer may
// correct (measured: 0.7 m in 1 s) and the column topples after ~2 s. The Week 5
// *milestone* (10 boxes, default 8 iterations) is covered by the World stability
// test; this scene isolates the position layer on a column the velocity solver is
// actually holding up, so it uses the iteration count that makes the column
// stand (32 passes still propagate long-range support every frame).
TowerMeasurements RunThirtyBoxTower(bool useSplitImpulse, int32_t stepCount, int32_t velocityIterations) {
    World world; // gravity (0, -10)
    SolverSettings settings = world.GetSolverSettings();
    settings.useSplitImpulse = useSplitImpulse;
    world.SetSolverSettings(settings);

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.friction = 0.5f;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    std::vector<BodyHandle> boxes;
    for (int32_t index = 0; index < 30; ++index) {
        BodyDef boxDef;
        boxDef.type = BodyType::Dynamic;
        boxDef.position = Vec2{0.0f, 0.5f + static_cast<float>(index)};
        boxDef.friction = 0.5f;
        boxDef.density = 1.0f;
        const BodyHandle box = world.CreateBody(boxDef);
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(0.5f, 0.5f)));
        boxes.push_back(box);
    }

    constexpr int32_t kMeasureWindow = 60; // last simulated second
    TowerMeasurements measurements;
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt, velocityIterations, settings.positionIterations);
        if (step < stepCount - kMeasureWindow) {
            continue;
        }
        for (const Contact& contact : world.GetContacts()) {
            measurements.maxPenetration = std::max(measurements.maxPenetration, contact.manifold.penetration);
        }
        for (const BodyHandle box : boxes) {
            measurements.maxSpeed = std::max(measurements.maxSpeed, p2::Length(world.GetLinearVelocity(box)));
            measurements.maxAbsAngle = std::max(measurements.maxAbsAngle, std::fabs(world.GetAngle(box)));
        }
    }
    return measurements;
}

} // namespace

// ---------------------------------------------------------------------------
// P10 D1: the position layer must not touch the real velocity state.
// ---------------------------------------------------------------------------

TEST(SolverSplitImpulse, PositionLayerLeavesRealVelocitiesUntouched) {
    AnchoredContactScene scene =
        MakeAnchoredContactScene(/*bodyY=*/0.5f, /*penetrationGap=*/0.02f,
                                 /*velocity=*/Vec2{1.5f, -2.0f}, /*angularVelocity=*/0.7f);
    const p2::Transform transformABefore = scene.bodies[0].xf;
    const p2::Transform transformBBefore = scene.bodies[1].xf;
    const Vec2 velocityABefore = scene.bodies[0].linearVelocity;
    const Vec2 velocityBBefore = scene.bodies[1].linearVelocity;
    const float angularABefore = scene.bodies[0].angularVelocity;
    const float angularBBefore = scene.bodies[1].angularVelocity;

    SolverSettings settings;
    ContactSolver solver;
    solver.SolvePosition(scene.bodies, scene.contacts, kDt, settings.positionIterations, settings);

    // Bitwise unchanged: split impulse projects the correction into a separate
    // pseudo-velocity state, so it can neither add nor remove real kinetic energy.
    ExpectVecBitsEqual(scene.bodies[0].linearVelocity, velocityABefore);
    ExpectVecBitsEqual(scene.bodies[1].linearVelocity, velocityBBefore);
    EXPECT_EQ(FloatBits(scene.bodies[0].angularVelocity), FloatBits(angularABefore));
    EXPECT_EQ(FloatBits(scene.bodies[1].angularVelocity), FloatBits(angularBBefore));

    // The pseudo velocities are transient scratch: they must be cleared again.
    ExpectVecBitsEqual(scene.bodies[0].pseudoLinearVelocity, Vec2{0.0f, 0.0f});
    ExpectVecBitsEqual(scene.bodies[1].pseudoLinearVelocity, Vec2{0.0f, 0.0f});
    EXPECT_FLOAT_EQ(scene.bodies[0].pseudoAngularVelocity, 0.0f);
    EXPECT_FLOAT_EQ(scene.bodies[1].pseudoAngularVelocity, 0.0f);

    // ... but the transforms WERE corrected: A (above B, normal points down from A
    // to B) is pushed along -normal, i.e. upward, out of the overlap.
    EXPECT_GT(scene.bodies[0].xf.p.y, 0.5f);
    // The static body has zero inverse mass, so its pseudo velocity stayed zero
    // and its transform must be bit-identical.
    ExpectTransformBitsEqual(scene.bodies[1].xf, transformBBefore);
    // The dynamic body's correction is small (beta * excess), not a teleport.
    EXPECT_LT(std::fabs(scene.bodies[0].xf.p.y - transformABefore.p.y), p2::kMaxLinearCorrection);

    std::printf("[solver5] split-impulse isolation: vA=(%g,%g) wA=%g unchanged; dy=%g m\n",
                static_cast<double>(velocityABefore.x), static_cast<double>(velocityABefore.y),
                static_cast<double>(angularABefore),
                static_cast<double>(scene.bodies[0].xf.p.y - transformABefore.p.y));
}

// ---------------------------------------------------------------------------
// P10 D2: one position iteration removes exactly beta of the overlap excess.
// ---------------------------------------------------------------------------

TEST(SolverSplitImpulse, SinglePositionIterationMovesByBetaOfTheExcess) {
    // A dynamic (m = 1, I = 1) at y = 0.495 over static ground, normal (0, -1),
    // anchor (0,0). The lever arm rA = (0, -0.495) is parallel to the normal, so
    // K_n = invMassA = 1 and the impulse produces no torque.
    //   excess = penetration - slop = 0.05 - 0.005 = 0.045
    //   target = (beta/dt)*excess
    //   1 iteration: lambda = -(0 - target)/K_n = target -> pseudo vA = target
    //   integrate: dy = dt*target = beta*excess = 0.2*0.045 = 0.009 m
    // The anchor separation after the step must be 0.009 m, i.e. the overlap shrank
    // by exactly that amount.
    AnchoredContactScene scene = MakeAnchoredContactScene(/*bodyY=*/0.495f, /*penetrationGap=*/0.05f);
    SolverSettings settings;
    settings.positionIterations = 1;
    ContactSolver solver;
    solver.SolvePosition(scene.bodies, scene.contacts, kDt, 1, settings);

    const float expectedRise = settings.splitImpulseBeta * (0.05f - settings.splitImpulseSlop);
    const float actualRise = scene.bodies[0].xf.p.y - 0.495f;
    const float separation = AnchorSeparation(scene);
    std::printf("[solver5] split-impulse one iteration: dy=%.8f (expected %.8f), separation=%.8f\n",
                static_cast<double>(actualRise), static_cast<double>(expectedRise),
                static_cast<double>(separation));

    EXPECT_NEAR(actualRise, 0.009f, kTolerance);
    EXPECT_NEAR(separation, 0.009f, kTolerance);
    // No torque: the resulting angle is still exactly zero.
    EXPECT_FLOAT_EQ(scene.bodies[0].xf.q.s, 0.0f);
    EXPECT_FLOAT_EQ(scene.bodies[0].xf.q.c, 1.0f);

    // Real velocities were zero and stay zero.
    ExpectVecBitsEqual(scene.bodies[0].linearVelocity, Vec2{0.0f, 0.0f});
    EXPECT_FLOAT_EQ(scene.bodies[0].angularVelocity, 0.0f);
}

TEST(SolverSplitImpulse, LeverArmRotationContributesHalfOfTheCorrection) {
    // A dynamic (m = 1, I = 1) at the origin, B static at (0, 2), shared anchor at
    // (1, 0) -> rA = (1, 0), rB = (1, -2); normal (0, 1) -> K_n = 1 + 1*Cross(rA,n)^2
    // = 1 + 1 = 2, normalMass = 0.5.
    //   excess = 0.025 - 0.005 = 0.02; target = (beta/dt)*0.02 = 0.24 m/s
    //   1 iteration: lambda = -(0 - 0.24)*0.5 = 0.12
    //   pseudoLin = -J/m = (0, -0.12); pseudoAng = -Cross(rA, J)/I = -0.12
    //   integrate: dy = dt*(-0.12) = -0.002 m; dAngle = -0.12*dt = -0.002 rad
    // The anchor separation change is the sum of both terms:
    //   dy_anchor = dy + dAngle * rA.x = -0.002 + (-0.002)*1 = -0.004 m
    // so the separation grows by 0.004 = beta*excess, with the linear translation
    // supplying only half of it.
    AnchoredContactScene scene;
    scene.bodies.push_back(MakeDynamic(0, Vec2{0.0f, 0.0f}));
    scene.bodies.push_back(MakeStatic(1, Vec2{0.0f, 2.0f}));
    Contact contact;
    contact.bodyA = scene.bodies[0].handle;
    contact.bodyB = scene.bodies[1].handle;
    contact.manifold.normal = Vec2{0.0f, 1.0f};
    contact.manifold.penetration = 0.025f;
    contact.manifold.pointCount = 1;
    contact.manifold.points[0].localPointA = Vec2{1.0f, 0.0f};  // world (1, 0)
    contact.manifold.points[0].localPointB = Vec2{1.0f, -2.0f}; // world (1, 0)
    contact.manifold.points[0].id.type = p2::kContactTypeCircleCircle;
    scene.contacts.push_back(contact);

    SolverSettings settings;
    ContactSolver solver;
    solver.SolvePosition(scene.bodies, scene.contacts, kDt, 1, settings);

    EXPECT_NEAR(scene.bodies[0].xf.p.y, -0.002f, kTolerance);
    EXPECT_NEAR(scene.bodies[0].xf.p.x, 0.0f, kTolerance);
    EXPECT_NEAR(scene.bodies[0].xf.q.GetAngle(), -0.002f, kTolerance);
    const float separation = AnchorSeparation(scene);
    std::printf("[solver5] split-impulse lever arm: dy=%g dAngle=%g separation=%g (expected 0.004)\n",
                static_cast<double>(scene.bodies[0].xf.p.y),
                static_cast<double>(scene.bodies[0].xf.q.GetAngle()), static_cast<double>(separation));
    EXPECT_NEAR(separation, 0.004f, 1.0e-5f);

    // The correction was pure position/rotation: velocities untouched.
    ExpectVecBitsEqual(scene.bodies[0].linearVelocity, Vec2{0.0f, 0.0f});
    EXPECT_FLOAT_EQ(scene.bodies[0].angularVelocity, 0.0f);
}

// ---------------------------------------------------------------------------
// P10 D3: no iterations == no correction; disabled split impulse == Baumgarte.
// ---------------------------------------------------------------------------

TEST(SolverSplitImpulse, ZeroPositionIterationsPerformsNoCorrection) {
    AnchoredContactScene scene = MakeAnchoredContactScene(0.495f, 0.05f);
    const p2::Transform transformBefore = scene.bodies[0].xf;
    const Vec2 velocityBefore = scene.bodies[0].linearVelocity;

    SolverSettings settings;
    ContactSolver solver;
    solver.SolvePosition(scene.bodies, scene.contacts, kDt, /*iterations=*/0, settings);

    ExpectTransformBitsEqual(scene.bodies[0].xf, transformBefore);
    ExpectVecBitsEqual(scene.bodies[0].linearVelocity, velocityBefore);
    ExpectVecBitsEqual(scene.bodies[0].pseudoLinearVelocity, Vec2{0.0f, 0.0f});
    EXPECT_FLOAT_EQ(scene.bodies[0].pseudoAngularVelocity, 0.0f);
}

TEST(SolverSplitImpulse, SplitImpulseDisabledUsesTheBaumgarteBiasInstead) {
    // Same geometry: penetration 0.05, slop 0.005 -> excess 0.045.
    // Split impulse ON: the position layer owns the correction, so the velocity
    //   bias must stay zero (no double correction).
    // Split impulse OFF: Prepare adds bias = -(baumgarte/dt)*excess
    //   = -(0.2/dt)*0.045 = -0.54, a real separating velocity target that injects
    //   energy into the velocity layer - the documented comparison baseline.
    AnchoredContactScene splitScene = MakeAnchoredContactScene(0.495f, 0.05f);
    SolverSettings splitSettings; // useSplitImpulse = true by default
    ContactSolver splitSolver;
    splitSolver.Prepare(splitScene.bodies, splitScene.contacts, splitSettings, kDt);
    ASSERT_EQ(splitSolver.GetConstraintCount(), 1);
    EXPECT_FLOAT_EQ(splitSolver.GetConstraints()[0].bias, 0.0f);

    AnchoredContactScene baumgarteScene = MakeAnchoredContactScene(0.495f, 0.05f);
    SolverSettings baumgarteSettings;
    baumgarteSettings.useSplitImpulse = false;
    ContactSolver baumgarteSolver;
    baumgarteSolver.Prepare(baumgarteScene.bodies, baumgarteScene.contacts, baumgarteSettings, kDt);
    ASSERT_EQ(baumgarteSolver.GetConstraintCount(), 1);
    const float bias = baumgarteSolver.GetConstraints()[0].bias;
    const float expectedBias = -(baumgarteSettings.baumgarte / kDt) * (0.05f - baumgarteSettings.linearSlop);
    std::printf("[solver5] split-impulse off: bias=%.8f (expected %.8f)\n", static_cast<double>(bias),
                static_cast<double>(expectedBias));
    EXPECT_NEAR(bias, -0.54f, 1.0e-5f);
    EXPECT_NEAR(bias, expectedBias, 1.0e-5f);

    // With split impulse off the position layer must be a no-op (the bias above
    // already encodes the correction in the velocity layer).
    const p2::Transform transformBefore = baumgarteScene.bodies[0].xf;
    baumgarteSolver.SolvePosition(baumgarteScene.bodies, baumgarteScene.contacts, kDt,
                                  baumgarteSettings.positionIterations, baumgarteSettings);
    ExpectTransformBitsEqual(baumgarteScene.bodies[0].xf, transformBefore);
}

// ---------------------------------------------------------------------------
// P10 D4: 30-layer tower penetration, split impulse vs Baumgarte.
// ---------------------------------------------------------------------------

TEST(SolverSplitImpulse, ThirtyBoxTowerPenetrationStaysWithinTheSlopBand) {
    // The Week 5 position-layer acceptance scene: 30 unit boxes in a vertical
    // column, 5 simulated seconds (the column is an unstable equilibrium, so the
    // measurement window is the standing phase - see RunThirtyBoxTower).
    // 32 velocity iterations are used so the velocity solver can hold the 30:1
    // column; the position-layer question ("is the resting overlap inside the slop
    // band?") is independent of that choice. The control run uses the cheap
    // Baumgarte bias instead of the pseudo-velocity layer.
    constexpr int32_t kSteps = 300;
    constexpr int32_t kVelocityIterations = 32;

    // Context for the devlog: what the default 8-iteration budget does on the same
    // column (it sags/topples before the window, which is a velocity-iteration
    // limit, not a position-layer result).
    const TowerMeasurements defaultIterations = RunThirtyBoxTower(/*useSplitImpulse=*/true, kSteps, 8);
    std::printf("[solver5] tower30 splitImpulse=ON iters=8  maxPen=%.5f maxSpeed=%.5f max|angle|=%.5f\n",
                static_cast<double>(defaultIterations.maxPenetration),
                static_cast<double>(defaultIterations.maxSpeed),
                static_cast<double>(defaultIterations.maxAbsAngle));

    const TowerMeasurements split = RunThirtyBoxTower(/*useSplitImpulse=*/true, kSteps, kVelocityIterations);
    const TowerMeasurements baumgarte =
        RunThirtyBoxTower(/*useSplitImpulse=*/false, kSteps, kVelocityIterations);

    std::printf("[solver5] tower30 splitImpulse=ON  iters=%d maxPen=%.5f maxSpeed=%.5f max|angle|=%.5f\n",
                kVelocityIterations, static_cast<double>(split.maxPenetration),
                static_cast<double>(split.maxSpeed), static_cast<double>(split.maxAbsAngle));
    std::printf("[solver5] tower30 splitImpulse=OFF iters=%d maxPen=%.5f maxSpeed=%.5f max|angle|=%.5f\n",
                kVelocityIterations, static_cast<double>(baumgarte.maxPenetration),
                static_cast<double>(baumgarte.maxSpeed), static_cast<double>(baumgarte.maxAbsAngle));

    // The pseudo-velocity layer targets "penetration <= slop", so the resting
    // overlap must stay inside the slop band (5 mm).
    EXPECT_LT(split.maxPenetration, p2::kLinearSlop);
    EXPECT_LT(split.maxAbsAngle, 0.1f) << "the tower must stay upright, not topple";
    // The column is still dynamically settling at 5 s (its sway dominates the
    // body speeds), so the "no energy injection" difference is measured on the
    // dedicated resting-contact test below instead of on this scene.
}

// ---------------------------------------------------------------------------
// P10 D5: split impulse must not inject real velocity, Baumgarte must.
// ---------------------------------------------------------------------------

TEST(SolverSplitImpulse, RestingContactDoesNotGainVelocityFromPositionCorrection) {
    // A box starts 2 cm inside the ground (excess = 2 cm - 5 mm slop = 1.5 cm)
    // with zero velocity and rests there.
    //   Baumgarte: bias = -(baumgarte/dt)*excess = -12*0.015 = -0.18 m/s, i.e. the
    //     velocity solver is asked to give the box a real upward velocity of
    //     0.18 m/s. The box jumps/jitters: max speed over the run > 0.1 m/s.
    //   Split impulse: the same 1.5 cm is removed by the pseudo-velocity layer,
    //     which never touches the real velocities: max speed stays near zero.
    struct Run {
        float maxSpeed;
        float maxPenetration;
    };
    const auto runRelease = [](bool useSplitImpulse) {
        World world; // gravity (0, -10)
        SolverSettings settings = world.GetSolverSettings();
        settings.useSplitImpulse = useSplitImpulse;
        world.SetSolverSettings(settings);
        BodyDef groundDef;
        groundDef.type = BodyType::Static;
        groundDef.friction = 0.5f;
        const BodyHandle ground = world.CreateBody(groundDef);
        EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));
        BodyDef boxDef;
        boxDef.type = BodyType::Dynamic;
        boxDef.position = Vec2{0.0f, 0.48f}; // 2 cm overlap with the ground
        boxDef.friction = 0.5f;
        boxDef.density = 1.0f;
        const BodyHandle box = world.CreateBody(boxDef);
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(0.5f, 0.5f)));
        Run result{0.0f, 0.0f};
        for (int32_t step = 0; step < 120; ++step) {
            world.Step(kDt);
            result.maxSpeed = std::max(result.maxSpeed, p2::Length(world.GetLinearVelocity(box)));
            for (const Contact& contact : world.GetContacts()) {
                result.maxPenetration = std::max(result.maxPenetration, contact.manifold.penetration);
            }
        }
        return result;
    };

    const Run split = runRelease(/*useSplitImpulse=*/true);
    const Run baumgarte = runRelease(/*useSplitImpulse=*/false);
    std::printf("[solver5] resting-overlap max|v|: split=%.5f baumgarte=%.5f m/s (split maxPen=%.5f)\n",
                static_cast<double>(split.maxSpeed), static_cast<double>(baumgarte.maxSpeed),
                static_cast<double>(split.maxPenetration));

    // Split impulse: no real velocity is created by the correction.
    EXPECT_LT(split.maxSpeed, 0.05f);
    // Baumgarte: the same overlap is removed through a genuine 0.18 m/s target,
    // so the real velocity state is polluted - the documented comparison.
    EXPECT_GT(baumgarte.maxSpeed, 0.1f);
    EXPECT_LT(split.maxSpeed, baumgarte.maxSpeed);
}
