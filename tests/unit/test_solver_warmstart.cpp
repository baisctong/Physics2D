// Week 5 P09 D4 — warm starting tests.
//
// Covered contracts (hand-derived expectations in the comments):
//  - Prepare seeds the accumulator with inherited * warmStartFactor, and
//    WarmStart applies exactly the seed once (no double-counting);
//  - restitution bias is captured in Prepare (BEFORE warm start) from the initial
//    approach speed, so warm-start impulses cannot re-trigger bouncing;
//  - warmStartFactor == 0 and useWarmStarting == false are bit-identical
//    (same code path, both seeds exactly +0.0f);
//  - warm starting removes most of the first-iteration residual on a stack whose
//    previous solution is inherited (residual = 0.5*(1 - factor)*g*dt per frame
//    for the two-contact test frame), and the factor 0/0.5/1.0 comparison is
//    printed for the devlog;
//  - with the shipped default factor (1.0, Lead's Week 5 measurement) the 10-box
//    tower stands for 10 s while the cold baseline keeps the Week 4 collapse.

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
using p2::ContactPoint;
using p2::ContactSolver;
using p2::SolverSettings;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr float kTolerance = 1.0e-6f;
constexpr float kGravity = 10.0f;

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

Contact MakeCentralContact(BodyHandle bodyA, BodyHandle bodyB, Vec2 normal, float friction = 0.0f,
                           float restitution = 0.0f) {
    Contact contact;
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    contact.manifold.normal = normal;
    contact.manifold.penetration = 0.0f;
    contact.manifold.pointCount = 1;
    contact.manifold.points[0].localPointA = Vec2{0.0f, 0.0f};
    contact.manifold.points[0].localPointB = Vec2{0.0f, 0.0f};
    contact.manifold.points[0].id.type = p2::kContactTypeCircleCircle;
    contact.friction = friction;
    contact.restitution = restitution;
    return contact;
}

float NormalRelativeVelocity(const ContactSolver& solver, const std::vector<Body>& bodies,
                             const std::vector<Contact>& contacts, size_t constraintIndex) {
    const p2::ContactConstraintPoint& constraint = solver.GetConstraints().at(constraintIndex);
    const Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
    const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
    const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
    return p2::Dot(ContactSolver::ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB),
                   constraint.normal);
}

float MaxResidualNormalVelocity(const ContactSolver& solver, const std::vector<Body>& bodies,
                                const std::vector<Contact>& contacts) {
    float residual = 0.0f;
    for (size_t i = 0; i < solver.GetConstraints().size(); ++i) {
        residual = std::max(residual, std::fabs(NormalRelativeVelocity(solver, bodies, contacts, i)));
    }
    return residual;
}

uint32_t FloatBits(float value) noexcept {
    return std::bit_cast<uint32_t>(value);
}

// ---------------------------------------------------------------------------
// The canonical stack frame: ground <- body1 <- body2 with all three bodies
// pre-loaded with the post-gravity velocity (0, -g*dt). The previous frame's
// converged solution is inherited: contact0 (ground->body1) carries the impulse
// for both bodies, 2*m*g*dt, and contact1 (body1->body2) carries m*g*dt for the
// top body. With warmStartFactor == 1 no iteration is needed to cancel gravity.
// ---------------------------------------------------------------------------
struct StackFrame {
    std::vector<Body> bodies;
    std::vector<Contact> contacts;
};

StackFrame MakeStackFrame() {
    StackFrame frame;
    const float fallSpeed = kGravity * kDt; // 0.16666667 m/s downward after one gravity step
    frame.bodies.push_back(MakeStatic(0, Vec2{0.0f, 0.0f}));
    frame.bodies.push_back(MakeDynamic(1, Vec2{0.0f, 0.0f}, Vec2{0.0f, -fallSpeed}));
    frame.bodies.push_back(MakeDynamic(2, Vec2{0.0f, 0.0f}, Vec2{0.0f, -fallSpeed}));

    Contact groundContact =
        MakeCentralContact(frame.bodies[0].handle, frame.bodies[1].handle, Vec2{0.0f, 1.0f});
    groundContact.isPersistent = true;
    groundContact.manifold.points[0].normalImpulse = 2.0f * fallSpeed;
    Contact middleContact =
        MakeCentralContact(frame.bodies[1].handle, frame.bodies[2].handle, Vec2{0.0f, 1.0f});
    middleContact.isPersistent = true;
    middleContact.manifold.points[0].normalImpulse = fallSpeed;

    frame.contacts.push_back(groundContact);
    frame.contacts.push_back(middleContact);
    return frame;
}

struct StackSolveResult {
    float residualAfterOneIteration = 0.0f;
    float body1Velocity = 0.0f;
    float body2Velocity = 0.0f;
};

StackSolveResult SolveStackFrame(float warmStartFactor, bool useWarmStarting) {
    StackFrame frame = MakeStackFrame();
    SolverSettings settings;
    settings.warmStartFactor = warmStartFactor;
    settings.useWarmStarting = useWarmStarting;
    ContactSolver solver;
    solver.Prepare(frame.bodies, frame.contacts, settings);
    solver.WarmStart(frame.bodies, frame.contacts, settings);
    solver.SolveVelocity(frame.bodies, frame.contacts, 1, settings);
    StackSolveResult result;
    result.residualAfterOneIteration = MaxResidualNormalVelocity(solver, frame.bodies, frame.contacts);
    result.body1Velocity = frame.bodies[1].linearVelocity.y;
    result.body2Velocity = frame.bodies[2].linearVelocity.y;
    return result;
}

struct TowerOutcome {
    float maxPenetration = 0.0f; // worst penetration over the last second of the run
    float maxSpeed = 0.0f;       // worst linear body speed over the last second
    float maxAbsX = 0.0f;        // sideways spread: a collapsed tower scatters
    float maxAbsAngle = 0.0f;
    float finalSpacingError = 0.0f; // |y_top - 9.5| for the top box of the 10-tower
};

// Ground + ten unit boxes stacked at y = 0.5, 1.5, ... 9.5 (the Week 5 milestone
// scene). `warmStartFactor` is applied through World's SolverSettings.
TowerOutcome RunTenBoxTower(float warmStartFactor, bool useWarmStarting, int32_t stepCount) {
    World world; // gravity (0, -10)
    SolverSettings settings = world.GetSolverSettings();
    settings.warmStartFactor = warmStartFactor;
    settings.useWarmStarting = useWarmStarting;
    world.SetSolverSettings(settings);

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.friction = 0.5f;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    std::vector<BodyHandle> boxes;
    for (int32_t index = 0; index < 10; ++index) {
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
    TowerOutcome outcome;
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
        if (step < stepCount - kMeasureWindow) {
            continue;
        }
        for (const Contact& contact : world.GetContacts()) {
            outcome.maxPenetration = std::max(outcome.maxPenetration, contact.manifold.penetration);
        }
        for (const BodyHandle box : boxes) {
            outcome.maxSpeed = std::max(outcome.maxSpeed, p2::Length(world.GetLinearVelocity(box)));
            outcome.maxAbsX = std::max(outcome.maxAbsX, std::fabs(world.GetPosition(box).x));
            outcome.maxAbsAngle = std::max(outcome.maxAbsAngle, std::fabs(world.GetAngle(box)));
        }
    }
    outcome.finalSpacingError = std::fabs(world.GetPosition(boxes.back()).y - 9.5f);
    return outcome;
}

} // namespace

// ---------------------------------------------------------------------------
// D4a: warm start cancels the inherited load in one pass; the residual is
// (1 - warmStartFactor) * g*dt for the canonical stack frame.
// ---------------------------------------------------------------------------

TEST(SolverWarmStart, WarmStartRemovesTheFirstIterationResidual) {
    // Hand derivation (levers zero, m = I = 1, contacts solved ground first then
    // middle; x = g*dt = 0.16666667 is the post-gravity fall speed):
    //   contact0 (ground -> body1): K_n = invMassB = 1 -> normalMass = 1
    //   contact1 (body1 -> body2):  K_n = 1 + 1 = 2     -> normalMass = 0.5
    //   inherited impulses: J0 = 2x (supports both bodies), J1 = x (top body).
    //   Seeds = f * inherited. After WarmStart:
    //     body1: -x + 2fx - fx = (f - 1)x;  body2: -x + fx = (f - 1)x.
    //   Pass contact0: vn = (f-1)x -> lambda = (1-f)x -> body1 picks up (1-f)x
    //     and is exactly at rest again.
    //   Pass contact1: vn = v2 - v1 = (f-1)x -> lambda = 0.5(1-f)x -> body2 gets
    //     0.5(1-f)x, body1 loses it -> final residual |v1| = 0.5(1-f)x.
    //   So the middle contact splits the residual two ways; f = 1 is exact.
    const float fallSpeed = kGravity * kDt;
    const float factors[] = {0.0f, 0.5f, p2::kWarmStartFactor, 1.0f};
    float previousResidual = 0.5f * fallSpeed;
    for (const float factor : factors) {
        const StackSolveResult result = SolveStackFrame(factor, true);
        const float expected = 0.5f * (1.0f - factor) * fallSpeed;
        std::printf("[solver5] warm-start stack factor=%.2f residual1=%.8f expected=%.8f body1Vy=%+.8f "
                    "body2Vy=%+.8f\n",
                    static_cast<double>(factor), static_cast<double>(result.residualAfterOneIteration),
                    static_cast<double>(expected), static_cast<double>(result.body1Velocity),
                    static_cast<double>(result.body2Velocity));
        EXPECT_NEAR(result.residualAfterOneIteration, expected, 1.0e-5f) << "factor=" << factor;
        // More trust in last frame's solution can only reduce this residual.
        EXPECT_LE(result.residualAfterOneIteration, previousResidual + kTolerance) << "factor=" << factor;
        previousResidual = result.residualAfterOneIteration;
    }

    // factor = 1 (the shipped default) solves the frame with zero iterations of
    // error: every body is exactly at rest after a single Gauss-Seidel pass.
    const StackSolveResult fullTrust = SolveStackFrame(1.0f, true);
    EXPECT_FLOAT_EQ(fullTrust.residualAfterOneIteration, 0.0f);
    EXPECT_FLOAT_EQ(fullTrust.body1Velocity, 0.0f);
    EXPECT_FLOAT_EQ(fullTrust.body2Velocity, 0.0f);

    // Disabled warm starting == factor 0: the residual is half the gravity
    // increment (see derivation above).
    const StackSolveResult cold = SolveStackFrame(0.0f, false);
    std::printf("[solver5] warm-start stack cold residual1=%.8f\n",
                static_cast<double>(cold.residualAfterOneIteration));
    EXPECT_NEAR(cold.residualAfterOneIteration, 0.5f * fallSpeed, 1.0e-6f);
}

// ---------------------------------------------------------------------------
// D4b: factor 0 and "warm starting disabled" must be indistinguishable, down to
// the last bit, on a multi-contact solve that exercises normal + friction.
// ---------------------------------------------------------------------------

namespace {

struct SolverRun {
    std::vector<Body> bodies;
    std::vector<Contact> contacts;
    std::vector<p2::ContactConstraintPoint> constraints; // solved values
};

SolverRun RunMixedSolve(const SolverSettings& settings) {
    SolverRun run;
    run.bodies.push_back(MakeStatic(0, Vec2{0.0f, 0.0f}));
    run.bodies.push_back(MakeDynamic(1, Vec2{0.0f, 0.0f}, Vec2{0.7f, -2.0f}, 0.3f));
    run.bodies.push_back(MakeDynamic(2, Vec2{0.0f, 0.0f}, Vec2{-0.4f, -1.0f}, -0.5f));
    run.bodies.push_back(MakeDynamic(3, Vec2{0.0f, 0.0f}, Vec2{0.2f, -0.5f}, 0.1f));

    Contact first = MakeCentralContact(run.bodies[0].handle, run.bodies[1].handle, Vec2{0.0f, 1.0f}, 0.6f);
    first.isPersistent = true;
    first.manifold.points[0].normalImpulse = 3.0f;
    first.manifold.points[0].tangentImpulse = 0.4f;
    Contact second = MakeCentralContact(run.bodies[1].handle, run.bodies[2].handle, Vec2{0.0f, 1.0f}, 0.6f);
    second.isPersistent = true;
    second.manifold.points[0].normalImpulse = 1.5f;
    second.manifold.points[0].tangentImpulse = -0.2f;
    Contact third = MakeCentralContact(run.bodies[1].handle, run.bodies[3].handle, Vec2{1.0f, 0.0f}, 0.6f);
    third.isPersistent = true;
    third.manifold.points[0].normalImpulse = 0.8f;
    third.manifold.points[0].tangentImpulse = 0.1f;
    run.contacts = {first, second, third};

    ContactSolver solver;
    solver.Prepare(run.bodies, run.contacts, settings);
    solver.WarmStart(run.bodies, run.contacts, settings);
    solver.SolveVelocity(run.bodies, run.contacts, 4, settings);
    run.constraints = solver.GetConstraints();
    return run;
}

} // namespace

TEST(SolverWarmStart, FactorZeroIsBitIdenticalToDisabledWarmStarting) {
    SolverSettings zeroFactor; // useWarmStarting = true, factor = 0
    zeroFactor.warmStartFactor = 0.0f;
    SolverSettings disabled;
    disabled.useWarmStarting = false;

    const SolverRun zeroFactorRun = RunMixedSolve(zeroFactor);
    const SolverRun disabledRun = RunMixedSolve(disabled);

    ASSERT_EQ(zeroFactorRun.bodies.size(), disabledRun.bodies.size());
    ASSERT_EQ(zeroFactorRun.constraints.size(), disabledRun.constraints.size());
    ASSERT_EQ(zeroFactorRun.contacts.size(), disabledRun.contacts.size());

    for (size_t i = 0; i < zeroFactorRun.bodies.size(); ++i) {
        EXPECT_EQ(FloatBits(zeroFactorRun.bodies[i].linearVelocity.x),
                  FloatBits(disabledRun.bodies[i].linearVelocity.x))
            << "body " << i << " vx";
        EXPECT_EQ(FloatBits(zeroFactorRun.bodies[i].linearVelocity.y),
                  FloatBits(disabledRun.bodies[i].linearVelocity.y))
            << "body " << i << " vy";
        EXPECT_EQ(FloatBits(zeroFactorRun.bodies[i].angularVelocity),
                  FloatBits(disabledRun.bodies[i].angularVelocity))
            << "body " << i << " w";
    }
    for (size_t i = 0; i < zeroFactorRun.contacts.size(); ++i) {
        EXPECT_EQ(FloatBits(zeroFactorRun.contacts[i].manifold.points[0].normalImpulse),
                  FloatBits(disabledRun.contacts[i].manifold.points[0].normalImpulse))
            << "contact " << i << " accN";
        EXPECT_EQ(FloatBits(zeroFactorRun.contacts[i].manifold.points[0].tangentImpulse),
                  FloatBits(disabledRun.contacts[i].manifold.points[0].tangentImpulse))
            << "contact " << i << " accT";
    }

    // Sanity: warm starting ON with full trust must actually change the result,
    // otherwise the bitwise comparison above could pass vacuously.
    SolverSettings fullTrust;
    fullTrust.warmStartFactor = 1.0f;
    const SolverRun warmRun = RunMixedSolve(fullTrust);
    bool differs = false;
    for (size_t i = 0; i < warmRun.bodies.size(); ++i) {
        differs = differs || FloatBits(warmRun.bodies[i].linearVelocity.x) !=
                                 FloatBits(zeroFactorRun.bodies[i].linearVelocity.x);
        differs = differs || FloatBits(warmRun.bodies[i].linearVelocity.y) !=
                                 FloatBits(zeroFactorRun.bodies[i].linearVelocity.y);
        differs = differs || FloatBits(warmRun.bodies[i].angularVelocity) !=
                                 FloatBits(zeroFactorRun.bodies[i].angularVelocity);
    }
    EXPECT_TRUE(differs);
    std::printf("[solver5] warm-factor-0 vs disabled: bit-identical; factor=1 differs=%d\n",
                static_cast<int>(differs));
}

// ---------------------------------------------------------------------------
// D4c: restitution bias must be captured before warm starting.
// ---------------------------------------------------------------------------

TEST(SolverWarmStart, RestitutionBiasIsCapturedBeforeWarmStarting) {
    // A dynamic (m = 1) at v = (5, 0) towards static B; normal = (1, 0); e = 1
    // and the approach speed 5 exceeds the restitution threshold. The inherited
    // normal impulse is 10, so WarmStart changes vA from +5 to -5 AFTER Prepare.
    // Prepare must have captured vn0 = -5 -> bias = e*vn0 = -5. If the bias were
    // recomputed after warm start the pair would look separating and the bias
    // would drop to 0 - the classic "resting contact re-bounces forever" bug.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{5.0f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    Contact contact =
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.0f, /*restitution=*/1.0f);
    contact.isPersistent = true;
    contact.manifold.points[0].normalImpulse = 10.0f;
    std::vector<Contact> contacts{contact};

    SolverSettings settings;
    settings.warmStartFactor = 1.0f;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_NEAR(solver.GetConstraints()[0].bias, -5.0f, kTolerance);

    solver.WarmStart(bodies, contacts, settings);
    // Warm start moved A away from B: vA = 5 - 10*invMass(1) = -5 (separating).
    EXPECT_NEAR(bodies[0].linearVelocity.x, -5.0f, kTolerance);
    // ... yet the bias still describes the original approach speed.
    EXPECT_NEAR(solver.GetConstraints()[0].bias, -5.0f, kTolerance);
    std::printf(
        "[solver5] restitution bias after warm start: %.4f (expected -5.0000, pre-warm-start vn0=-5)\n",
        static_cast<double>(solver.GetConstraints()[0].bias));
}

// ---------------------------------------------------------------------------
// D4d: the shipped default (kWarmStartFactor = 1.0) keeps the Week 5 milestone
// tower standing; the cold baseline is the Week 4 collapse. (The full
// 0/0.5/0.85/1.0 World-level scan lives in core-dev's World stability test; the
// measured collapse steps are documented in Settings.h.)
// ---------------------------------------------------------------------------

TEST(SolverWarmStart, DefaultWarmStartingKeepsTheTenBoxTowerStable) {
    // 10 unit boxes, 10 simulated seconds, default 8 velocity / 3 position
    // iterations. Warm starting is the mechanism that makes the tower stand:
    // the cold run (disabled) reproduces the Week 4 collapse.
    constexpr int32_t kSteps = 600;
    ASSERT_FLOAT_EQ(p2::kWarmStartFactor, 1.0f) << "the shipped default must be full inheritance";
    ASSERT_FLOAT_EQ(SolverSettings{}.warmStartFactor, 1.0f);

    const TowerOutcome cold = RunTenBoxTower(0.0f, false, kSteps);
    std::printf(
        "[solver5] tower10 warm=OFF  pen=%.5f maxSpeed=%.5f max|x|=%.4f max|angle|=%.4f topSpacingErr=%.4f\n",
        static_cast<double>(cold.maxPenetration), static_cast<double>(cold.maxSpeed),
        static_cast<double>(cold.maxAbsX), static_cast<double>(cold.maxAbsAngle),
        static_cast<double>(cold.finalSpacingError));

    const TowerOutcome warm = RunTenBoxTower(SolverSettings{}.warmStartFactor, true, kSteps);
    std::printf(
        "[solver5] tower10 warm=ON   pen=%.5f maxSpeed=%.5f max|x|=%.4f max|angle|=%.4f topSpacingErr=%.4f\n",
        static_cast<double>(warm.maxPenetration), static_cast<double>(warm.maxSpeed),
        static_cast<double>(warm.maxAbsX), static_cast<double>(warm.maxAbsAngle),
        static_cast<double>(warm.finalSpacingError));

    // Milestone acceptance on the default settings: overlap below the 0.01 m
    // limit, tower assembled, top box still near y = 9.5.
    EXPECT_LT(warm.maxPenetration, 0.01f);
    EXPECT_LT(warm.finalSpacingError, 0.05f) << "the top box must stay near its stacked height";
    EXPECT_LT(warm.maxAbsX, 0.1f) << "the tower must not scatter sideways";

    // The cold baseline keeps the Week 4 behaviour: the tower comes apart.
    EXPECT_GT(cold.finalSpacingError, 1.0f);
    EXPECT_GT(cold.maxAbsX, 0.5f);
    // Warm starting is measurably better on the same run.
    EXPECT_LT(warm.maxPenetration, cold.maxPenetration);
    EXPECT_LT(warm.finalSpacingError, cold.finalSpacingError);
}
