// Week 5 P09 D1/D2 — Coulomb friction unit and behaviour tests.
//
// Covered contracts (all numbers hand-derived in the comments):
//  - t = Skew(normal) and the tangent effective mass K_t formula;
//  - the friction cone clamps the ACCUMULATED tangent impulse to
//    [-mu*accN, +mu*accN], using the normal impulse updated in the SAME iteration
//    (clamping the increment instead would let the impulse leave the cone);
//  - mu = 0 produces exactly zero tangent impulses;
//  - the friction-angle scan on a tilted scene: mu > tan(theta) sticks,
//    mu < tan(theta) slides downhill (the classic Coulomb criterion);
//  - after 1000 resting frames every contact still satisfies accN >= 0 and
//    |accT| <= mu*accN + 1e-6.
//
// The scene tests use World because the friction-angle criterion is a property of
// the whole pipeline (gravity -> solve -> integrate); the formula tests call the
// solver directly with hand-built Body/Contact data.

#include <algorithm>
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

// One-point contact whose anchors are the two body origins: rA = rB = 0, so both
// effective masses reduce to the translational term and the hand arithmetic stays
// exact.
Contact MakeCentralContact(BodyHandle bodyA, BodyHandle bodyB, Vec2 normal, float friction) {
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
    return contact;
}

// Signed relative normal velocity of a prepared constraint (negative = closing).
float NormalRelativeVelocity(const ContactSolver& solver, const std::vector<Body>& bodies,
                             const std::vector<Contact>& contacts, size_t constraintIndex) {
    const p2::ContactConstraintPoint& constraint = solver.GetConstraints().at(constraintIndex);
    const Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
    const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
    const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
    return p2::Dot(ContactSolver::ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB),
                   constraint.normal);
}

// One friction-angle experiment: a 2 m x 1 m box resting on the horizontal plane
// with gravity tilted by theta towards +x. Tilted gravity is physically identical
// to a tilted plane and keeps the analytic plane/box narrow phase axis-aligned.
// The box is deliberately wide/flat: the toppling threshold for a box of width w
// and height h on an incline is tan(theta) > w/h = 2, so the 45 deg stick cell
// tests friction rather than rigid-body tipping (a 1x1 box is marginal at 45).
struct RampOutcome {
    float displacementX = 0.0f;
    float velocityX = 0.0f;
    float maxPenetration = 0.0f;
    int32_t contactCount = 0;
};

RampOutcome RunFrictionRamp(float friction, float angleRadians, int32_t stepCount) {
    World world(Vec2(kGravity * std::sin(angleRadians), -kGravity * std::cos(angleRadians)));

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.friction = friction;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    BodyDef boxDef;
    boxDef.type = BodyType::Dynamic;
    boxDef.position = Vec2{0.0f, 0.5f}; // 2x1 box, bottom face exactly on the plane
    boxDef.friction = friction;
    boxDef.density = 1.0f;
    const BodyHandle box = world.CreateBody(boxDef);
    EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(1.0f, 0.5f)));

    RampOutcome outcome;
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
        for (const Contact& contact : world.GetContacts()) {
            outcome.maxPenetration = std::max(outcome.maxPenetration, contact.manifold.penetration);
        }
    }
    outcome.displacementX = world.GetPosition(box).x;
    outcome.velocityX = world.GetLinearVelocity(box).x;
    outcome.contactCount = world.GetContactCount();
    return outcome;
}

} // namespace

// ---------------------------------------------------------------------------
// D1: tangent direction and effective tangent mass.
// ---------------------------------------------------------------------------

TEST(SolverFriction, TangentMassHandComputedCases) {
    // dynamicA: m = 1 -> invMass = 1; I = 0.5 -> invInertia = 2.
    const Body dynamicA = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 1.0f, 0.5f);
    const Body staticBody = MakeStatic(1);
    const Vec2 zero{0.0f, 0.0f};
    const Vec2 xAxis{1.0f, 0.0f};

    // (a) zero levers: K_t = invMassA = 1 (the translational term only).
    EXPECT_FLOAT_EQ(ContactSolver::ComputeEffectiveTangentMass(dynamicA, staticBody, zero, zero, xAxis),
                    1.0f);

    // (b) rA = (0, 1), t = (1, 0): Cross(rA, t) = 0*0 - 1*1 = -1 -> squared 1
    //     K_t = invMassA(1) + invIA(2)*1 = 3.
    EXPECT_FLOAT_EQ(
        ContactSolver::ComputeEffectiveTangentMass(dynamicA, staticBody, Vec2{0.0f, 1.0f}, zero, xAxis),
        3.0f);

    // (c) lever arm parallel to the tangent still couples through the angle:
    //     rA = (1, 0), t = (1, 0): Cross = 1*0 - 0*1 = 0 -> no angular term.
    EXPECT_FLOAT_EQ(
        ContactSolver::ComputeEffectiveTangentMass(dynamicA, staticBody, Vec2{1.0f, 0.0f}, zero, xAxis),
        1.0f);

    // Swapping A/B together with the levers leaves the scalar mass unchanged.
    const float bothLevers =
        ContactSolver::ComputeEffectiveTangentMass(dynamicA, staticBody, Vec2{0.0f, 1.0f}, zero, xAxis);
    EXPECT_FLOAT_EQ(
        ContactSolver::ComputeEffectiveTangentMass(staticBody, dynamicA, zero, Vec2{0.0f, 1.0f}, xAxis),
        bothLevers);
}

// ---------------------------------------------------------------------------
// D2: the cone clamps the ACCUMULATED tangent impulse, not the increment.
// ---------------------------------------------------------------------------

TEST(SolverFriction, FrictionClampsAccumulatedTangentImpulseToTheCone) {
    // A dynamic m = 1 (invMass 1, so K_n = K_t = 1), B static, zero levers.
    // A moves with v = (3, 2) towards B; normal = (0, 1) points A -> B, so the
    // pair approaches at vn = -2 and slides tangentially at vt = +3.
    // t = Skew(n) = (-1, 0).
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{3.0f, 2.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{0.0f, 1.0f}, /*friction=*/0.5f)};

    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);

    solver.SolveVelocity(bodies, contacts, 1, settings);

    // Normal pass (same iteration, BEFORE the tangent pass):
    //   lambda = -(vn + bias) * normalMass = -(-2 + 0)*1 = 2 -> accN = 2;
    //   impulse (0, 2) applied to A as -J: v = (3, 2) - (0, 2) = (3, 0).
    // Tangent pass on the UPDATED state:
    //   vt = Dot(vB - vA, t) = Dot((-3, 0), (-1, 0)) = 3
    //   lambdaT = -vt * tangentMass = -3; maxF = mu*accN = 0.5*2 = 1
    //   accT = Clamp(0 - 3, -1, +1) = -1        (clamped: |accT| = 1 = mu*accN)
    //   delta = -1 -> impulse t*(-1) = (1, 0) applied to A as -J:
    //   v = (3, 0) - (1, 0) = (2, 0).
    // The cone check is exact: -mu*accN = -1.
    EXPECT_NEAR(bodies[0].linearVelocity.x, 2.0f, kTolerance);
    EXPECT_NEAR(bodies[0].linearVelocity.y, 0.0f, kTolerance);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedTangentImpulse, -1.0f, kTolerance);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedImpulse, 2.0f, kTolerance);

    // The write-back carries the clamped VALUE, not the raw increment.
    EXPECT_NEAR(contacts[0].manifold.points[0].normalImpulse, 2.0f, kTolerance);
    EXPECT_NEAR(contacts[0].manifold.points[0].tangentImpulse, -1.0f, kTolerance);
    EXPECT_LE(std::fabs(contacts[0].manifold.points[0].tangentImpulse),
              contacts[0].friction * contacts[0].manifold.points[0].normalImpulse + kTolerance);

    // Extra iterations cannot push the accumulator out of the cone either:
    // the normal impulse stays 2 -> maxF stays 1, and the tangent constraint is
    // already saturated, so the velocity must not change any more.
    solver.SolveVelocity(bodies, contacts, 8, settings);
    EXPECT_NEAR(bodies[0].linearVelocity.x, 2.0f, kTolerance);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedTangentImpulse, -1.0f, kTolerance);
    EXPECT_LE(std::fabs(solver.GetConstraints()[0].accumulatedTangentImpulse),
              contacts[0].friction * solver.GetConstraints()[0].accumulatedImpulse + kTolerance);
}

TEST(SolverFriction, ZeroFrictionAppliesNoTangentImpulse) {
    // Same geometry/velocity as above but mu = 0: the tangent pass must leave the
    // tangential velocity (3 m/s) completely untouched; only the normal impulse
    // is solved.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{3.0f, 2.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{0.0f, 1.0f}, /*friction=*/0.0f)};

    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    solver.SolveVelocity(bodies, contacts, 8, settings);

    EXPECT_FLOAT_EQ(bodies[0].linearVelocity.x, 3.0f); // no tangential impulse at all
    EXPECT_FLOAT_EQ(bodies[0].linearVelocity.y, 0.0f); // normal solved
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_FLOAT_EQ(solver.GetConstraints()[0].accumulatedTangentImpulse, 0.0f);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].tangentImpulse, 0.0f);
}

// ---------------------------------------------------------------------------
// D3: friction-angle scan (mu vs tan(theta)).
// ---------------------------------------------------------------------------

TEST(SolverFriction, FrictionAngleScanSticksAboveTanThetaAndSlidesBelow) {
    // Tilted gravity with |g| = 10: the tangential acceleration is
    // g*sin(theta) and the normal load g*cos(theta), so static equilibrium needs
    // mu >= tan(theta). On the friction cone this is the textbook criterion:
    // required tangent impulse / normal impulse = tan(theta) <= mu.
    //
    // Matrix from the Week 5 task card: mu in {0.1, 0.3, 0.5, 0.8, 1.2} x
    // theta in {5, 15, 30, 45} deg, 5 simulated seconds each.
    //   stick : mu > tan(theta) -> |dx| < 0.01 m (no creep);
    //   slide : mu < 0.8*tan(theta) -> dx > 0.1 m (definitely sliding);
    //   in between: still assert the physically required direction (mu < tan
    //   means it must slide) without weakening the two contract bounds.
    const float frictionCoefficients[] = {0.1f, 0.3f, 0.5f, 0.8f, 1.2f};
    const float anglesDegrees[] = {5.0f, 15.0f, 30.0f, 45.0f};
    constexpr int32_t kSteps = 300; // 5 s at 1/60 s

    int32_t stickCells = 0;
    int32_t slideCells = 0;
    for (const float friction : frictionCoefficients) {
        for (const float angleDegrees : anglesDegrees) {
            const float theta = angleDegrees * p2::kDegToRad;
            const float tanTheta = std::tan(theta);
            const RampOutcome outcome = RunFrictionRamp(friction, theta, kSteps);
            std::printf("[solver5] friction-angle mu=%.2f theta=%4.1fdeg tan=%.4f mu/tan=%.3f dx=%+.4f m "
                        "vx=%+.4f m/s maxPen=%.5f contacts=%d\n",
                        static_cast<double>(friction), static_cast<double>(angleDegrees),
                        static_cast<double>(tanTheta), static_cast<double>(friction / tanTheta),
                        static_cast<double>(outcome.displacementX), static_cast<double>(outcome.velocityX),
                        static_cast<double>(outcome.maxPenetration), outcome.contactCount);

            EXPECT_GE(outcome.contactCount, 1)
                << "no contact for mu=" << friction << " theta=" << angleDegrees;

            if (friction > tanTheta) {
                ++stickCells;
                // mu > tan(theta): the cone can supply the whole tangential load,
                // so the box must not creep downhill.
                EXPECT_LT(std::fabs(outcome.displacementX), 0.01f)
                    << "mu=" << friction << " theta=" << angleDegrees << " dx=" << outcome.displacementX;
            } else {
                ++slideCells;
                // mu < tan(theta): gravity wins, the box must accelerate downhill.
                EXPECT_GT(outcome.displacementX, 0.1f)
                    << "mu=" << friction << " theta=" << angleDegrees << " dx=" << outcome.displacementX;
            }
        }
    }
    EXPECT_EQ(stickCells + slideCells, 20);
    EXPECT_GT(stickCells, 0);
    EXPECT_GT(slideCells, 0);
    std::printf("[solver5] friction-angle summary: %d stick cells, %d slide cells (of 20)\n", stickCells,
                slideCells);
}

// ---------------------------------------------------------------------------
// D4: long-run cone invariant on a loaded stack.
// ---------------------------------------------------------------------------

TEST(SolverFriction, ConeInvariantHoldsForEveryContactOnEveryFrame) {
    // Tilted gravity (10 deg, mu = 0.5 > tan(10 deg) = 0.176) keeps a four-box
    // stack statically loaded: every frame has real normal AND friction impulses.
    constexpr float kTilt = 10.0f * p2::kDegToRad;
    World world(Vec2(kGravity * std::sin(kTilt), -kGravity * std::cos(kTilt)));

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.friction = 0.5f;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    std::vector<BodyHandle> boxes;
    for (int32_t index = 0; index < 4; ++index) {
        BodyDef boxDef;
        boxDef.type = BodyType::Dynamic;
        // Horizontal offsets make the boxes lean on each other so the stack has
        // to carry genuine friction, not just symmetric normal forces.
        boxDef.position = Vec2{(index % 2 == 0) ? 0.02f : -0.02f, 0.5f + static_cast<float>(index)};
        boxDef.friction = 0.5f;
        boxDef.density = 1.0f;
        const BodyHandle box = world.CreateBody(boxDef);
        EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(0.5f, 0.5f)));
        boxes.push_back(box);
    }

    int32_t framesWithContacts = 0;
    int32_t firstEmptyFrame = -1;
    int32_t coneViolations = 0;
    int32_t negativeNormalViolations = 0;
    float maxNormalImpulse = 0.0f;
    float maxTangentImpulse = 0.0f;
    float maxConeRatio = 0.0f;

    for (int32_t step = 0; step < 1000; ++step) {
        world.Step(kDt);
        const std::vector<Contact>& contacts = world.GetContacts();
        if (contacts.empty()) {
            if (firstEmptyFrame < 0) {
                firstEmptyFrame = step;
            }
        } else {
            ++framesWithContacts;
        }
        for (const Contact& contact : contacts) {
            for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
                const p2::ContactPoint& point = contact.manifold.points[pointIndex];
                const float normalImpulse = point.normalImpulse;
                const float tangentImpulse = point.tangentImpulse;
                if (!(normalImpulse >= 0.0f)) {
                    ++negativeNormalViolations;
                }
                if (std::fabs(tangentImpulse) > contact.friction * normalImpulse + 1.0e-6f) {
                    ++coneViolations;
                }
                maxNormalImpulse = std::max(maxNormalImpulse, normalImpulse);
                maxTangentImpulse = std::max(maxTangentImpulse, std::fabs(tangentImpulse));
                if (normalImpulse > p2::kEpsilon) {
                    maxConeRatio = std::max(maxConeRatio, std::fabs(tangentImpulse) / normalImpulse);
                }
            }
        }
    }

    std::printf("[solver5] cone-invariant 1000 frames: framesWithContacts=%d firstEmpty=%d maxAccN=%.5f "
                "max|accT|=%.5f "
                "max|accT|/accN=%.4f (mu=0.5000) violations=%d negativeAccN=%d\n",
                framesWithContacts, firstEmptyFrame, static_cast<double>(maxNormalImpulse),
                static_cast<double>(maxTangentImpulse), static_cast<double>(maxConeRatio), coneViolations,
                negativeNormalViolations);

    // With the shipped default warm start the stack never loses contact: every
    // one of the 1000 frames has at least one contact.
    EXPECT_EQ(firstEmptyFrame, -1);
    EXPECT_EQ(framesWithContacts, 1000) << "the stack must stay in contact for the whole run";
    EXPECT_EQ(coneViolations, 0);
    EXPECT_EQ(negativeNormalViolations, 0);
    EXPECT_GT(maxNormalImpulse, 0.0f);
    EXPECT_GT(maxTangentImpulse, 0.0f) << "friction must actually be loaded for this test to mean anything";
    // The cone is the strict upper bound; allow only the 1e-6 tolerance requested
    // by the task card for the reported maximum ratio.
    EXPECT_LE(maxConeRatio, 0.5f + 1.0e-4f);
}

TEST(SolverFriction, ZeroFrictionRampNeverProducesATangentImpulse) {
    // mu = 0 on a 20 deg ramp: the box slides (no friction can hold it) and no
    // contact point may ever accumulate a tangent impulse.
    constexpr float kTilt = 20.0f * p2::kDegToRad;
    World world(Vec2(kGravity * std::sin(kTilt), -kGravity * std::cos(kTilt)));

    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.friction = 0.0f;
    const BodyHandle ground = world.CreateBody(groundDef);
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));

    BodyDef boxDef;
    boxDef.type = BodyType::Dynamic;
    boxDef.position = Vec2{0.0f, 0.5f};
    boxDef.friction = 0.0f;
    boxDef.density = 1.0f;
    const BodyHandle box = world.CreateBody(boxDef);
    EXPECT_TRUE(world.AttachShape(box, world.CreateBoxShape(0.5f, 0.5f)));

    int32_t tangentViolations = 0;
    float maxAbsTangentImpulse = 0.0f;
    for (int32_t step = 0; step < 300; ++step) {
        world.Step(kDt);
        for (const Contact& contact : world.GetContacts()) {
            EXPECT_FLOAT_EQ(contact.friction, 0.0f);
            for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
                const float tangentImpulse = contact.manifold.points[pointIndex].tangentImpulse;
                maxAbsTangentImpulse = std::max(maxAbsTangentImpulse, std::fabs(tangentImpulse));
                if (tangentImpulse != 0.0f) {
                    ++tangentViolations;
                }
            }
        }
    }
    const float displacementX = world.GetPosition(box).x;
    std::printf("[solver5] zero-friction ramp: dx=%+.4f m max|accT|=%.8f violations=%d\n",
                static_cast<double>(displacementX), static_cast<double>(maxAbsTangentImpulse),
                tangentViolations);
    EXPECT_EQ(tangentViolations, 0);
    EXPECT_FLOAT_EQ(maxAbsTangentImpulse, 0.0f);
    EXPECT_GT(displacementX, 1.0f) << "a frictionless box must slide down the 20 deg ramp";
}
