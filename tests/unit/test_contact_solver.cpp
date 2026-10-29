// Unit tests for p2::ContactSolver — the Sequential Impulse velocity layer
// (design document Week 3 P08, extended in Week 5 P09: friction, warm starting).
//
// The solver is tested directly on hand-built Body/Contact data, not through
// World: the contracts under test are the impulse formulas (v = J/m,
// w = Cross(r,J)/I), the effective normal/tangent masses, the accumulated-impulse
// clamp, the restitution bias, the warm-start seeds and the iteration
// convergence. Every expected number is derived in the comment next to the
// assertion. The Week 5 friction-cone, warm-start and split-impulse behaviour has
// its own files (test_solver_*.cpp); this file keeps the velocity-layer basics.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Collision/Contact.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"
#include "physics2d/Solver/ContactSolver.h"

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::ContactConstraintPoint;
using p2::ContactSolver;
using p2::SolverSettings;
using p2::Vec2;

// Impulse results are products/quotients of exactly representable inputs; 1e-6
// covers the float rounding with a wide margin (relative error ~1e-7).
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
    // The solver indexes bodies by handle.index; tests build the vector in that
    // same order so a handle index is also the vector position.
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

// Contact whose anchors are the two body origins (localPoint == 0), so the
// lever arms rA/rB are exactly zero and the formulas reduce to the linear case.
Contact MakeCentralContact(BodyHandle bodyA, BodyHandle bodyB, Vec2 normal, float restitution = 0.0f,
                           int32_t pointCount = 1) {
    Contact contact;
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    contact.manifold.normal = normal;
    contact.manifold.penetration = 0.0f;
    contact.manifold.pointCount = pointCount;
    for (int32_t i = 0; i < pointCount; ++i) {
        contact.manifold.points[i].localPointA = Vec2{0.0f, 0.0f};
        contact.manifold.points[i].localPointB = Vec2{0.0f, 0.0f};
        contact.manifold.points[i].id.type = p2::kContactTypeCircleCircle;
    }
    contact.restitution = restitution;
    return contact;
}

// Relative normal velocity of a prepared constraint after the solve: positive
// means separating, negative means the pair is still closing.
float NormalRelativeVelocity(const ContactSolver& solver, const std::vector<Body>& bodies,
                             const std::vector<Contact>& contacts, size_t constraintIndex) {
    const ContactConstraintPoint& constraint = solver.GetConstraints().at(constraintIndex);
    const Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
    const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
    const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
    const Vec2 relativeVelocity =
        ContactSolver::ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
    return p2::Dot(relativeVelocity, constraint.normal);
}

} // namespace

// ---------------------------------------------------------------------------
// D1: impulse application — the solver's output must match v = J/m and
// w = Cross(r, J)/I exactly, on both the linear and the angular side.
// ---------------------------------------------------------------------------

TEST(ContactSolver, BodyImpulseResponseMatchesAnalyticFormula) {
    // Case 1: m = 2, I = 3, J = (4, 0) applied at world (0, 2) on a body at the
    // origin, so r = (0, 2):
    //   v = J/m = (2, 0)
    //   w = Cross(r, J)/I = (0*0 - 2*4)/3 = -8/3 = -2.6666667
    Body body = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 2.0f, 3.0f);
    body.ApplyLinearImpulse(Vec2{4.0f, 0.0f}, Vec2{0.0f, 2.0f});
    EXPECT_NEAR(body.linearVelocity.x, 2.0f, kTolerance);
    EXPECT_NEAR(body.linearVelocity.y, 0.0f, kTolerance);
    EXPECT_NEAR(body.angularVelocity, -8.0f / 3.0f, kTolerance);

    // Case 2: m = 4, I = 2, J = (0, 6) applied at world (3, 0), body at origin:
    //   v = (0, 1.5)
    //   w = Cross((3,0), (0,6))/I = (3*6 - 0*0)/2 = 9
    Body body2 = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 4.0f, 2.0f);
    body2.ApplyLinearImpulse(Vec2{0.0f, 6.0f}, Vec2{3.0f, 0.0f});
    EXPECT_NEAR(body2.linearVelocity.x, 0.0f, kTolerance);
    EXPECT_NEAR(body2.linearVelocity.y, 1.5f, kTolerance);
    EXPECT_NEAR(body2.angularVelocity, 9.0f, kTolerance);

    // Case 3: body at (1, 1), m = 1, I = 0.5, J = (-2, 0) at world (1, 3):
    //   r = (0, 2); v = (-2, 0)
    //   w = Cross((0,2), (-2,0))/0.5 = (0*0 - 2*(-2))/0.5 = 8
    Body body3 = MakeDynamic(0, Vec2{1.0f, 1.0f}, Vec2{0.0f, 0.0f}, 0.0f, 1.0f, 0.5f);
    body3.ApplyLinearImpulse(Vec2{-2.0f, 0.0f}, Vec2{1.0f, 3.0f});
    EXPECT_NEAR(body3.linearVelocity.x, -2.0f, kTolerance);
    EXPECT_NEAR(body3.linearVelocity.y, 0.0f, kTolerance);
    EXPECT_NEAR(body3.angularVelocity, 8.0f, kTolerance);

    // Infinite-mass bodies must ignore the impulse entirely.
    Body staticBody = MakeStatic(0, Vec2{0.0f, 0.0f});
    staticBody.ApplyLinearImpulse(Vec2{5.0f, 5.0f}, Vec2{1.0f, 1.0f});
    EXPECT_FLOAT_EQ(staticBody.linearVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(staticBody.linearVelocity.y, 0.0f);
    EXPECT_FLOAT_EQ(staticBody.angularVelocity, 0.0f);
}

// ---------------------------------------------------------------------------
// D2: effective normal mass K_n — three hand-computed configurations, plus the
// A/B symmetry and positivity that the solver relies on.
// ---------------------------------------------------------------------------

TEST(ContactSolver, EffectiveNormalMassHandComputedCases) {
    // dynamicA: m = 1 -> invMass = 1; I = 0.5 -> invInertia = 2.
    const Body dynamicA = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 1.0f, 0.5f);
    // dynamicB: m = 2 -> invMass = 0.5; I = 2 -> invInertia = 0.5.
    const Body dynamicB = MakeDynamic(1, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 2.0f, 2.0f);
    // equal mass/inertia partner: invMass = 1, invInertia = 1.
    const Body equalMass = MakeDynamic(2, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 1.0f, 1.0f);
    const Body staticBody = MakeStatic(3);
    const Vec2 zero{0.0f, 0.0f};
    const Vec2 xAxis{1.0f, 0.0f};
    const Vec2 yAxis{0.0f, 1.0f};

    // (a) dynamic vs static, both levers zero:
    //     K = invMassA + 0 + 0 + 0 = 1.
    EXPECT_FLOAT_EQ(ContactSolver::ComputeEffectiveNormalMass(dynamicA, staticBody, zero, zero, xAxis), 1.0f);

    // (b) two dynamic bodies with equal unit inverse mass and zero levers:
    //     K = 1 + 1 = 2.
    EXPECT_FLOAT_EQ(ContactSolver::ComputeEffectiveNormalMass(dynamicA, equalMass, zero, zero, xAxis), 2.0f);

    // (c) lever arm converts impulse into torque. rA = (0, 1), n = (1, 0):
    //     Cross(rA, n) = 0*0 - 1*1 = -1 -> squared 1
    //     K = invMassA(1) + 0 + invIA(2)*1 + 0 = 3.
    EXPECT_FLOAT_EQ(
        ContactSolver::ComputeEffectiveNormalMass(dynamicA, staticBody, Vec2{0.0f, 1.0f}, zero, xAxis), 3.0f);

    // (d) both levers, vertical normal. rA = (1, 0), n = (0, 1):
    //     Cross = 1*1 - 0*0 = 1 -> A term 2; rB = (0, 1) is parallel to n ->
    //     Cross = 0 -> B term 0.
    //     K = invMassA(1) + invMassB(0.5) + 2 + 0 = 3.5.
    const float bothLevers = ContactSolver::ComputeEffectiveNormalMass(dynamicA, dynamicB, Vec2{1.0f, 0.0f},
                                                                       Vec2{0.0f, 1.0f}, yAxis);
    EXPECT_FLOAT_EQ(bothLevers, 3.5f);
    // Swapping A/B together with the levers must not change K (it is a scalar
    // mass property of the pair, not an ordering-dependent value).
    EXPECT_FLOAT_EQ(ContactSolver::ComputeEffectiveNormalMass(dynamicB, dynamicA, Vec2{0.0f, 1.0f},
                                                              Vec2{1.0f, 0.0f}, yAxis),
                    bothLevers);

    // K > 0 for every legal pair: the solver asserts on the degenerates and a
    // positive value is what makes normalMass = 1/K safe.
    EXPECT_GT(bothLevers, 0.0f);
    EXPECT_GT(ContactSolver::ComputeEffectiveNormalMass(dynamicA, equalMass, Vec2{0.3f, 0.4f},
                                                        Vec2{-0.2f, 0.1f}, Vec2{0.0f, 1.0f}),
              0.0f);
}

TEST(ContactSolver, RelativeVelocityIncludesAngularTerms) {
    // A: v = (1, 0), w = 2, rA = (0, 1):
    //   anchor velocity = (1, 0) + Cross(2, (0,1))
    //                   = (1, 0) + (-2*1, 2*0) = (-1, 0)
    // B: static at rest with rB = 0 -> (0, 0)
    // vRel = vB - vA = (1, 0)   (positive: separating along +x).
    const Body bodyA = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, 2.0f);
    const Body staticB = MakeStatic(1);
    const Vec2 relative =
        ContactSolver::ComputeRelativeVelocity(bodyA, staticB, Vec2{0.0f, 1.0f}, Vec2{0.0f, 0.0f});
    EXPECT_NEAR(relative.x, 1.0f, kTolerance);
    EXPECT_NEAR(relative.y, 0.0f, kTolerance);

    // Both bodies spinning:
    //   A: v = 0, w = 3, rA = (2, 0) -> Cross(3, (2,0)) = (0, 6)
    //   B: v = (5, 0), w = -1, rB = (0, 2) -> Cross(-1, (0,2)) = (2, 0)
    //      anchor velocity = (7, 0)
    //   vRel = (7, 0) - (0, 6) = (7, -6).
    const Body spinningA = MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 3.0f);
    const Body spinningB = MakeDynamic(1, Vec2{0.0f, 0.0f}, Vec2{5.0f, 0.0f}, -1.0f);
    const Vec2 spinRelative =
        ContactSolver::ComputeRelativeVelocity(spinningA, spinningB, Vec2{2.0f, 0.0f}, Vec2{0.0f, 2.0f});
    EXPECT_NEAR(spinRelative.x, 7.0f, kTolerance);
    EXPECT_NEAR(spinRelative.y, -6.0f, kTolerance);
}

// ---------------------------------------------------------------------------
// Prepare: levers, normal/tangent masses, warm-start seeds (Week 5 contract).
// ---------------------------------------------------------------------------

TEST(ContactSolver, PrepareComputesLeversMassesAndSeedsWarmStartImpulses) {
    // A: dynamic at (1, 2), m = 2 (invMass 0.5), I = 2 (invInertia 0.5).
    // B: static at the origin.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{1.0f, 2.0f}, Vec2{0.0f, 0.0f}, 0.0f, 2.0f, 2.0f));
    bodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));

    Contact contact = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f});
    // localPointA (0.5, -0.25) with the body at (1, 2) and no rotation:
    //   worldA = (1.5, 1.75); rA = worldA - p = (0.5, -0.25).
    // localPointB (2, 0) on the static body at the origin: worldB = (2, 0), rB = (2, 0).
    contact.manifold.points[0].localPointA = Vec2{0.5f, -0.25f};
    contact.manifold.points[0].localPointB = Vec2{2.0f, 0.0f};
    // Week 5: World inherits last frame's accumulated impulses for a persistent
    // contact (matched by ContactID::Key), so this pair carries non-zero values.
    contact.isPersistent = true;
    contact.manifold.points[0].normalImpulse = 8.0f;
    contact.manifold.points[0].tangentImpulse = 2.0f;
    std::vector<Contact> contacts{contact};

    SolverSettings settings; // warm starting on; factor set explicitly for the hand numbers
    settings.warmStartFactor = 0.5f;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);

    ASSERT_EQ(solver.GetConstraintCount(), 1);
    const ContactConstraintPoint& constraint = solver.GetConstraints()[0];
    EXPECT_EQ(constraint.contactIndex, 0);
    EXPECT_EQ(constraint.pointIndex, 0);
    EXPECT_NEAR(constraint.rA.x, 0.5f, kTolerance);
    EXPECT_NEAR(constraint.rA.y, -0.25f, kTolerance);
    EXPECT_NEAR(constraint.rB.x, 2.0f, kTolerance);
    EXPECT_NEAR(constraint.rB.y, 0.0f, kTolerance);
    EXPECT_FLOAT_EQ(constraint.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(constraint.normal.y, 0.0f);
    // Week 5: t = Skew(n) = (-n.y, n.x) = (0, 1).
    EXPECT_FLOAT_EQ(constraint.tangent.x, 0.0f);
    EXPECT_FLOAT_EQ(constraint.tangent.y, 1.0f);

    // K_n = invMassA + invMassB + invIA*Cross(rA,n)^2 + invIB*Cross(rB,n)^2
    //   Cross((0.5, -0.25), (1, 0)) = 0.5*0 - (-0.25)*1 = 0.25 -> squared 0.0625
    //   K_n = 0.5 + 0 + 0.5*0.0625 + 0 = 0.53125 = 17/32
    //   normalMass = 1/K_n = 32/17 = 1.88235294...
    EXPECT_NEAR(constraint.normalMass, 32.0f / 17.0f, 1.0e-5f);

    // K_t with t = (0, 1):
    //   Cross((0.5, -0.25), (0, 1)) = 0.5*1 - (-0.25)*0 = 0.5 -> squared 0.25
    //   Cross((2, 0), (0, 1)) = 2*1 - 0*0 = 2 -> squared 4, but invIB = 0
    //   K_t = 0.5 + 0 + 0.5*0.25 + 0 = 0.625 = 5/8
    //   tangentMass = 1/K_t = 8/5 = 1.6
    EXPECT_NEAR(constraint.tangentMass, 1.6f, kTolerance);

    // Week 5 warm-start seeds: inherited * warmStartFactor = 8*0.5 and 2*0.5.
    // (The shipped default factor is 1.0 = full inheritance; the factor is set to
    // 0.5 here to pin the scaling arithmetic.)
    EXPECT_NEAR(constraint.accumulatedImpulse, 4.0f, kTolerance);
    EXPECT_NEAR(constraint.accumulatedTangentImpulse, 1.0f, kTolerance);
    // Prepare must NOT clobber the persistent storage: writing it back is
    // SolveVelocity's job, after the iterations have produced this frame's value.
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].normalImpulse, 8.0f);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].tangentImpulse, 2.0f);

    // A non-persistent point starts at exactly zero even if a stale impulse is
    // still sitting in the ContactPoint: World only inherits matched points, and
    // the solver must never resurrect stale data for an unmatched feature.
    std::vector<Contact> coldContacts{contact};
    coldContacts[0].isPersistent = false;
    coldContacts[0].manifold.points[0].normalImpulse = 42.0f;
    coldContacts[0].manifold.points[0].tangentImpulse = 9.0f;
    ContactSolver coldSolver;
    coldSolver.Prepare(bodies, coldContacts, settings);
    ASSERT_EQ(coldSolver.GetConstraintCount(), 1);
    EXPECT_FLOAT_EQ(coldSolver.GetConstraints()[0].accumulatedImpulse, 0.0f);
    EXPECT_FLOAT_EQ(coldSolver.GetConstraints()[0].accumulatedTangentImpulse, 0.0f);

    // useWarmStarting = false must force zero seeds even for a persistent pair
    // (this is the W4-behaviour baseline, not a second code path).
    SolverSettings noWarmSettings;
    noWarmSettings.useWarmStarting = false;
    ContactSolver noWarmSolver;
    noWarmSolver.Prepare(bodies, contacts, noWarmSettings);
    ASSERT_EQ(noWarmSolver.GetConstraintCount(), 1);
    EXPECT_FLOAT_EQ(noWarmSolver.GetConstraints()[0].accumulatedImpulse, 0.0f);
    EXPECT_FLOAT_EQ(noWarmSolver.GetConstraints()[0].accumulatedTangentImpulse, 0.0f);
}

TEST(ContactSolver, PrepareBuildsOneConstraintPerManifoldPoint) {
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{1.0f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));

    Contact empty = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.0f, 0);
    Contact twoPoint = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{0.0f, 1.0f}, 0.0f, 2);
    std::vector<Contact> contacts{empty, twoPoint};

    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);

    // The empty contact contributes nothing; the two-point contact contributes
    // two constraints whose contactIndex points at contacts[1].
    ASSERT_EQ(solver.GetConstraintCount(), 2);
    EXPECT_EQ(solver.GetConstraints()[0].contactIndex, 1);
    EXPECT_EQ(solver.GetConstraints()[0].pointIndex, 0);
    EXPECT_EQ(solver.GetConstraints()[1].contactIndex, 1);
    EXPECT_EQ(solver.GetConstraints()[1].pointIndex, 1);
}

// ---------------------------------------------------------------------------
// D5: restitution bias — only approaches faster than the threshold bounce.
// ---------------------------------------------------------------------------

TEST(ContactSolver, RestitutionBiasUsesInitialApproachAndThreshold) {
    // A: dynamic m = 1 at (-0.5, 0) moving +x at 10; B: static at (0.5, 0).
    // Normal A -> B = (1, 0); both anchors are the body origins, so K = 1.
    //   vn0 = Dot(vB - vA, n) = (0 - 10) = -10  (approaching)
    //   e = 0.8, threshold = 1 -> vn0 < -threshold, bias = e*vn0 = -8.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{10.0f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.8f)};
    SolverSettings settings; // default restitutionThreshold = kRestitutionVelocityThreshold = 1
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);

    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_NEAR(solver.GetConstraints()[0].bias, -8.0f, kTolerance);

    // One iteration drives vn to -bias = +8 (separating at e*|vn0|):
    //   K = 1, normalMass = 1; lambda = -(-10 + -8) = 18;
    //   A: 10 - 18*1 = -8 (moving away from B); vRel.n = 0 - (-8) = 8.
    solver.SolveVelocity(bodies, contacts, 1, settings);
    EXPECT_NEAR(NormalRelativeVelocity(solver, bodies, contacts, 0), 8.0f, 1.0e-5f);
    EXPECT_NEAR(bodies[0].linearVelocity.x, -8.0f, 1.0e-5f);
}

TEST(ContactSolver, RestitutionIsSuppressedBelowTheVelocityThreshold) {
    // Same geometry, approach speed 0.5 m/s < threshold 1.0: resting contacts
    // must not bounce on gravity noise, so the bias stays zero.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{0.5f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.8f)};
    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_FLOAT_EQ(solver.GetConstraints()[0].bias, 0.0f);

    // Inelastic solve: the relative normal velocity goes to 0.
    solver.SolveVelocity(bodies, contacts, 8, settings);
    EXPECT_NEAR(NormalRelativeVelocity(solver, bodies, contacts, 0), 0.0f, kTolerance);
    EXPECT_NEAR(bodies[0].linearVelocity.x, 0.0f, kTolerance);

    // Boundary: an approach of exactly the threshold is NOT above it
    // (the comparison is strict), so it is still treated as resting.
    std::vector<Body> boundaryBodies;
    boundaryBodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{1.0f, 0.0f}));
    boundaryBodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> boundaryContacts{
        MakeCentralContact(boundaryBodies[0].handle, boundaryBodies[1].handle, Vec2{1.0f, 0.0f}, 0.8f)};
    ContactSolver boundarySolver;
    boundarySolver.Prepare(boundaryBodies, boundaryContacts, settings);
    EXPECT_FLOAT_EQ(boundarySolver.GetConstraints()[0].bias, 0.0f);
}

TEST(ContactSolver, RestitutionOfZeroAndCustomThreshold) {
    // e = 0 disables restitution even for a fast approach: bias must stay 0.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{10.0f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.0f)};
    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    EXPECT_FLOAT_EQ(solver.GetConstraints()[0].bias, 0.0f);

    // A lower configured threshold makes a 0.5 m/s approach bounce:
    //   vn0 = -0.5 < -0.4 -> bias = e*vn0 = 0.5*(-0.5) = -0.25.
    std::vector<Body> slowBodies;
    slowBodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{0.5f, 0.0f}));
    slowBodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> slowContacts{
        MakeCentralContact(slowBodies[0].handle, slowBodies[1].handle, Vec2{1.0f, 0.0f}, 0.5f)};
    SolverSettings slowSettings;
    slowSettings.restitutionThreshold = 0.4f;
    ContactSolver slowSolver;
    slowSolver.Prepare(slowBodies, slowContacts, slowSettings);
    ASSERT_EQ(slowSolver.GetConstraintCount(), 1);
    EXPECT_NEAR(slowSolver.GetConstraints()[0].bias, -0.25f, kTolerance);
}

// ---------------------------------------------------------------------------
// D3: one iteration of the normal impulse on a head-on pair.
// ---------------------------------------------------------------------------

TEST(ContactSolver, ElasticHeadOnCollisionStopsAAndMovesB) {
    // Equal spheres (m = 1, I = 1), A at (-1, 0) moving at +2, B at (1, 0) at
    // rest, e = 1, zero levers. Approach speed 2 > threshold 1, so:
    //   vn0 = -2; bias = e*vn0 = -2; K = 1 + 1 = 2 -> normalMass = 0.5
    //   lambda = -(vn + bias) * normalMass = -(-2 - 2)*0.5 = 2
    //   A: 2 - 2*1 = 0   (stops)
    //   B: 0 + 2*1 = 2   (leaves with the incident speed)
    // This is P08's "A stops, B moves" result; it requires e = 1 (an e = 0
    // contact is perfectly inelastic and gives the common velocity instead).
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-1.0f, 0.0f}, Vec2{2.0f, 0.0f}));
    bodies.push_back(MakeDynamic(1, Vec2{1.0f, 0.0f}, Vec2{0.0f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 1.0f)};
    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_NEAR(solver.GetConstraints()[0].bias, -2.0f, kTolerance);

    solver.SolveVelocity(bodies, contacts, 1, settings);
    EXPECT_NEAR(bodies[0].linearVelocity.x, 0.0f, 1.0e-5f);
    EXPECT_NEAR(bodies[1].linearVelocity.x, 2.0f, 1.0e-5f);
    // Momentum: m*(2) = 2 before, m*(0) + m*(2) = 2 after.
    EXPECT_NEAR(bodies[0].mass * bodies[0].linearVelocity.x + bodies[1].mass * bodies[1].linearVelocity.x,
                2.0f, 1.0e-4f);
}

TEST(ContactSolver, InelasticHeadOnConservesMomentum) {
    // Mass ratio 1:3, e = 0, A at v = 4, B at rest, zero levers:
    //   K = invMassA + invMassB = 1 + 1/3 = 4/3 -> normalMass = 3/4
    //   lambda = -(vn + 0) * normalMass = 4 * 0.75 = 3
    //   A: 4 - 3*1 = 1; B: 0 + 3*(1/3) = 1 (perfectly inelastic common velocity)
    //   momentum: 1*4 = 4 before; 1*1 + 3*1 = 4 after.
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-1.0f, 0.0f}, Vec2{4.0f, 0.0f}, 0.0f, 1.0f, 1.0f));
    bodies.push_back(MakeDynamic(1, Vec2{1.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 3.0f, 3.0f));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.0f)};
    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    solver.SolveVelocity(bodies, contacts, 1, settings);

    EXPECT_NEAR(bodies[0].linearVelocity.x, 1.0f, 1.0e-5f);
    EXPECT_NEAR(bodies[1].linearVelocity.x, 1.0f, 1.0e-5f);
    const float momentum =
        bodies[0].mass * bodies[0].linearVelocity.x + bodies[1].mass * bodies[1].linearVelocity.x;
    EXPECT_NEAR(momentum, 4.0f, 1.0e-4f);

    // The converged accumulated impulse is 3 and is non-negative; it is also
    // written back into the persistent ContactPoint (Week 5 warm-start input).
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedImpulse, 3.0f, 1.0e-5f);
    EXPECT_NEAR(contacts[0].manifold.points[0].normalImpulse, 3.0f, 1.0e-5f);
    EXPECT_GE(contacts[0].manifold.points[0].normalImpulse, 0.0f);

    // Extra iterations must not change a single-contact solution.
    solver.SolveVelocity(bodies, contacts, 8, settings);
    EXPECT_NEAR(bodies[0].linearVelocity.x, 1.0f, 1.0e-5f);
    EXPECT_NEAR(contacts[0].manifold.points[0].normalImpulse, 3.0f, 1.0e-5f);
}

// ---------------------------------------------------------------------------
// D4: iteration convergence on a three-body chain.
// ---------------------------------------------------------------------------

namespace {

struct ChainResult {
    std::vector<float> velocities;
    float maxAbsNormalVelocity = 0.0f;
    float totalMomentum = 0.0f;
};

// Three equal unit masses in a row, A moving at +2 towards a resting B and C,
// all contacts perfectly inelastic (e = 0). The unique solution is the common
// velocity: total momentum 2 / 3 = 2/3 for every body.
ChainResult RunChain(int32_t iterations) {
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-1.0f, 0.0f}, Vec2{2.0f, 0.0f}));
    bodies.push_back(MakeDynamic(1, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}));
    bodies.push_back(MakeDynamic(2, Vec2{1.0f, 0.0f}, Vec2{0.0f, 0.0f}));

    std::vector<Contact> contacts;
    contacts.push_back(MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}));
    contacts.push_back(MakeCentralContact(bodies[1].handle, bodies[2].handle, Vec2{1.0f, 0.0f}));

    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    solver.SolveVelocity(bodies, contacts, iterations, settings);

    ChainResult result;
    for (const Body& body : bodies) {
        result.velocities.push_back(body.linearVelocity.x);
        result.totalMomentum += body.mass * body.linearVelocity.x;
    }
    for (size_t i = 0; i < solver.GetConstraints().size(); ++i) {
        result.maxAbsNormalVelocity = std::max(
            result.maxAbsNormalVelocity, std::fabs(NormalRelativeVelocity(solver, bodies, contacts, i)));
        // Accumulated impulses must always satisfy the clamp.
        EXPECT_GE(solver.GetConstraints()[i].accumulatedImpulse, 0.0f);
    }
    return result;
}

} // namespace

TEST(ContactSolver, IterationsReduceResidualNormalVelocity) {
    // Print the convergence curve (P08 D4 asks for it explicitly): the impulse
    // propagation through the middle body needs several Gauss-Seidel passes.
    const int32_t iterationCounts[] = {1, 2, 4, 8, 16, 32};
    std::vector<float> residuals;
    for (const int32_t iterations : iterationCounts) {
        const ChainResult result = RunChain(iterations);
        residuals.push_back(result.maxAbsNormalVelocity);
        std::cout << "[contact_solver] iterations=" << iterations
                  << " maxAbsVn=" << result.maxAbsNormalVelocity << "\n";
    }

    // Monotone improvement on the residual, and 8 iterations must be strictly
    // better than 1 (the P08 acceptance criterion).
    for (size_t i = 1; i < residuals.size(); ++i) {
        EXPECT_LE(residuals[i], residuals[i - 1] + kTolerance) << "iteration step " << i;
    }
    EXPECT_LT(residuals[3], residuals[0]);

    // 32 passes are enough for the inelastic chain to reach the common velocity
    // 2/3 within 1e-3, and momentum is conserved by every internal impulse.
    const ChainResult converged = RunChain(32);
    for (const float velocity : converged.velocities) {
        EXPECT_NEAR(velocity, 2.0f / 3.0f, 1.0e-3f);
    }
    EXPECT_NEAR(converged.totalMomentum, 2.0f, 1.0e-4f);
}

// ---------------------------------------------------------------------------
// Warm start (Week 5) and bookkeeping edge cases.
// ---------------------------------------------------------------------------

TEST(ContactSolver, WarmStartAppliesTheSeededImpulsesExactlyOnce) {
    // Week 5 replaces the Week 3 no-op: WarmStart applies the seeded accumulated
    // impulses (already scaled by warmStartFactor in Prepare) once, with the same
    // -J/+J rule the iterations use.
    //
    // Geometry: A dynamic at the origin (m = 2 -> invMass 0.5, I = 4 ->
    // invInertia 0.25), B static at the origin, shared anchor at world (1, 2), so
    // rA = rB = (1, 2). normal = (0, 1) (A -> B), t = Skew(n) = (-1, 0).
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 2.0f, 4.0f));
    bodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));
    Contact contact = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{0.0f, 1.0f});
    contact.manifold.points[0].localPointA = Vec2{1.0f, 2.0f};
    contact.manifold.points[0].localPointB = Vec2{1.0f, 2.0f};
    contact.isPersistent = true;
    contact.manifold.points[0].normalImpulse = 4.0f;
    contact.manifold.points[0].tangentImpulse = -2.0f;
    std::vector<Contact> contacts{contact};

    SolverSettings settings;
    settings.warmStartFactor = 0.5f; // seeds: accN = 2, accT = -1
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedImpulse, 2.0f, kTolerance);
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedTangentImpulse, -1.0f, kTolerance);

    solver.WarmStart(bodies, contacts, settings);

    // J = n*accN + t*accT = (0,2) + (-1,0)*(-1) = (1, 2).
    // A receives -J: v = -J/m = (-0.5, -1); w = -Cross(rA, J)/I = 0 because rA is
    // parallel to J (Cross((1,2),(1,2)) == 0), so there is no angular response.
    EXPECT_NEAR(bodies[0].linearVelocity.x, -0.5f, kTolerance);
    EXPECT_NEAR(bodies[0].linearVelocity.y, -1.0f, kTolerance);
    EXPECT_NEAR(bodies[0].angularVelocity, 0.0f, kTolerance);
    // Static B ignores the impulse entirely.
    EXPECT_FLOAT_EQ(bodies[1].linearVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(bodies[1].linearVelocity.y, 0.0f);
    // The cached accumulator is still the seed (WarmStart applies, it does not
    // re-scale), so a second solve starts from exactly the applied impulse.
    EXPECT_NEAR(solver.GetConstraints()[0].accumulatedImpulse, 2.0f, kTolerance);

    // Disabled warm starting: the seeds are zero, so WarmStart is a clean no-op.
    std::vector<Body> coldBodies;
    coldBodies.push_back(MakeDynamic(0, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0.0f, 2.0f, 4.0f));
    coldBodies.push_back(MakeStatic(1, Vec2{0.0f, 0.0f}));
    std::vector<Contact> coldContacts{contact};
    SolverSettings coldSettings;
    coldSettings.useWarmStarting = false;
    ContactSolver coldSolver;
    coldSolver.Prepare(coldBodies, coldContacts, coldSettings);
    coldSolver.WarmStart(coldBodies, coldContacts, coldSettings);
    EXPECT_FLOAT_EQ(coldBodies[0].linearVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(coldBodies[0].linearVelocity.y, 0.0f);
    EXPECT_FLOAT_EQ(coldBodies[0].angularVelocity, 0.0f);
}

TEST(ContactSolver, ZeroIterationsWritesBackTheWarmStartSeed) {
    std::vector<Body> bodies;
    bodies.push_back(MakeDynamic(0, Vec2{-0.5f, 0.0f}, Vec2{2.0f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    Contact contact = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f});
    contact.manifold.points[0].normalImpulse = 7.0f; // stale value from a caller
    contact.isPersistent = false;
    std::vector<Contact> contacts{contact};

    SolverSettings settings;
    settings.warmStartFactor = 0.5f;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);

    // Week 5: Prepare no longer resets the field, but a non-persistent pair must
    // not use the stale value - the accumulator starts at zero and a zero- or
    // negative-iteration solve writes that back.
    EXPECT_FLOAT_EQ(solver.GetConstraints()[0].accumulatedImpulse, 0.0f);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].normalImpulse, 7.0f);
    solver.SolveVelocity(bodies, contacts, 0, settings);
    solver.SolveVelocity(bodies, contacts, -3, settings);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].normalImpulse, 0.0f);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].tangentImpulse, 0.0f);
    EXPECT_FLOAT_EQ(bodies[0].linearVelocity.x, 2.0f);

    // Persistent pair: the applied (warm-start) impulse is 7*0.5 = 3.5, so the
    // write-back must be 3.5 - not 7 and not 3.5*0.5. This is the invariant that
    // keeps the cached accumulator and the applied impulse from diverging.
    std::vector<Contact> persistentContacts{contact};
    persistentContacts[0].isPersistent = true;
    ContactSolver persistentSolver;
    persistentSolver.Prepare(bodies, persistentContacts, settings);
    ASSERT_EQ(persistentSolver.GetConstraintCount(), 1);
    EXPECT_NEAR(persistentSolver.GetConstraints()[0].accumulatedImpulse, 3.5f, kTolerance);
    persistentSolver.SolveVelocity(bodies, persistentContacts, 0, settings);
    EXPECT_NEAR(persistentContacts[0].manifold.points[0].normalImpulse, 3.5f, kTolerance);
}

// ---------------------------------------------------------------------------
// Degenerate pair contract: a static-static contact has K_n == 0. Prepare
// reports it with a debug assert and skips the point; in release builds the
// skip happens without aborting, so a mis-ordered contact list cannot crash the
// simulation.
// ---------------------------------------------------------------------------

namespace {

class SolverDeathTest : public p2test::DeathTestFixture {};

} // namespace

TEST_F(SolverDeathTest, StaticStaticContactAssertsOnNonPositiveMass) {
    std::vector<Body> bodies;
    bodies.push_back(MakeStatic(0, Vec2{-0.5f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f})};
    SolverSettings settings;
    ContactSolver solver;
    EXPECT_DEATH(solver.Prepare(bodies, contacts, settings), "non-positive effective normal mass");
}

#ifdef NDEBUG
TEST(ContactSolver, StaticStaticContactIsSkippedInReleaseBuilds) {
    // Release builds compile the assert out: the point must be skipped (not
    // divided by zero) so that the solver stays a total function.
    std::vector<Body> bodies;
    bodies.push_back(MakeStatic(0, Vec2{-0.5f, 0.0f}));
    bodies.push_back(MakeStatic(1, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f})};
    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    EXPECT_EQ(solver.GetConstraintCount(), 0);
    solver.SolveVelocity(bodies, contacts, 4, settings);
    EXPECT_FLOAT_EQ(contacts[0].manifold.points[0].normalImpulse, 0.0f);
}
#endif // NDEBUG
