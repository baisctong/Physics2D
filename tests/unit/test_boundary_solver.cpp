// Week 8 (W8-A) — geometry / solver boundary contract tests.
//
// WHY THIS FILE EXISTS
// --------------------
// The Week 8 DoD requires every known crash point to have either a debug assert
// or an explicitly documented limitation. Most individual rejections are already
// pinned down by the module owners' files (test_polygon_shape.cpp,
// test_world_sat.cpp, test_world_shapes.cpp, test_contact_solver.cpp,
// test_world_step.cpp). This file covers the remaining holes at the geometry /
// solver boundary and, where the input is legal, states the honest contract
// instead of pretending the physics is fine:
//
//   - degenerate polygons are rejected without crashing (self-contained
//     re-statement of the validation contract) and the Week 8 addition: FINITE
//     coordinates whose shoelace / edge-normal arithmetic overflows are rejected
//     too (before Week 8 such a shape was accepted with (0,0) edge normals);
//   - density boundaries: zero / negative / non-finite, static vs dynamic;
//   - extreme velocities above the kMaxTranslation per-step clamp: finite state,
//     valid fat AABB, valid SAP internal state, no NaN/Inf;
//   - a 1 x 100 m thin bar: huge AABB, huge contact lever arms, no NaN;
//   - an extreme 1 : 1e6 mass ratio: K_n finite and positive (no division by
//     zero), momentum conserved to float precision, no NaN;
//   - NaN/Inf inputs and illegal Step/Prepare/SolvePosition parameters through
//     the death-test contract pattern from TestSupport.h (the sandbox SKIPs those
//     with a visible reason; CI and a work-area-external run execute them).
//
// Every expected value is hand-computed in a comment next to the assertion; none
// is captured from a run. Death tests only apply to Debug builds (NDEBUG compiles
// the asserts out), exactly like the rest of the suite.

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Collision/Contact.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"
#include "physics2d/Solver/ContactSolver.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

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

BodyDef MakeBodyDef(BodyType type, Vec2 position, float density = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.density = density;
    return def;
}

// Hand-built solver body (same convention as test_contact_solver.cpp): the vector
// position equals handle.index so the solver can index bodies by handle.
Body MakeSolverBody(int32_t index, BodyType type, Vec2 position, Vec2 velocity = Vec2{0.0f, 0.0f},
                    float mass = 1.0f, float inertia = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.linearVelocity = velocity;
    def.mass = mass;
    def.inertia = inertia;
    Body body;
    body.Initialize(def);
    body.handle = BodyHandle{index, 1u};
    return body;
}

// One-point contact whose anchors are both body origins (zero lever arms), so the
// impulse formulas reduce to their linear form.
Contact MakeCentralContact(BodyHandle bodyA, BodyHandle bodyB, Vec2 normal, float penetration = 0.0f) {
    Contact contact;
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    contact.manifold.normal = normal;
    contact.manifold.penetration = penetration;
    contact.manifold.pointCount = 1;
    contact.manifold.points[0].localPointA = Vec2{0.0f, 0.0f};
    contact.manifold.points[0].localPointB = Vec2{0.0f, 0.0f};
    contact.manifold.points[0].id.type = p2::kContactTypeCircleCircle;
    return contact;
}

void AddGroundPlane(World& world) {
    const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2{0.0f, 0.0f}));
    ASSERT_TRUE(world.IsValid(ground));
    ASSERT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f)));
}

// True when every hot body field the boundary contract cares about is finite.
[[nodiscard]] bool IsFiniteBodyState(const World& world, BodyHandle handle) {
    const Body* body = world.GetBody(handle);
    if (body == nullptr) {
        return false;
    }
    return p2::IsValid(body->xf.p) && std::isfinite(body->xf.q.c) && std::isfinite(body->xf.q.s) &&
           p2::IsValid(body->linearVelocity) && std::isfinite(body->angularVelocity) &&
           p2::IsValid(body->pseudoLinearVelocity) && std::isfinite(body->pseudoAngularVelocity) &&
           p2::IsValid(body->force) && std::isfinite(body->torque) && body->aabb.IsValid();
}

} // namespace

// ---------------------------------------------------------------------------
// A. Geometry boundary: degenerate polygons, and the Week 8 overflow guard.
// ---------------------------------------------------------------------------

TEST(BoundaryPolygon, DegeneratePolygonsAreRejectedWithoutCrash) {
    // Self-contained restatement of the PolygonShape.h validation contract, one
    // case per rejection rule. All of these are plain value-level calls: no
    // assert, no crash, deterministic `false`, and the shape is left invalid.
    p2::PolygonShape shape;

    // (a) zero area: three collinear vertices.
    const Vec2 flat[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{2.0f, 0.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, flat, 3));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // (b) near-zero area: a sliver thinner than kLinearSlop^2 (the guard keeps the
    // centroid division away from zero).
    const Vec2 sliver[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.5f, 1.0e-6f}};
    EXPECT_FALSE(p2::SetPolygon(shape, sliver, 3));

    // (c) clockwise winding: shoelace area negative.
    const Vec2 clockwise[4] = {Vec2{-1.0f, -1.0f}, Vec2{-1.0f, 1.0f}, Vec2{1.0f, 1.0f}, Vec2{1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, clockwise, 4));

    // (d) concave vertex (negative turn while traversing CCW).
    const Vec2 concave[6] = {Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, Vec2{2.0f, 1.0f},
                             Vec2{1.0f, 1.0f}, Vec2{1.0f, 2.0f}, Vec2{0.0f, 2.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, concave, 6));

    // (e) duplicate adjacent vertex (closer than kLinearSlop: 1 mm < 5 mm).
    const Vec2 duplicate[5] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, -0.999f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f},
                               Vec2{-1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, duplicate, 5));

    // (f) redundant collinear vertex on an edge.
    const Vec2 collinear[5] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f},
                               Vec2{-1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, collinear, 5));

    // (g) non-finite coordinate and a null/invalid vertex array.
    const Vec2 nonFinite[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f},
                               Vec2{0.0f, std::numeric_limits<float>::quiet_NaN()}};
    EXPECT_FALSE(p2::SetPolygon(shape, nonFinite, 3));
    EXPECT_FALSE(p2::SetPolygon(shape, nullptr, 4));
    EXPECT_FALSE(p2::SetPolygon(shape, flat, 2)); // fewer than three vertices

    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());
}

TEST(BoundaryPolygon, FiniteButOverflowingCoordinatesAreRejected) {
    // Week 8 hardening (PolygonShape.cpp): a polygon can have FINITE coordinates
    // whose derived arithmetic still overflows float. This used to be accepted and
    // then silently produced (0,0) edge normals (Normalized() returns zero for an
    // infinite squared length) / a non-finite centroid, which poisons every SAT
    // axis and the mass properties one module later.
    p2::PolygonShape shape;

    // (a) half extents (1e19, 1e-2): the horizontal edges are 2e19 long, so
    // LengthSquared = 4e38 > FLT_MAX (3.4e38) overflows to +inf.
    EXPECT_FALSE(p2::SetPolygonAsBox(shape, 1.0e19f, 1.0e-2f));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // (b) both extents huge: both the squared edge lengths and the shoelace sum
    // overflow.
    EXPECT_FALSE(p2::SetPolygonAsBox(shape, 1.0e19f, 1.0e19f));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // (c) FLT_MAX-sized triangle: the vertex differences (2 * FLT_MAX) overflow to
    // +inf, so the squared edge length is not finite.
    const float maxFloat = std::numeric_limits<float>::max();
    const Vec2 hugeTriangle[3] = {Vec2{-maxFloat, 0.0f}, Vec2{maxFloat, 0.0f}, Vec2{0.0f, maxFloat}};
    EXPECT_FALSE(p2::SetPolygon(shape, hugeTriangle, 3));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // (d) shoelace-overflow guard: a diamond with vertices on the axes at radius
    // 1e19. Every edge has squared length 2e38 (finite, below FLT_MAX), but each
    // Cross(P_i, P_i+1) is 1e38 and their sum overflows to +inf; the centroid
    // division `1/(3*twiceArea)` would then multiply by 0 and produce NaN.
    const Vec2 hugeDiamond[4] = {Vec2{1.0e19f, 0.0f}, Vec2{0.0f, 1.0e19f}, Vec2{-1.0e19f, 0.0f},
                                 Vec2{0.0f, -1.0e19f}};
    EXPECT_FALSE(p2::SetPolygon(shape, hugeDiamond, 4));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // (e) centroid-overflow guard: axis-aligned square from (0,0) to (1e19,1e19).
    // The shoelace sum stays finite (twiceArea = 2e38) and every edge length is
    // finite, but the weighted centroid terms (P_i + P_i+1) * Cross reach ~1e57
    // and overflow: without the guard the shape would be accepted with an
    // infinite centroid and poison ComputePolygonMass / AttachShape later.
    const Vec2 hugeSquare[4] = {Vec2{0.0f, 0.0f}, Vec2{1.0e19f, 0.0f}, Vec2{1.0e19f, 1.0e19f},
                                Vec2{0.0f, 1.0e19f}};
    EXPECT_FALSE(p2::SetPolygon(shape, hugeSquare, 4));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());

    // A representable box is still accepted after the rejected calls (the guard
    // does not reject ordinary geometry).
    EXPECT_TRUE(p2::SetPolygonAsBox(shape, 0.5f, 0.5f));
    EXPECT_TRUE(shape.IsValid());
}

// ---------------------------------------------------------------------------
// B. Density boundary: zero (legal BodyDef) / negative / non-finite.
// ---------------------------------------------------------------------------

TEST(BoundaryDensity, NonFiniteAndNegativeDensityAreRejectedAtCreateBody) {
    // CreateBody validates the BodyDef at the boundary and returns an invalid
    // handle for negative or non-finite values (Section 3.4: never silently
    // repair). This is a return-value contract, not a debug assert: it holds in
    // every build.
    World world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float invalidDensities[] = {nan, inf, -inf, -1.0f};
    for (const float density : invalidDensities) {
        EXPECT_FALSE(
            world.IsValid(world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}, density))))
            << "density " << density;
        // The same material validation applies to static bodies: density stays a
        // material property even though static bodies derive no mass from it.
        EXPECT_FALSE(
            world.IsValid(world.CreateBody(MakeBodyDef(BodyType::Static, Vec2{0.0f, 0.0f}, density))))
            << "density " << density;
    }
    // Rejected definitions never consume a pool slot ...
    EXPECT_EQ(world.GetBodyCount(), 0);
    // ... and a valid definition is still accepted afterwards.
    EXPECT_TRUE(world.IsValid(world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}, 1.0f))));
}

TEST(BoundaryDensity, ZeroDensityDynamicBodyUsesTheBodyDefMassFallback) {
    // Contract: density == 0 is a legal BodyDef value (only negative/non-finite
    // values are rejected). Before a shape is attached there is nothing to derive
    // a mass from, so the BodyDef mass/inertia fallback applies. Attempting to
    // attach a shape then requires density > 0: that is a debug assert
    // (see BoundaryDensityDeathTest.ZeroDensityDynamicAttachAsserts) and a clean
    // `false` return in release builds.
    World world;
    BodyDef def = MakeBodyDef(BodyType::Dynamic, Vec2{1.0f, 2.0f}, 0.0f);
    def.mass = 2.5f;
    def.inertia = 4.0f;
    const BodyHandle body = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(body));
    EXPECT_FLOAT_EQ(world.GetMass(body), 2.5f);
    EXPECT_FLOAT_EQ(world.GetInvMass(body), 1.0f / 2.5f);
    const Body* state = world.GetBody(body);
    ASSERT_NE(state, nullptr);
    EXPECT_FLOAT_EQ(state->inertia, 4.0f);
    EXPECT_EQ(state->proxyId, -1);                              // no shape -> no broad-phase proxy
    EXPECT_EQ(state->aabb.lowerBound, world.GetPosition(body)); // point AABB
}

TEST(BoundaryDensity, StaticAndKinematicBodiesIgnoreDensityWhenAttaching) {
    // Attaching a shape to a static/kinematic body does not derive mass from the
    // density (they are infinite-mass by design), so density 0 is legal there.
    World world;
    const BodyType types[] = {BodyType::Static, BodyType::Kinematic};
    for (const BodyType type : types) {
        const BodyHandle body = world.CreateBody(MakeBodyDef(type, Vec2{0.0f, 0.0f}, 0.0f));
        ASSERT_TRUE(world.IsValid(body));
        ASSERT_TRUE(world.AttachShape(body, world.CreateCircleShape(0.5f)));
        EXPECT_FLOAT_EQ(world.GetMass(body), 0.0f); // infinite mass represented as 0
        EXPECT_FLOAT_EQ(world.GetInvMass(body), 0.0f);
    }
}

// ---------------------------------------------------------------------------
// C. Extreme velocity: per-step clamp, finite state, valid broad-phase state.
// ---------------------------------------------------------------------------

TEST(BoundaryVelocity, HugeVelocityIsClampedPerStepWithoutNonFiniteState) {
    // Gravity off so the only motion is the horizontal velocity. 1e6 m/s * dt is
    // 16666.7 m, far above kMaxTranslation = 2 m: the tunnelling defence clamps
    // the per-step displacement and keeps its direction.
    World world(Vec2{0.0f, 0.0f});
    auto broadPhase = std::make_unique<p2::BroadPhaseSAP>();
    p2::BroadPhaseSAP* sap = broadPhase.get();
    world.SetBroadPhase(std::move(broadPhase));

    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}));
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.AttachShape(body, world.CreateBoxShape(0.5f, 0.5f)));
    world.SetLinearVelocity(body, Vec2{1.0e6f, 0.0f});

    for (int32_t step = 0; step < 10; ++step) {
        world.Step(kDt);
        ASSERT_TRUE(IsFiniteBodyState(world, body)) << "step " << step;
        ASSERT_TRUE(sap->ValidateInternalState()) << "step " << step;
        const Vec2 position = world.GetPosition(body);
        // The clamp is per step and direction preserving: n steps travel exactly
        // n * kMaxTranslation (the same arithmetic as test_world_step.cpp).
        EXPECT_FLOAT_EQ(position.x, p2::kMaxTranslation * static_cast<float>(step + 1));
        EXPECT_FLOAT_EQ(position.y, 0.0f);
        // The clamp bounds displacement, not velocity: the body keeps 1e6 m/s and
        // the next step clamps again instead of exploding.
        EXPECT_FLOAT_EQ(p2::Length(world.GetLinearVelocity(body)), 1.0e6f);
        // The fat AABB was rebuilt around the clamped position and contains it.
        const Body* state = world.GetBody(body);
        ASSERT_NE(state, nullptr);
        EXPECT_TRUE(p2::Contains(state->aabb, position));
    }
    world.CheckSane();
}

TEST(BoundaryVelocity, HugeDownwardVelocityAgainstGroundStaysFinite) {
    // Same 2 m/step clamp against a static ground plane: with no CCD a body moving
    // faster than kMaxTranslation/dt = 120 m/s can pass through the plane in one
    // step (documented limitation, docs/math.md boundary section). The contract
    // checked here is only "finite state + valid broad-phase internal state", not
    // a collision result.
    World world; // default gravity (0, -10)
    auto broadPhase = std::make_unique<p2::BroadPhaseSAP>();
    p2::BroadPhaseSAP* sap = broadPhase.get();
    world.SetBroadPhase(std::move(broadPhase));
    AddGroundPlane(world);

    const BodyHandle ball = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 50.0f}));
    ASSERT_TRUE(world.IsValid(ball));
    ASSERT_TRUE(world.AttachShape(ball, world.CreateCircleShape(0.5f)));
    world.SetLinearVelocity(ball, Vec2{0.0f, -5.0e7f}); // 833 km per step unclamped

    for (int32_t step = 0; step < 30; ++step) {
        world.Step(kDt);
        ASSERT_TRUE(IsFiniteBodyState(world, ball)) << "step " << step;
        ASSERT_TRUE(sap->ValidateInternalState()) << "step " << step;
    }
    world.CheckSane();
}

// ---------------------------------------------------------------------------
// D. 1 x 100 m thin bar: huge AABB, huge lever arms, no NaN.
// ---------------------------------------------------------------------------

TEST(BoundaryThinBar, OneHundredMetreBarOnGroundStaysFinite) {
    // Box with half extents (50, 0.5) resting on the ground: the AABB is 100 m
    // wide, the contact anchors sit up to 50 m away from the centre of mass (huge
    // lever arms for the solver) and the SAP broad phase has to keep one very wide
    // proxy among ordinary ones.
    // Contract: no NaN/Inf, valid AABBs, valid SAP state and the bar stays in
    // contact with the ground. It does NOT claim the bar stays perfectly flat -
    // the 2-point PGS solve has a documented ordering bias on long lever arms.
    World world; // default gravity (0, -10)
    auto broadPhase = std::make_unique<p2::BroadPhaseSAP>();
    p2::BroadPhaseSAP* sap = broadPhase.get();
    world.SetBroadPhase(std::move(broadPhase));
    AddGroundPlane(world);

    const BodyHandle bar = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.5f}));
    ASSERT_TRUE(world.IsValid(bar));
    ASSERT_TRUE(world.AttachShape(bar, world.CreateBoxShape(50.0f, 0.5f)));
    const Body* initial = world.GetBody(bar);
    ASSERT_NE(initial, nullptr);
    EXPECT_NEAR(initial->aabb.GetWidth(), 100.0f, 1.0e-3f);

    for (int32_t step = 0; step < 200; ++step) {
        world.Step(kDt);
        ASSERT_TRUE(IsFiniteBodyState(world, bar)) << "step " << step;
        ASSERT_TRUE(sap->ValidateInternalState()) << "step " << step;
        const Body* state = world.GetBody(bar);
        ASSERT_NE(state, nullptr);
        ASSERT_TRUE(state->aabb.IsValid()) << "step " << step;
        // The fat AABB still spans the bar: 100 m solid + 2 * kAabbMargin. A small
        // residual rotation may shave a few cm off the width, so only the order of
        // magnitude is asserted (not an exact 100.2).
        EXPECT_GE(state->aabb.GetWidth(), 99.0f) << "step " << step;
        EXPECT_LE(state->aabb.GetWidth(), 102.0f) << "step " << step;
    }
    EXPECT_GT(world.GetContactCount(), 0) << "the bar must still be in contact with the ground";
    world.CheckSane();
}

// ---------------------------------------------------------------------------
// E. Extreme mass ratio 1 : 1e6.
// ---------------------------------------------------------------------------

TEST(BoundaryMassRatio, SolverEffectiveMassStaysFiniteAtOneToMillion) {
    // A = 1 kg (invMass 1) moving at +1 m/s towards B = 1e6 kg (invMass 1e-6) at
    // rest; zero lever arms (anchors at both body origins), so only the
    // translational terms contribute:
    //   K_n = invMassA + invMassB = 1 + 1e-6 = 1.000001
    // The reciprocal (the normal mass the solver divides by) is ~0.999999, i.e.
    // finite and positive: no division by zero, and the light body's term keeps
    // the heavy body from degenerating into a numerically immovable anchor.
    std::vector<Body> bodies;
    bodies.push_back(MakeSolverBody(0, BodyType::Dynamic, Vec2{-0.5f, 0.0f}, Vec2{1.0f, 0.0f}, 1.0f, 1.0f));
    bodies.push_back(
        MakeSolverBody(1, BodyType::Dynamic, Vec2{0.5f, 0.0f}, Vec2{0.0f, 0.0f}, 1.0e6f, 1.0e6f));
    std::vector<Contact> contacts{MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f})};

    const float effectiveMass = ContactSolver::ComputeEffectiveNormalMass(
        bodies[0], bodies[1], Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f});
    ASSERT_TRUE(std::isfinite(effectiveMass));
    EXPECT_GT(effectiveMass, 1.0f);
    EXPECT_NEAR(effectiveMass, 1.000001f, 2.0e-6f);

    SolverSettings settings;
    ContactSolver solver;
    solver.Prepare(bodies, contacts, settings);
    ASSERT_EQ(solver.GetConstraintCount(), 1);
    const float normalMass = solver.GetConstraints()[0].normalMass;
    ASSERT_TRUE(std::isfinite(normalMass));
    EXPECT_GT(normalMass, 0.0f);
    EXPECT_NEAR(normalMass, 1.0f, 1.0e-5f);

    // One iteration with the approach speed vRel.n = -1 m/s:
    //   lambda = -(vn + bias) / K_n = 1 / 1.000001 = 0.999999
    //   dvA = -lambda / mA  = -0.999999  -> vA = +1e-6 m/s
    //   dvB = +lambda / mB  = +0.999999e-6 -> vB = +0.999999e-6 m/s
    // The post-solve relative normal velocity is ~0 and the total momentum is
    // conserved: p = 1 * 1 = 1 kg m/s before and after.
    solver.SolveVelocity(bodies, contacts, 1, settings);
    ASSERT_TRUE(p2::IsValid(bodies[0].linearVelocity));
    ASSERT_TRUE(p2::IsValid(bodies[1].linearVelocity));
    EXPECT_NEAR(bodies[0].linearVelocity.x, 0.0f, 2.0e-6f);
    EXPECT_NEAR(bodies[1].linearVelocity.x, 1.0e-6f, 2.0e-7f);
    const float relativeNormalVelocity = bodies[1].linearVelocity.x - bodies[0].linearVelocity.x;
    EXPECT_NEAR(relativeNormalVelocity, 0.0f, 1.0e-6f);
    const float momentum = 1.0f * bodies[0].linearVelocity.x + 1.0e6f * bodies[1].linearVelocity.x;
    EXPECT_NEAR(momentum, 1.0f, 1.0e-4f);
    EXPECT_GE(contacts[0].manifold.points[0].normalImpulse, 0.0f);
    EXPECT_TRUE(std::isfinite(contacts[0].manifold.points[0].normalImpulse));
}

TEST(BoundaryMassRatio, WorldStackWithOneToMillionMassRatioStaysFinite) {
    // 1 m^2 box of density 1e6 (m = 1e6 kg) below a 1 kg box, both dynamic, on a
    // static plane. Contract: finite state for the whole 2 s, no NaN, the light
    // box is not lost or ejected by the mass ratio. (It does not claim the
    // 8-iteration solve converges as tightly as the equal-mass case.)
    World world; // default gravity (0, -10)
    AddGroundPlane(world);

    BodyDef heavyDef = MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.5f}, 1.0e6f);
    heavyDef.friction = 0.5f;
    const BodyHandle heavy = world.CreateBody(heavyDef);
    ASSERT_TRUE(world.IsValid(heavy));
    ASSERT_TRUE(world.AttachShape(heavy, world.CreateBoxShape(0.5f, 0.5f)));
    EXPECT_FLOAT_EQ(world.GetMass(heavy), 1.0e6f); // area 1 m^2 * density

    BodyDef lightDef = MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 1.5f}, 1.0f);
    lightDef.friction = 0.5f;
    const BodyHandle light = world.CreateBody(lightDef);
    ASSERT_TRUE(world.IsValid(light));
    ASSERT_TRUE(world.AttachShape(light, world.CreateBoxShape(0.5f, 0.5f)));
    EXPECT_FLOAT_EQ(world.GetMass(light), 1.0f);

    for (int32_t step = 0; step < 120; ++step) {
        world.Step(kDt);
        ASSERT_TRUE(IsFiniteBodyState(world, heavy)) << "step " << step;
        ASSERT_TRUE(IsFiniteBodyState(world, light)) << "step " << step;
    }
    // Both boxes are still where the stack belongs: the light box stayed above the
    // heavy one instead of being ejected / falling through (initial y 1.5).
    EXPECT_GT(world.GetPosition(light).y, 1.0f);
    EXPECT_GT(world.GetPosition(heavy).y, 0.0f);
    world.CheckSane();
}

// ---------------------------------------------------------------------------
// F. Debug-only contracts (asserts). The fixture turns each test into a visible
// SKIP when the assert is compiled out (NDEBUG) or when the sandbox forbids the
// temp-file write GoogleTest's death tests need; CI (and a work-area-external run)
// executes them for real. See tests/unit/TestSupport.h.
// ---------------------------------------------------------------------------

class BoundaryDensityDeathTest : public p2test::DeathTestFixture {};

TEST_F(BoundaryDensityDeathTest, ZeroDensityDynamicAttachAsserts) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}, 0.0f));
    const p2::ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.IsValidShape(circle));
    // World::AttachShape: "attaching to a Dynamic body requires density > 0".
    EXPECT_DEATH((void)world.AttachShape(body, circle), "requires density > 0");
}

TEST_F(BoundaryDensityDeathTest, HugeDensityOverflowingMassAsserts) {
    World world;
    // Box area = 4 m^2 (half extents 1), density 3.4e38 => mass = 1.36e39, which
    // overflows float. ComputePolygonMass rejects the non-finite mass.
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}, 3.4e38f));
    const p2::ShapeHandle box = world.CreateBoxShape(1.0f, 1.0f);
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.IsValidShape(box));
    EXPECT_DEATH((void)world.AttachShape(body, box), "non-finite mass");
}

TEST_F(BoundaryDensityDeathTest, WorldCreateShapeRejectsOverflowingGeometry) {
    World world;
    // Week 8 overflow guard reaches the World-level API as the same "invalid
    // handle + debug assert" contract as every other rejected geometry.
    EXPECT_DEATH((void)world.CreateBoxShape(1.0e19f, 1.0e-2f), "CreateBoxShape");
    const float maxFloat = std::numeric_limits<float>::max();
    const Vec2 hugeTriangle[3] = {Vec2{-maxFloat, 0.0f}, Vec2{maxFloat, 0.0f}, Vec2{0.0f, maxFloat}};
    EXPECT_DEATH((void)world.CreatePolygonShape(hugeTriangle, 3), "CreatePolygonShape");
    EXPECT_EQ(world.GetShapeCount(), 0); // rejections never consume a shape slot
}

class BoundarySolverDeathTest : public p2test::DeathTestFixture {};

TEST_F(BoundarySolverDeathTest, PrepareRejectsNonFiniteManifoldNormal) {
    std::vector<Body> bodies;
    bodies.push_back(MakeSolverBody(0, BodyType::Dynamic, Vec2{-0.5f, 0.0f}, Vec2{1.0f, 0.0f}));
    bodies.push_back(MakeSolverBody(1, BodyType::Dynamic, Vec2{0.5f, 0.0f}));
    Contact contact = MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f});
    contact.manifold.normal = Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f};
    std::vector<Contact> contacts{contact};
    SolverSettings settings;
    ContactSolver solver;
    EXPECT_DEATH(solver.Prepare(bodies, contacts, settings), "non-finite manifold normal/penetration");
}

TEST_F(BoundarySolverDeathTest, PrepareRejectsNonPositiveDt) {
    std::vector<Body> bodies;
    bodies.push_back(MakeSolverBody(0, BodyType::Dynamic, Vec2{-0.5f, 0.0f}));
    bodies.push_back(MakeSolverBody(1, BodyType::Dynamic, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f})};
    SolverSettings settings;
    ContactSolver solver;
    // Baumgarte bias is (beta / dt) * penetration: dt = 0 would divide by zero.
    EXPECT_DEATH(solver.Prepare(bodies, contacts, settings, 0.0f), "dt must be finite and > 0");
}

TEST_F(BoundarySolverDeathTest, SolvePositionRejectsNonFiniteDt) {
    std::vector<Body> bodies;
    bodies.push_back(MakeSolverBody(0, BodyType::Dynamic, Vec2{-0.5f, 0.0f}));
    bodies.push_back(MakeSolverBody(1, BodyType::Dynamic, Vec2{0.5f, 0.0f}));
    std::vector<Contact> contacts{
        MakeCentralContact(bodies[0].handle, bodies[1].handle, Vec2{1.0f, 0.0f}, 0.01f)};
    SolverSettings settings;
    ContactSolver solver;
    EXPECT_DEATH(solver.SolvePosition(bodies, contacts, std::numeric_limits<float>::quiet_NaN(), 3, settings),
                 "dt must be finite and > 0");
}

TEST_F(BoundarySolverDeathTest, SolvePositionRejectsOutOfRangeBodyIndex) {
    std::vector<Body> bodies;
    bodies.push_back(MakeSolverBody(0, BodyType::Dynamic, Vec2{-0.5f, 0.0f}));
    bodies.push_back(MakeSolverBody(1, BodyType::Dynamic, Vec2{0.5f, 0.0f}));
    // Hand-built contact pointing at a body slot that does not exist. The position
    // layer indexes the body vector directly, so this must be a readable assert
    // (before Week 8 it was undefined behaviour).
    Contact contact = MakeCentralContact(bodies[0].handle, BodyHandle{5, 1u}, Vec2{1.0f, 0.0f}, 0.01f);
    std::vector<Contact> contacts{contact};
    SolverSettings settings;
    ContactSolver solver;
    EXPECT_DEATH(solver.SolvePosition(bodies, contacts, kDt, 3, settings), "bodyB index out of range");
}

class BoundaryWorldDeathTest : public p2test::DeathTestFixture {};

TEST_F(BoundaryWorldDeathTest, StepRejectsInvalidDt) {
    World world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_DEATH(world.Step(nan), "dt must be finite and > 0");
    EXPECT_DEATH(world.Step(0.0f), "dt must be finite and > 0");
    EXPECT_DEATH(world.Step(-kDt), "dt must be finite and > 0");
}

TEST_F(BoundaryWorldDeathTest, StepRejectsNegativeIterationCounts) {
    World world;
    EXPECT_DEATH(world.Step(kDt, -1, 3), "iteration counts must be non-negative");
    EXPECT_DEATH(world.Step(kDt, 3, -1), "iteration counts must be non-negative");
}

TEST_F(BoundaryWorldDeathTest, ConstructorRejectsNonFiniteGravity) {
    EXPECT_DEATH(
        {
            World badWorld(Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f});
            (void)badWorld;
        },
        "non-finite gravity");
}

TEST_F(BoundaryWorldDeathTest, SetLinearVelocityRejectsNonFinite) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}));
    ASSERT_TRUE(world.IsValid(body));
    EXPECT_DEATH(world.SetLinearVelocity(body, Vec2{std::numeric_limits<float>::infinity(), 0.0f}),
                 "non-finite velocity");
}

TEST_F(BoundaryWorldDeathTest, SetTransformRejectsNonFinite) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}));
    ASSERT_TRUE(world.IsValid(body));
    EXPECT_DEATH(world.SetTransform(body, Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f}, 0.0f),
                 "non-finite transform");
}

TEST_F(BoundaryWorldDeathTest, SetFrictionRejectsNegativeOrNonFinite) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2{0.0f, 0.0f}));
    ASSERT_TRUE(world.IsValid(body));
    EXPECT_DEATH(world.SetFriction(body, -1.0f), "friction must be finite and >= 0");
    EXPECT_DEATH(world.SetFriction(body, std::numeric_limits<float>::infinity()),
                 "friction must be finite and >= 0");
}
