// Week 2 D3/D5: World contact generation, persistence and pair-filter tests.
//
// Contract (World.h / Contact.h): UpdateContacts rebuilds the list every step
// with O(N^2) enumeration + AABB filter and the analytic circle narrow phase;
// bodyA/bodyB are ordered by ascending body index; friction combines as the
// geometric mean, restitution as the max; per-point impulses are inherited from
// the previous frame by ContactID::Key() and isPersistent says whether the pair
// (and every point) existed before.
//
// Analytic expectations (from Analytic.cpp / P06):
//   circle above ground plane: normal = +y when A = plane (lower index),
//   penetration = radius - signedDistance, anchor = projection of the centre.

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <cstdint>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::ShapeHandle;
using p2::Transform;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;

struct Ground {
    BodyHandle body;
    ShapeHandle shape;
};

// Static body at the origin with an upward ground plane (normal +y, offset 0).
Ground AddGround(World& world, float friction = 0.3f, float restitution = 0.0f) {
    BodyDef def;
    def.type = BodyType::Static;
    def.friction = friction;
    def.restitution = restitution;
    Ground ground;
    ground.body = world.CreateBody(def);
    ground.shape = world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f);
    EXPECT_TRUE(world.AttachShape(ground.body, ground.shape));
    return ground;
}

// Dynamic circle body with the given radius and material.
BodyHandle AddBall(World& world, Vec2 position, float radius = 0.5f, float friction = 0.3f,
                   float restitution = 0.0f) {
    BodyDef def;
    def.type = BodyType::Dynamic;
    def.position = position;
    def.density = 1.0f;
    def.friction = friction;
    def.restitution = restitution;
    const BodyHandle body = world.CreateBody(def);
    EXPECT_TRUE(world.AttachShape(body, world.CreateCircleShape(radius)));
    return body;
}

// Re-projects a stored local anchor through the transform that was current WHEN
// the manifold was generated. The manifold is generated in phase 3, before
// IntegratePositions moves the body, so using the post-step transform would mix
// two different moments of the step.
Vec2 AnchorInWorld(const Transform& xfAtGeneration, Vec2 localPoint) {
    return p2::Mul(xfAtGeneration, localPoint);
}

TEST(WorldContactTest, BallAbovePlaneGeneratesNoContact) {
    World world;
    AddGround(world);
    AddBall(world, Vec2(0.0f, 2.0f)); // signed distance 2 m > radius 0.5 m

    world.Step(kDt);
    EXPECT_EQ(world.GetContactCount(), 0);
    EXPECT_TRUE(world.GetContacts().empty());
}

TEST(WorldContactTest, TangentCircleIsOnePointContactWithZeroPenetration) {
    World world;
    const Ground ground = AddGround(world);
    const BodyHandle ball = AddBall(world, Vec2(0.0f, 0.5f)); // exactly tangent
    ASSERT_TRUE(world.IsValid(ball));

    // Snapshot the transforms of the moment the manifold will be generated in
    // (phase 3), before the step integrates positions.
    const Transform groundTransformAtGeneration = world.GetTransform(ground.body);
    const Transform ballTransformAtGeneration = world.GetTransform(ball);

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& contact = world.GetContacts()[0];

    // A = plane (index 0), B = ball (index 1): the plane's world normal is +y,
    // and CollidePlaneAndCircle flips the circle-plane manifold, so the stored
    // normal points from A to B == +y.
    EXPECT_EQ(contact.bodyA, ground.body);
    EXPECT_EQ(contact.bodyB, ball);
    EXPECT_FLOAT_EQ(contact.manifold.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, 1.0f);
    // Tangent case: penetration is exactly 0 but pointCount == 1 (Manifold
    // contract: touching is a contact).
    EXPECT_FLOAT_EQ(contact.manifold.penetration, 0.0f);
    EXPECT_EQ(contact.manifold.pointCount, 1);

    // The anchor is the touch point (0, 0) in world space. In the plane's local
    // frame that is (0, 0); in the ball's local frame (origin at the centre
    // (0, 0.5)) it is (0, -0.5). Both must re-project to the same world point
    // when using the transform of the moment the manifold was generated (phase 3,
    // before IntegratePositions moved the ball down by g*dt^2).
    const Vec2 anchorA = AnchorInWorld(groundTransformAtGeneration, contact.manifold.points[0].localPointA);
    const Vec2 anchorB = AnchorInWorld(ballTransformAtGeneration, contact.manifold.points[0].localPointB);
    EXPECT_NEAR(contact.manifold.points[0].localPointA.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointA.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointB.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointB.y, -0.5f, 1.0e-6f);
    EXPECT_NEAR(anchorA.x, 0.0f, 1.0e-5f);
    EXPECT_NEAR(anchorA.y, 0.0f, 1.0e-5f);
    EXPECT_NEAR(anchorB.x, 0.0f, 1.0e-5f);
    EXPECT_NEAR(anchorB.y, 0.0f, 1.0e-5f);

    // ContactID encodes the circle-plane feature; the flip bit is set because the
    // geometry is evaluated as (circle, plane) and then flipped for A/B order.
    EXPECT_EQ(contact.manifold.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(contact.manifold.points[0].id.flip, 1);
    // Materials: sqrt(0.3 * 0.3) = 0.3, max(0, 0) = 0.
    EXPECT_NEAR(contact.friction, 0.3f, 1.0e-6f);
    EXPECT_FLOAT_EQ(contact.restitution, 0.0f);
    EXPECT_FALSE(contact.isPersistent);
}

TEST(WorldContactTest, PenetratingCircleReportsAnalyticPenetration) {
    World world;
    AddGround(world);
    const BodyHandle ball = AddBall(world, Vec2(0.0f, 0.3f)); // signed distance 0.3
    ASSERT_TRUE(world.IsValid(ball));

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& contact = world.GetContacts()[0];
    // penetration = radius - signedDistance = 0.5 - 0.3 = 0.2.
    EXPECT_NEAR(contact.manifold.penetration, 0.2f, 1.0e-6f);
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, 1.0f);
    // The manifold is generated BEFORE the temporary response moves the body, so
    // the stored local anchors describe the pre-response geometry:
    //   world anchor = centre - normal * signedDistance = (0, 0),
    //   ball-local   = (0, 0) - (0, 0.3) = (0, -0.3),
    //   ground-local = (0, 0).
    EXPECT_NEAR(contact.manifold.points[0].localPointB.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointB.y, -0.3f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointA.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointA.y, 0.0f, 1.0e-6f);
}

TEST(WorldContactTest, ContactPersistsWithStableKeyAcrossFrames) {
    World world;
    AddGround(world);
    const BodyHandle ball = AddBall(world, Vec2(0.0f, 0.3f));
    ASSERT_TRUE(world.IsValid(ball));

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    EXPECT_FALSE(world.GetContacts()[0].isPersistent); // first frame of the pair
    const uint16_t firstKey = world.GetContacts()[0].manifold.points[0].id.Key();

    // The contact persists from frame to frame: the solver removes the approach
    // velocity and the split-impulse position layer settles the ball in the slop
    // band (penetration ~ kLinearSlop = 0.005 m), so the same physical contact
    // point must produce the same ContactID::Key() on the second step.
    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& second = world.GetContacts()[0];
    EXPECT_TRUE(second.isPersistent);
    EXPECT_EQ(second.manifold.points[0].id.Key(), firstKey);
    // Persistence matching has found the predecessor's point. Since Week 3 the
    // solver writes the accumulated normal impulse back into the contact point
    // every step (the accumulated clamp keeps it non-negative); only Week 5 warm
    // starting actually APPLIES that inherited value again.
    EXPECT_GE(second.manifold.points[0].normalImpulse, 0.0f);
}

TEST(WorldContactTest, AabbFilterRejectsDistantBodies) {
    World world(Vec2(0.0f, 0.0f)); // gravity off: the pair only moves if it collides
    // Two circles at x = -10 and +10 (radius 0.5): the AABBs [-10.5, -9.5] and
    // [9.5, 10.5] do not overlap, so the narrow phase is never asked.
    const BodyHandle left = AddBall(world, Vec2(-10.0f, 0.0f));
    const BodyHandle right = AddBall(world, Vec2(10.0f, 0.0f));
    ASSERT_TRUE(world.IsValid(left));
    ASSERT_TRUE(world.IsValid(right));

    world.Step(kDt);
    EXPECT_EQ(world.GetContactCount(), 0);
}

TEST(WorldContactTest, InfiniteMassPairsGenerateNoContacts) {
    World world(Vec2(0.0f, 0.0f)); // gravity off
    // All five bodies sit on top of each other; only pairs with at least one
    // dynamic body may generate a contact (nothing could push an infinite-mass
    // body apart, and the response is mass-weighted).
    BodyDef staticDef;
    staticDef.type = BodyType::Static;
    BodyDef kinematicDef;
    kinematicDef.type = BodyType::Kinematic;

    const BodyHandle staticA = world.CreateBody(staticDef);
    const BodyHandle staticB = world.CreateBody(staticDef);
    const BodyHandle kinematicA = world.CreateBody(kinematicDef);
    const BodyHandle kinematicB = world.CreateBody(kinematicDef);
    for (BodyHandle body : {staticA, staticB, kinematicA, kinematicB}) {
        ASSERT_TRUE(world.AttachShape(body, world.CreateCircleShape(1.0f)));
    }

    world.Step(kDt);
    EXPECT_EQ(world.GetContactCount(), 0);
}

TEST(WorldContactTest, TwoBallsOnGroundProduceDeterministicallyOrderedContacts) {
    World world;
    const Ground ground = AddGround(world);
    const BodyHandle leftBall = AddBall(world, Vec2(-2.0f, 0.5f)); // tangent
    const BodyHandle rightBall = AddBall(world, Vec2(2.0f, 0.5f)); // tangent
    ASSERT_TRUE(world.IsValid(leftBall));
    ASSERT_TRUE(world.IsValid(rightBall));

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 2);
    // i < j enumeration: every pair (plane=0, left=1) then (plane=0, right=2);
    // bodyA is always the lower index, so the order is fixed by construction.
    EXPECT_EQ(world.GetContacts()[0].bodyA, ground.body);
    EXPECT_EQ(world.GetContacts()[0].bodyB, leftBall);
    EXPECT_EQ(world.GetContacts()[1].bodyA, ground.body);
    EXPECT_EQ(world.GetContacts()[1].bodyB, rightBall);
    EXPECT_LT(world.GetContacts()[0].bodyB.index, world.GetContacts()[1].bodyB.index);
}

TEST(WorldContactTest, CircleCircleNormalPointsFromLowerIndexBody) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle bodyA = AddBall(world, Vec2(0.0f, 0.0f)); // index 0
    const BodyHandle bodyB = AddBall(world, Vec2(0.5f, 0.0f)); // index 1
    ASSERT_TRUE(world.IsValid(bodyA));
    ASSERT_TRUE(world.IsValid(bodyB));

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& contact = world.GetContacts()[0];
    EXPECT_EQ(contact.bodyA, bodyA);
    EXPECT_EQ(contact.bodyB, bodyB);
    // centre distance 0.5, radii 0.5 + 0.5 -> penetration 0.5; normal = +x.
    EXPECT_NEAR(contact.manifold.penetration, 0.5f, 1.0e-6f);
    EXPECT_FLOAT_EQ(contact.manifold.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, 0.0f);
    EXPECT_EQ(contact.manifold.points[0].id.type, p2::kContactTypeCircleCircle);
    // Anchor = midpoint of the overlap band on the normal axis:
    //   centreA + normal * (rA - penetration/2) = (0.25, 0).
    EXPECT_NEAR(contact.manifold.points[0].localPointA.x, 0.25f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointA.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(contact.manifold.points[0].localPointB.x, -0.25f, 1.0e-6f);
}

TEST(WorldContactTest, ReversedCreationOrderFlipsThePlaneNormal) {
    World world;
    // Ball created first (index 0) and ground second (index 1): A = circle,
    // B = plane, so the stored normal is -worldNormal = (0, -1).
    const BodyHandle ball = AddBall(world, Vec2(0.0f, 0.5f));
    const Ground ground = AddGround(world);
    ASSERT_TRUE(world.IsValid(ball));

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& contact = world.GetContacts()[0];
    EXPECT_EQ(contact.bodyA, ball);
    EXPECT_EQ(contact.bodyB, ground.body);
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, -1.0f);
    EXPECT_FLOAT_EQ(contact.manifold.penetration, 0.0f);
    EXPECT_EQ(contact.manifold.points[0].id.type, p2::kContactTypeCirclePlane);
    // Not flipped in this order: CollideCircleAndPlane produced the manifold.
    EXPECT_EQ(contact.manifold.points[0].id.flip, 0);
}

TEST(WorldContactTest, CombinedMaterialsFollowTheDocumentedRules) {
    World world;
    // friction = sqrt(0.4 * 0.9) = sqrt(0.36) = 0.6; restitution = max(0, 0.5).
    AddGround(world, 0.4f, 0.0f);
    AddBall(world, Vec2(0.0f, 0.5f), 0.5f, 0.9f, 0.5f);

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const Contact& contact = world.GetContacts()[0];
    EXPECT_NEAR(contact.friction, 0.6f, 1.0e-6f);
    EXPECT_FLOAT_EQ(contact.restitution, 0.5f);
}

TEST(WorldContactTest, BodyWithoutShapeNeverGeneratesContacts) {
    World world(Vec2(0.0f, 0.0f));
    AddBall(world, Vec2(0.0f, 0.0f)); // collider
    // A second dynamic body at the same place without a shape: pair needs two
    // valid shapes, so it must be skipped.
    BodyDef def;
    def.type = BodyType::Dynamic;
    const BodyHandle colliderless = world.CreateBody(def);
    ASSERT_TRUE(world.IsValid(colliderless));

    world.Step(kDt);
    EXPECT_EQ(world.GetContactCount(), 0);
}

TEST(WorldContactTest, DestroyBodyDropsCachedContacts) {
    World world;
    const Ground ground = AddGround(world);
    const BodyHandle ball = AddBall(world, Vec2(0.0f, 0.5f));
    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);

    world.DestroyBody(ball);
    // Readers (debug UI, testbed) must never see a contact whose body handle has
    // just gone stale; the next UpdateContacts would rebuild the list anyway.
    EXPECT_EQ(world.GetContactCount(), 0);
    EXPECT_TRUE(world.GetContacts().empty());
    EXPECT_TRUE(world.IsValid(ground.body));
}

TEST(WorldContactTest, ClearResetsContactsShapesAndBodies) {
    World world;
    AddGround(world);
    AddBall(world, Vec2(0.0f, 0.5f));
    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);

    world.Clear();
    EXPECT_EQ(world.GetContactCount(), 0);
    EXPECT_EQ(world.GetShapeCount(), 0);
    EXPECT_EQ(world.GetBodyCount(), 0);
    EXPECT_TRUE(world.GetContacts().empty());
}

} // namespace
