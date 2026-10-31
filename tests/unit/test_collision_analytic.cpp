// Unit tests for the Week 2 analytic circle collision paths (design document
// Week 2 D2/D4, P06): CollideCircles, CollideCircleAndPlane,
// CollidePlaneAndCircle and FlipManifold.
//
// Every configuration is axis-aligned or built from exact trigonometric values
// (90 degree rotations), so the expected normals, penetrations and contact
// points below are hand-computed and the arithmetic is written in the comments.
// The rotating/scaled cases at the bottom are property checks (anchor round trip,
// unit normal) rather than hand-computed geometry.

#include <gtest/gtest.h>

#include "physics2d/Collision/Analytic.h"
#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <cstdint>

namespace {

using p2::CircleShape;
using p2::ContactPoint;
using p2::Manifold;
using p2::PlaneShape;
using p2::Rot;
using p2::Transform;
using p2::Vec2;

// Touching/overlap results are ratios of the input radii and distances, so 1e-6
// covers the float rounding of the square root and the transform multiplications.
constexpr float kTolerance = 1.0e-6f;

// Round-trip tolerance for re-deriving a world anchor from a local one: the
// inverse transform then forward transform costs a few ulps of the coordinate
// magnitude, which is at most a few metres in these tests. The task requires
// agreement below 1e-5.
constexpr float kAnchorTolerance = 1.0e-5f;

void ExpectNearVec2(Vec2 actual, Vec2 expected, float tolerance) {
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
}

// The solver rebuilds the world lever arms every frame, so the anchor stored in
// A's frame and the one stored in B's frame must describe the same world point.
void ExpectWorldAnchorsAgree(const Manifold& manifold, const Transform& xfA, const Transform& xfB,
                             float tolerance) {
    ASSERT_GT(manifold.pointCount, 0);
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        const Vec2 worldA = p2::Mul(xfA, manifold.points[i].localPointA);
        const Vec2 worldB = p2::Mul(xfB, manifold.points[i].localPointB);
        EXPECT_NEAR(worldA.x, worldB.x, tolerance) << "point " << i;
        EXPECT_NEAR(worldA.y, worldB.y, tolerance) << "point " << i;
    }
}

// Two equal circles (r = 0.5) whose centres are `x` apart on the x axis.
CircleShape MakeCircle(float radius) {
    CircleShape circle;
    circle.radius = radius;
    return circle;
}

Transform MakeTranslation(float x, float y) {
    return Transform{Vec2{x, y}, Rot()};
}

} // namespace

TEST(CollideCircles, SeparatedCirclesProduceAnEmptyManifold) {
    // rA + rB = 0.5 + 0.5 = 1.0; centres are 2.0 apart, so the gap is 1.0.
    const CircleShape circle = MakeCircle(0.5f);
    const Manifold manifold =
        p2::CollideCircles(circle, MakeTranslation(0.0f, 0.0f), circle, MakeTranslation(2.0f, 0.0f));

    EXPECT_EQ(manifold.pointCount, 0);
    EXPECT_FALSE(manifold.IsTouching());
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
}

TEST(CollideCircles, TangentCirclesTouchWithZeroPenetration) {
    // Distance 1.0 == rA + rB, so the circles are exactly tangent:
    //   penetration = 1.0 - 1.0 = 0
    //   normal      = (delta / |delta|) = (1, 0)  (from A toward B)
    //   contact     = centerA + normal * (rA - penetration/2) = (0.5, 0)
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfA = MakeTranslation(0.0f, 0.0f);
    const Transform xfB = MakeTranslation(1.0f, 0.0f);
    const Manifold manifold = p2::CollideCircles(circle, xfA, circle, xfB);

    ASSERT_EQ(manifold.pointCount, 1);
    EXPECT_TRUE(manifold.IsTouching());
    // Exactly representable inputs: 1.0 - 1.0 is exactly zero.
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
    EXPECT_FLOAT_EQ(manifold.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(manifold.normal.y, 0.0f);

    EXPECT_NEAR(manifold.points[0].localPointA.x, 0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.y, 0.0f, kTolerance);
    // A world point (0.5, 0) in B's frame is 0.5 - 1.0 = -0.5 on x.
    EXPECT_NEAR(manifold.points[0].localPointB.x, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointB.y, 0.0f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, xfA, xfB, kAnchorTolerance);

    // The type field is the Week 2 circle-circle encoding; polygon indices are 0.
    EXPECT_EQ(manifold.points[0].id.type, p2::kContactTypeCircleCircle);
    EXPECT_EQ(manifold.points[0].id.referenceEdge, 0);
    EXPECT_EQ(manifold.points[0].id.incidentVertex, 0);

    // Tangent from the other side: normal points from A toward B, i.e. (-1, 0).
    const Manifold left =
        p2::CollideCircles(circle, MakeTranslation(0.0f, 0.0f), circle, MakeTranslation(-1.0f, 0.0f));
    ASSERT_EQ(left.pointCount, 1);
    EXPECT_FLOAT_EQ(left.penetration, 0.0f);
    EXPECT_FLOAT_EQ(left.normal.x, -1.0f);
    EXPECT_FLOAT_EQ(left.normal.y, 0.0f);
}

TEST(CollideCircles, OverlappingCirclesReportPenetrationAlongTheNormal) {
    // Centres 0.6 apart with radius sum 1.0:
    //   penetration = 1.0 - 0.6 = 0.4
    //   normal      = (1, 0)
    //   contact     = (0, 0) + (1, 0) * (0.5 - 0.2) = (0.3, 0)
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfA = MakeTranslation(0.0f, 0.0f);
    const Transform xfB = MakeTranslation(0.6f, 0.0f);
    const Manifold manifold = p2::CollideCircles(circle, xfA, circle, xfB);

    ASSERT_EQ(manifold.pointCount, 1);
    EXPECT_NEAR(manifold.penetration, 0.4f, kTolerance);
    EXPECT_FLOAT_EQ(manifold.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(manifold.normal.y, 0.0f);
    EXPECT_NEAR(manifold.points[0].localPointA.x, 0.3f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.y, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointB.x, -0.3f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, xfA, xfB, kAnchorTolerance);

    // Different radii, vertical layout: rA = 1, rB = 2, centres 2.5 apart:
    //   penetration = 3 - 2.5 = 0.5
    //   normal      = (0, 1)
    //   contact     = (0, 0) + (0, 1) * (1 - 0.25) = (0, 0.75)
    const CircleShape big = MakeCircle(2.0f);
    const CircleShape small = MakeCircle(1.0f);
    const Transform xfSmall = MakeTranslation(0.0f, 0.0f);
    const Transform xfBig = MakeTranslation(0.0f, 2.5f);
    const Manifold vertical = p2::CollideCircles(small, xfSmall, big, xfBig);

    ASSERT_EQ(vertical.pointCount, 1);
    EXPECT_NEAR(vertical.penetration, 0.5f, kTolerance);
    EXPECT_FLOAT_EQ(vertical.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(vertical.normal.y, 1.0f);
    EXPECT_NEAR(vertical.points[0].localPointA.y, 0.75f, kTolerance);
    // The same world point in the big circle's frame: 0.75 - 2.5 = -1.75.
    EXPECT_NEAR(vertical.points[0].localPointB.y, -1.75f, kTolerance);
    ExpectWorldAnchorsAgree(vertical, xfSmall, xfBig, kAnchorTolerance);
}

TEST(CollideCircles, CoincidentCentresUseADeterministicNormal) {
    // Geometric direction is undefined for coincident centres. The documented
    // fallback is the fixed +x axis: penetration = rA + rB = 1.0, no NaN, and the
    // same scene always produces the same manifold (Section 7.4 determinism).
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xf = MakeTranslation(1.0f, 2.0f);
    const Manifold manifold = p2::CollideCircles(circle, xf, circle, xf);

    ASSERT_EQ(manifold.pointCount, 1);
    EXPECT_FLOAT_EQ(manifold.penetration, 1.0f);
    EXPECT_FLOAT_EQ(manifold.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(manifold.normal.y, 0.0f);
    EXPECT_TRUE(std::isfinite(manifold.points[0].localPointA.x));
    EXPECT_TRUE(std::isfinite(manifold.points[0].localPointA.y));

    // Determinism: a second identical call produces the same values bit for bit.
    const Manifold again = p2::CollideCircles(circle, xf, circle, xf);
    EXPECT_FLOAT_EQ(again.penetration, manifold.penetration);
    EXPECT_TRUE(again.points[0].localPointA == manifold.points[0].localPointA);

    // Coincident centres with different local centres and transforms: the same
    // fallback applies because the world centres still coincide.
    CircleShape offsetA = MakeCircle(0.5f);
    offsetA.localCenter = Vec2{1.0f, 2.0f};
    CircleShape offsetB = MakeCircle(0.5f);
    offsetB.localCenter = Vec2{-1.0f, -1.0f};
    // centerA = (1, 2); centerB = (2, 3) + (-1, -1) = (1, 2).
    const Transform xfA = MakeTranslation(0.0f, 0.0f);
    const Transform xfB = MakeTranslation(2.0f, 3.0f);
    const Manifold shifted = p2::CollideCircles(offsetA, xfA, offsetB, xfB);
    ASSERT_EQ(shifted.pointCount, 1);
    EXPECT_FLOAT_EQ(shifted.penetration, 1.0f);
    EXPECT_FLOAT_EQ(shifted.normal.x, 1.0f);
    EXPECT_FLOAT_EQ(shifted.normal.y, 0.0f);
}

TEST(CollideCircles, SwappingArgumentsMirrorsTheManifold) {
    // A at (0,0) r=0.5, B at (0.6,0) r=0.5: penetration 0.4, normal (1,0).
    // Evaluating (B,A) must flip the normal, keep the penetration and point
    // count, and keep the same ContactID Key (the feature encoding has no A/B
    // ordering for circles, so the same physical contact must produce one key).
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfA = MakeTranslation(0.0f, 0.0f);
    const Transform xfB = MakeTranslation(0.6f, 0.0f);

    const Manifold ab = p2::CollideCircles(circle, xfA, circle, xfB);
    const Manifold ba = p2::CollideCircles(circle, xfB, circle, xfA);
    ASSERT_EQ(ab.pointCount, 1);
    ASSERT_EQ(ba.pointCount, 1);

    EXPECT_FLOAT_EQ(ba.normal.x, -ab.normal.x);
    EXPECT_FLOAT_EQ(ba.normal.y, -ab.normal.y);
    EXPECT_FLOAT_EQ(ba.penetration, ab.penetration);
    EXPECT_EQ(ba.points[0].id.Key(), ab.points[0].id.Key());

    // The anchors are the same physical points, just stored in the other body's
    // frame: localPointA(ba) must equal localPointB(ab), up to rounding.
    ExpectNearVec2(ba.points[0].localPointA, ab.points[0].localPointB, kAnchorTolerance);
    ExpectNearVec2(ba.points[0].localPointB, ab.points[0].localPointA, kAnchorTolerance);
    ExpectWorldAnchorsAgree(ba, xfB, xfA, kAnchorTolerance);
}

TEST(CollideCircleAndPlane, SeparatedCircleAbovePlaneProducesEmptyManifold) {
    // Ground plane normal (0, 1), offset 0. Circle centre at y = 2, r = 0.5:
    //   signedDistance = Dot((0,1), (0,2)) - 0 = 2 > 0.5  => separated.
    const PlaneShape plane;
    const CircleShape circle = MakeCircle(0.5f);
    const Manifold manifold =
        p2::CollideCircleAndPlane(circle, MakeTranslation(0.0f, 2.0f), plane, Transform{});

    EXPECT_EQ(manifold.pointCount, 0);
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
}

TEST(CollideCircleAndPlane, TangentCircleTouchesWithZeroPenetration) {
    // Centre at y = 0.5 with r = 0.5 => signedDistance = 0.5, penetration = 0.
    // normal   = -planeWorldNormal = (0, -1)   (A = circle -> B = plane)
    // contact  = center - worldNormal * signedDistance = (0, 0.5) - (0, 0.5) = (0, 0)
    const PlaneShape plane;
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfCircle = MakeTranslation(0.0f, 0.5f);
    const Transform xfPlane = Transform{};
    const Manifold manifold = p2::CollideCircleAndPlane(circle, xfCircle, plane, xfPlane);

    ASSERT_EQ(manifold.pointCount, 1);
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
    EXPECT_FLOAT_EQ(manifold.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(manifold.normal.y, -1.0f);
    // The circle body is at (0, 0.5) and the touch point is the world origin:
    // localPointA = worldPoint - position = (0, 0) - (0, 0.5) = (0, -0.5).
    EXPECT_NEAR(manifold.points[0].localPointA.x, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.y, -0.5f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, xfCircle, xfPlane, kAnchorTolerance);
    EXPECT_EQ(manifold.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(manifold.points[0].id.flip, 0);
}

TEST(CollideCircleAndPlane, PenetratingAndDeeplyPenetratingCircles) {
    const PlaneShape plane;
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPlane = Transform{};

    // Shallow: centre y = 0.2 => signedDistance 0.2, penetration = 0.5 - 0.2 = 0.3.
    // Contact = (0, 0.2) - (0,1)*0.2 = (0, 0).
    const Manifold shallow = p2::CollideCircleAndPlane(circle, MakeTranslation(0.0f, 0.2f), plane, xfPlane);
    ASSERT_EQ(shallow.pointCount, 1);
    EXPECT_NEAR(shallow.penetration, 0.3f, kTolerance);
    EXPECT_FLOAT_EQ(shallow.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(shallow.normal.y, -1.0f);
    // Body at (0, 0.2), contact at the world origin -> local (0, -0.2).
    EXPECT_NEAR(shallow.points[0].localPointA.x, 0.0f, kTolerance);
    EXPECT_NEAR(shallow.points[0].localPointA.y, -0.2f, kTolerance);

    // Deep: the centre is below the surface (signedDistance = -0.3), which is the
    // "tunnelled through the ground" state. The analytic formula still gives the
    // overlap that has to be removed: penetration = 0.5 + 0.3 = 0.8, and the
    // contact is the projection of the centre onto the surface, (0, 0).
    const Manifold deep = p2::CollideCircleAndPlane(circle, MakeTranslation(0.0f, -0.3f), plane, xfPlane);
    ASSERT_EQ(deep.pointCount, 1);
    EXPECT_NEAR(deep.penetration, 0.8f, kTolerance);
    EXPECT_FLOAT_EQ(deep.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(deep.normal.y, -1.0f);
    // Body at (0, -0.3), contact at the world origin -> local (0, +0.3).
    EXPECT_NEAR(deep.points[0].localPointA.x, 0.0f, kTolerance);
    EXPECT_NEAR(deep.points[0].localPointA.y, 0.3f, kTolerance);
    ExpectWorldAnchorsAgree(deep, MakeTranslation(0.0f, -0.3f), xfPlane, kAnchorTolerance);

    // With a non-zero local centre the world centre must include the transform:
    // localCenter (0.25, 0), body at (0, 0.3) => world centre (0.25, 0.3),
    // signedDistance 0.3, penetration 0.2, contact = (0.25, 0).
    CircleShape offsetCircle = MakeCircle(0.5f);
    offsetCircle.localCenter = Vec2{0.25f, 0.0f};
    const Manifold offset =
        p2::CollideCircleAndPlane(offsetCircle, MakeTranslation(0.0f, 0.3f), plane, xfPlane);
    ASSERT_EQ(offset.pointCount, 1);
    EXPECT_NEAR(offset.penetration, 0.2f, kTolerance);
    // Contact world point (0.25, 0); body at (0, 0.3) -> local (0.25, -0.3).
    EXPECT_NEAR(offset.points[0].localPointA.x, 0.25f, kTolerance);
    EXPECT_NEAR(offset.points[0].localPointA.y, -0.3f, kTolerance);
    ExpectWorldAnchorsAgree(offset, MakeTranslation(0.0f, 0.3f), xfPlane, kAnchorTolerance);
}

TEST(CollideCircleAndPlane, RotatedAndTranslatedPlaneUsesWorldNormalAndOffset) {
    // Plane local: normal (0,1), offset 0. Body transform: 90 deg CCW + (3, 4).
    //   worldNormal = Rot90 * (0, 1) = (-1, 0)
    //   worldOffset = 0 + Dot((-1, 0), (3, 4)) = -3
    // The surface is therefore x = 3, with the solid on the +x side.
    const PlaneShape plane;
    const Transform xfPlane{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const CircleShape circle = MakeCircle(0.5f);

    // Tangent: centre (2.5, 4) has signedDistance = Dot((-1,0),(2.5,4)) - (-3)
    // = -2.5 + 3 = 0.5 = r, so penetration is 0 and contact = (3, 4).
    const Manifold tangent = p2::CollideCircleAndPlane(circle, MakeTranslation(2.5f, 4.0f), plane, xfPlane);
    ASSERT_EQ(tangent.pointCount, 1);
    EXPECT_NEAR(tangent.penetration, 0.0f, kTolerance);
    // normal = -worldNormal = (1, 0); the tiny y component is sin(pi/2) rounding.
    EXPECT_NEAR(tangent.normal.x, 1.0f, kTolerance);
    EXPECT_NEAR(tangent.normal.y, 0.0f, kTolerance);
    EXPECT_NEAR(tangent.points[0].localPointA.x, 3.0f - 2.5f, kTolerance);
    EXPECT_NEAR(tangent.points[0].localPointB.x, 3.0f - 3.0f, kTolerance);
    ExpectWorldAnchorsAgree(tangent, MakeTranslation(2.5f, 4.0f), xfPlane, kAnchorTolerance);

    // Overlap: centre (3.2, 4) has signedDistance = -3.2 + 3 = -0.2, so the
    // penetration is 0.5 + 0.2 = 0.7 and the contact projection is (3, 4).
    const Manifold overlap = p2::CollideCircleAndPlane(circle, MakeTranslation(3.2f, 4.0f), plane, xfPlane);
    ASSERT_EQ(overlap.pointCount, 1);
    EXPECT_NEAR(overlap.penetration, 0.7f, 1.0e-5f);
    EXPECT_NEAR(overlap.normal.x, 1.0f, kTolerance);
    EXPECT_NEAR(overlap.normal.y, 0.0f, kTolerance);
    ExpectWorldAnchorsAgree(overlap, MakeTranslation(3.2f, 4.0f), xfPlane, kAnchorTolerance);
    // Contact = (3, 4) in world space: (3 - 3.2, 4 - 4) = (-0.2, 0) in the
    // circle's frame, and the origin of the plane's frame (its body origin is
    // exactly the surface point (3, 4)).
    EXPECT_NEAR(overlap.points[0].localPointA.x, -0.2f, 1.0e-5f);
    EXPECT_NEAR(overlap.points[0].localPointA.y, 0.0f, 1.0e-5f);
    EXPECT_NEAR(overlap.points[0].localPointB.x, 0.0f, 1.0e-5f);
    EXPECT_NEAR(overlap.points[0].localPointB.y, 0.0f, 1.0e-5f);
}

TEST(CollidePlaneAndCircle, IsTheFlippedCircleAndPlaneManifold) {
    // The generator is written for (A = circle, B = plane); the plane-first
    // overload must be the exact flip: normal reversed, anchors swapped, flip bit
    // toggled, penetration and point count untouched.
    const PlaneShape plane;
    const Transform xfPlane{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfCircle = MakeTranslation(3.2f, 4.0f); // 0.7 penetration

    const Manifold circleFirst = p2::CollideCircleAndPlane(circle, xfCircle, plane, xfPlane);
    const Manifold planeFirst = p2::CollidePlaneAndCircle(plane, xfPlane, circle, xfCircle);
    ASSERT_EQ(circleFirst.pointCount, 1);
    ASSERT_EQ(planeFirst.pointCount, 1);

    EXPECT_FLOAT_EQ(planeFirst.normal.x, -circleFirst.normal.x);
    EXPECT_FLOAT_EQ(planeFirst.normal.y, -circleFirst.normal.y);
    EXPECT_FLOAT_EQ(planeFirst.penetration, circleFirst.penetration);

    EXPECT_TRUE(planeFirst.points[0].localPointA == circleFirst.points[0].localPointB);
    EXPECT_TRUE(planeFirst.points[0].localPointB == circleFirst.points[0].localPointA);

    // Both use the circle-plane feature identity. `flip` is orientation metadata
    // (Week 4) and is deliberately excluded from Key(), so the contact identity is
    // identical whether the geometry was evaluated as (A,B) or (B,A).
    EXPECT_EQ(planeFirst.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(circleFirst.points[0].id.flip, 0);
    EXPECT_EQ(planeFirst.points[0].id.flip, 1);
    EXPECT_EQ(planeFirst.points[0].id.Key(), circleFirst.points[0].id.Key());

    // The plane-first anchors must agree in world space too.
    ExpectWorldAnchorsAgree(planeFirst, xfPlane, xfCircle, kAnchorTolerance);
}

TEST(FlipManifold, ReversesGeometryAndIsAnInvolution) {
    // Start from a real circle-circle manifold: penetration 0.4, normal (1,0),
    // contact (0.3, 0) in A's frame and (-0.3, 0) in B's frame.
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfA = MakeTranslation(0.0f, 0.0f);
    const Transform xfB = MakeTranslation(0.6f, 0.0f);
    const Manifold original = p2::CollideCircles(circle, xfA, circle, xfB);
    ASSERT_EQ(original.pointCount, 1);

    const Manifold flipped = p2::FlipManifold(original);
    EXPECT_EQ(flipped.pointCount, original.pointCount);
    EXPECT_FLOAT_EQ(flipped.penetration, original.penetration);
    // Exact negation: the normal is stored, not recomputed.
    EXPECT_FLOAT_EQ(flipped.normal.x, -original.normal.x);
    EXPECT_FLOAT_EQ(flipped.normal.y, -original.normal.y);
    // Anchors are exchanged exactly.
    EXPECT_TRUE(flipped.points[0].localPointA == original.points[0].localPointB);
    EXPECT_TRUE(flipped.points[0].localPointB == original.points[0].localPointA);
    EXPECT_EQ(flipped.points[0].id.flip, 1);
    // Flipping toggles the flip metadata but not the contact identity: Key() is
    // flip-independent (Week 4 semantics), so the same physical contact keeps the
    // same Key regardless of the A/B evaluation order.
    EXPECT_EQ(flipped.points[0].id.Key(), original.points[0].id.Key());

    // Flipping twice restores the original manifold, including the Key.
    const Manifold restored = p2::FlipManifold(flipped);
    EXPECT_FLOAT_EQ(restored.normal.x, original.normal.x);
    EXPECT_FLOAT_EQ(restored.normal.y, original.normal.y);
    EXPECT_FLOAT_EQ(restored.penetration, original.penetration);
    EXPECT_TRUE(restored.points[0].localPointA == original.points[0].localPointA);
    EXPECT_TRUE(restored.points[0].localPointB == original.points[0].localPointB);
    EXPECT_EQ(restored.points[0].id.Key(), original.points[0].id.Key());

    // An empty manifold stays empty; only pointCount/penetration are contract.
    const Manifold empty = p2::CollideCircles(circle, xfA, circle, MakeTranslation(5.0f, 0.0f));
    ASSERT_EQ(empty.pointCount, 0);
    const Manifold flippedEmpty = p2::FlipManifold(empty);
    EXPECT_EQ(flippedEmpty.pointCount, 0);
    EXPECT_FLOAT_EQ(flippedEmpty.penetration, 0.0f);
}

TEST(FlipManifold, SwapsEveryPointOfATwoPointManifold) {
    // Circle paths produce one point, but Week 4's clipping produces two; the
    // flip must swap both points (the loop, not just points[0]).
    Manifold twoPoint;
    twoPoint.normal = Vec2{0.0f, 1.0f};
    twoPoint.penetration = 0.25f;
    twoPoint.pointCount = 2;
    for (int32_t i = 0; i < 2; ++i) {
        const float fi = static_cast<float>(i);
        twoPoint.points[i].localPointA = Vec2{1.0f + fi, 2.0f};
        twoPoint.points[i].localPointB = Vec2{-1.0f, 3.0f + fi};
        twoPoint.points[i].id.referenceEdge = static_cast<uint8_t>(i + 1);
        twoPoint.points[i].id.incidentVertex = static_cast<uint8_t>(i + 2);
        twoPoint.points[i].id.type = p2::kContactTypeFaceFace;
    }

    const Manifold flipped = p2::FlipManifold(twoPoint);
    ASSERT_EQ(flipped.pointCount, 2);
    for (int32_t i = 0; i < 2; ++i) {
        EXPECT_TRUE(flipped.points[i].localPointA == twoPoint.points[i].localPointB) << "point " << i;
        EXPECT_TRUE(flipped.points[i].localPointB == twoPoint.points[i].localPointA) << "point " << i;
        // Feature indices and the type survive the flip; only the flip bit toggles.
        EXPECT_EQ(flipped.points[i].id.referenceEdge, twoPoint.points[i].id.referenceEdge);
        EXPECT_EQ(flipped.points[i].id.incidentVertex, twoPoint.points[i].id.incidentVertex);
        EXPECT_EQ(flipped.points[i].id.type, twoPoint.points[i].id.type);
        EXPECT_EQ(flipped.points[i].id.flip, 1);
    }
}

TEST(ContactID, CircleFeatureTypesAndKeyPacking) {
    // Key layout: bits 8-9 = type. `flip` is Week-4 metadata and is NOT hashed
    // into the identity (the same physical contact must keep its Key when A/B are
    // swapped, P07 D3/D5).
    const CircleShape circle = MakeCircle(0.5f);
    const PlaneShape plane;

    // Circle-circle: type 2 -> Key = 2 << 8 = 0x0200.
    const Manifold circleCircle =
        p2::CollideCircles(circle, MakeTranslation(0.0f, 0.0f), circle, MakeTranslation(0.6f, 0.0f));
    ASSERT_EQ(circleCircle.pointCount, 1);
    EXPECT_EQ(circleCircle.points[0].id.type, p2::kContactTypeCircleCircle);
    EXPECT_EQ(circleCircle.points[0].id.Key(), 0x0200);

    // Circle-plane: type 3 -> Key = 3 << 8 = 0x0300.
    const Transform xfCircle = MakeTranslation(0.0f, 0.2f);
    const Manifold circlePlane = p2::CollideCircleAndPlane(circle, xfCircle, plane, Transform{});
    ASSERT_EQ(circlePlane.pointCount, 1);
    EXPECT_EQ(circlePlane.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(circlePlane.points[0].id.Key(), 0x0300);

    // Plane-circle: same identity, flip metadata set, same Key (0x0300).
    const Manifold planeCircle = p2::CollidePlaneAndCircle(plane, Transform{}, circle, xfCircle);
    ASSERT_EQ(planeCircle.pointCount, 1);
    EXPECT_EQ(planeCircle.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(planeCircle.points[0].id.flip, 1);
    EXPECT_EQ(planeCircle.points[0].id.Key(), 0x0300);
}

TEST(ContactID, KeyIsStableAcrossFramesWhileTheContactPersists) {
    // Circles have no discrete features, so the Key must not change while the
    // pair stays in contact: this is the precondition for warm starting (Week 5).
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfA = MakeTranslation(0.0f, 0.0f);

    uint16_t previousKey = 0;
    for (int frame = 0; frame < 10; ++frame) {
        // Slide B sideways, from x = 0.6 to x = 0.51: still overlapping (distance
        // 0.6..0.51 < radius sum 1.0) but at a different relative position.
        const float x = 0.6f - 0.01f * static_cast<float>(frame);
        const Manifold manifold = p2::CollideCircles(circle, xfA, circle, MakeTranslation(x, 0.0f));
        ASSERT_EQ(manifold.pointCount, 1) << "frame " << frame;
        const uint16_t key = manifold.points[0].id.Key();
        if (frame > 0) {
            EXPECT_EQ(key, previousKey) << "frame " << frame;
        }
        previousKey = key;
    }
}

TEST(ContactID, KeyIsIndependentOfFlipMetadata) {
    // Week 4 semantics: Key() is the physical contact identity and must not encode
    // the `flip` metadata. Otherwise evaluating a pair as (A,B) or (B,A) would
    // change the identity, and warm starting (Week 5) would lose the contact.
    p2::ContactID original;
    original.referenceEdge = 3;
    original.incidentVertex = 5;
    original.type = p2::kContactTypeFaceFace;
    original.flip = 0;

    p2::ContactID flipped = original;
    flipped.flip = 1;

    // Key layout: referenceEdge bits 0-3, incidentVertex bits 4-7, type bits 8-9.
    //   3 | (5 << 4) | (1 << 8) = 3 | 0x50 | 0x100 = 0x153.
    EXPECT_EQ(original.Key(), 0x0153);
    EXPECT_EQ(flipped.Key(), original.Key());
    // The full comparison still distinguishes the orientation metadata.
    EXPECT_NE(flipped, original);
}

TEST(ContactPoints, WorldAnchorsAgreeForArbitraryTransforms) {
    // Rotated local centres and non-axis-aligned body transforms make the
    // inverse-transform path non-trivial. Both paths are checked as properties:
    // one contact point, unit normal, and the two stored anchors must reconstruct
    // the same world point within 1e-5 (the solver recomputes lever arms from
    // them every frame).
    CircleShape circleA = MakeCircle(0.7f);
    circleA.localCenter = Vec2{0.2f, -0.1f};
    const CircleShape circleB = MakeCircle(0.4f); // local centre at the origin

    for (int i = 1; i <= 20; ++i) {
        const float fi = static_cast<float>(i);
        const Transform xfA{Vec2{0.37f * fi, -0.21f * fi}, Rot(0.13f * fi)};
        const Vec2 centerA = p2::Mul(xfA, circleA.localCenter);

        // Place B's centre 0.3 to the right and 0.1 below A's centre: distance
        // sqrt(0.1) = 0.3162 < 0.7 + 0.4 = 1.1, so every iteration overlaps.
        const Transform xfB{Vec2{centerA.x + 0.3f, centerA.y - 0.1f}, Rot(-0.07f * fi)};
        const Manifold manifold = p2::CollideCircles(circleA, xfA, circleB, xfB);

        ASSERT_EQ(manifold.pointCount, 1) << "iteration " << i;
        EXPECT_GT(manifold.penetration, 0.0f);
        EXPECT_NEAR(p2::Length(manifold.normal), 1.0f, kTolerance);
        ExpectWorldAnchorsAgree(manifold, xfA, xfB, kAnchorTolerance);
    }

    // Circle-plane property check: put the circle centre at a fixed signed
    // distance 0.25 from the rotated plane surface plus a tangential offset, so
    // the projection onto the plane is not the trivial axis case.
    const PlaneShape plane;
    CircleShape centeredCircle = MakeCircle(0.7f); // local centre is the origin
    for (int i = 1; i <= 20; ++i) {
        const float fi = static_cast<float>(i);
        const Transform xfPlane{Vec2{0.11f * fi, -0.05f * fi}, Rot(0.09f * fi)};
        const Vec2 worldNormal = p2::ComputePlaneWorldNormal(plane, xfPlane);
        const float worldOffset = p2::ComputePlaneWorldOffset(plane, xfPlane);
        const Vec2 tangent = p2::Skew(worldNormal);
        const Vec2 center = worldNormal * (worldOffset + 0.25f) + tangent * 0.31f;
        const Transform xfCircle{center, Rot(-0.17f * fi)};
        const Manifold manifold = p2::CollideCircleAndPlane(centeredCircle, xfCircle, plane, xfPlane);

        ASSERT_EQ(manifold.pointCount, 1) << "iteration " << i;
        EXPECT_NEAR(manifold.penetration, centeredCircle.radius - 0.25f, 1.0e-5f);
        EXPECT_NEAR(p2::Length(manifold.normal), 1.0f, kTolerance);
        ExpectWorldAnchorsAgree(manifold, xfCircle, xfPlane, kAnchorTolerance);
    }
}
