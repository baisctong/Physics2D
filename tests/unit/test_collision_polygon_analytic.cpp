// Unit tests for the Week 4 analytic polygon collision paths (design document
// Week 4 D1 + prompt P06/P04): circle vs convex polygon and polygon vs infinite
// plane.
//
// The test geometry is a box built with SetPolygonAsBox (canonical CCW order
// starting at (+hx,-hy)), so every normal, edge index, penetration and anchor
// below is derived by hand in the comments. The flipping/identity checks also pin
// the Week-4 ContactID rules: the Key is orientation-independent, while `flip` is
// metadata.

#include <gtest/gtest.h>

#include "physics2d/Collision/Analytic.h"
#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using p2::CircleShape;
using p2::Manifold;
using p2::PlaneShape;
using p2::PolygonShape;
using p2::Rot;
using p2::Transform;
using p2::Vec2;

// All inputs are exactly representable; 1e-6 covers the float rounding of the
// square root and the rotation multiplications.
constexpr float kTolerance = 1.0e-6f;

// Anchors are inverse-transformed and re-transformed; a few ulps of the coordinate
// magnitude accumulate, so the reconstruction check allows 1e-5 (task requirement).
constexpr float kAnchorTolerance = 1.0e-5f;

PolygonShape MakeSquare(float halfExtent = 1.0f) {
    PolygonShape shape;
    const bool ok = p2::SetPolygonAsBox(shape, halfExtent, halfExtent);
    EXPECT_TRUE(ok);
    return shape;
}

CircleShape MakeCircle(float radius = 0.5f) {
    CircleShape circle;
    circle.radius = radius;
    return circle;
}

Transform Translate(float x, float y) {
    return Transform{Vec2{x, y}, Rot()};
}

void ExpectNearVec2(Vec2 actual, Vec2 expected, float tolerance) {
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
}

// The solver re-derives the world anchors from the two local frames every frame,
// so both stored anchors must describe the same world point.
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

} // namespace

// ---------------------------------------------------------------------------
// Circle vs polygon: Voronoi-region selection of the reference feature.
// ---------------------------------------------------------------------------

TEST(CollideCircleAndPolygon, FaceContactUsesEdgeVoronoiFeature) {
    const PolygonShape square = MakeSquare(); // 2 x 2 box, edges: 0 right, 1 top, 2 left, 3 bottom
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPolygon = Transform{};

    // Tangent to the right edge (edge 0, outward normal (+1,0)) with the centre at
    // (1.5, 0): separation = 1.5 - 1 = 0.5 = r, so penetration = 0. The circle ->
    // polygon normal is (-1,0) and the anchor is the edge projection (1, 0).
    const Manifold tangent = p2::CollideCircleAndPolygon(circle, Translate(1.5f, 0.0f), square, xfPolygon);
    ASSERT_EQ(tangent.pointCount, 1);
    EXPECT_NEAR(tangent.penetration, 0.0f, kTolerance);
    EXPECT_NEAR(tangent.normal.x, -1.0f, kTolerance);
    EXPECT_NEAR(tangent.normal.y, 0.0f, kTolerance);
    EXPECT_EQ(tangent.points[0].id.type, p2::kContactTypeVertexFace);
    EXPECT_EQ(tangent.points[0].id.referenceEdge, 0); // nearest polygon edge
    EXPECT_EQ(tangent.points[0].id.incidentVertex, 0);
    // The circle body is at (1.5, 0) and the anchor is world (1, 0).
    ExpectNearVec2(tangent.points[0].localPointA, Vec2{1.0f - 1.5f, 0.0f}, kTolerance);
    ExpectNearVec2(tangent.points[0].localPointB, Vec2{1.0f, 0.0f}, kTolerance);
    ExpectWorldAnchorsAgree(tangent, Translate(1.5f, 0.0f), xfPolygon, kAnchorTolerance);

    // Overlap: centre (1.3, 0) => penetration = r - (1.3 - 1) = 0.5 - 0.3 = 0.2.
    const Manifold overlap = p2::CollideCircleAndPolygon(circle, Translate(1.3f, 0.0f), square, xfPolygon);
    ASSERT_EQ(overlap.pointCount, 1);
    EXPECT_NEAR(overlap.penetration, 0.2f, kTolerance);
    EXPECT_NEAR(overlap.normal.x, -1.0f, kTolerance);
    EXPECT_NEAR(overlap.normal.y, 0.0f, kTolerance);
    EXPECT_EQ(overlap.points[0].id.referenceEdge, 0);
    ExpectWorldAnchorsAgree(overlap, Translate(1.3f, 0.0f), xfPolygon, kAnchorTolerance);

    // Left face is edge 2 (v2 -> v3, outward normal (-1,0)); a circle at (-1.3, 0)
    // therefore sees normal (+1,0) and referenceEdge 2.
    const Manifold left = p2::CollideCircleAndPolygon(circle, Translate(-1.3f, 0.0f), square, xfPolygon);
    ASSERT_EQ(left.pointCount, 1);
    EXPECT_NEAR(left.penetration, 0.2f, kTolerance);
    EXPECT_NEAR(left.normal.x, 1.0f, kTolerance);
    EXPECT_NEAR(left.normal.y, 0.0f, kTolerance);
    EXPECT_EQ(left.points[0].id.referenceEdge, 2);
}

TEST(CollideCircleAndPolygon, CornerContactUsesVertexVoronoiFeature) {
    const PolygonShape square = MakeSquare();
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPolygon = Transform{};

    // Centre (1.3, 1.3) is diagonally outside vertex v1 = (1,1). The projections on
    // edges 0 and 1 are both beyond the segment, so the vertex Voronoi region wins:
    //   distance = sqrt(0.3^2 + 0.3^2) = 0.42426407
    //   penetration = 0.5 - 0.42426407 = 0.07573593
    //   normal = (v1 - c)/|v1 - c| = (-0.70710678, -0.70710678)
    const Manifold corner = p2::CollideCircleAndPolygon(circle, Translate(1.3f, 1.3f), square, xfPolygon);
    ASSERT_EQ(corner.pointCount, 1);
    const float expectedDistance = std::sqrt(0.3f * 0.3f + 0.3f * 0.3f);
    EXPECT_NEAR(corner.penetration, 0.5f - expectedDistance, kTolerance);
    EXPECT_NEAR(corner.normal.x, -0.70710678f, 1.0e-5f);
    EXPECT_NEAR(corner.normal.y, -0.70710678f, 1.0e-5f);
    EXPECT_EQ(corner.points[0].id.type, p2::kContactTypeVertexFace);
    // ContactID.h documents the dual use: referenceEdge holds the vertex index in
    // the vertex Voronoi region. v1 = (1,1) is vertex index 1.
    EXPECT_EQ(corner.points[0].id.referenceEdge, 1);
    EXPECT_EQ(corner.points[0].id.incidentVertex, 0);
    ExpectNearVec2(corner.points[0].localPointB, Vec2{1.0f, 1.0f}, kTolerance);
    ExpectWorldAnchorsAgree(corner, Translate(1.3f, 1.3f), xfPolygon, kAnchorTolerance);

    // Near-corner touch: centre = v1 + ((r - 1e-3)/sqrt(2)) * (1,1), so the centre
    // sits 1 mm inside the radius and the penetration is ~1e-3. (An exactly tangent
    // corner is avoided because sqrt rounding could flip the distance across the
    // radius and turn the contact into a separation; the face case above already
    // pins exact tangency with exactly representable coordinates.)
    const float offset = (0.5f - 1.0e-3f) / std::sqrt(2.0f);
    const Manifold cornerTouch =
        p2::CollideCircleAndPolygon(circle, Translate(1.0f + offset, 1.0f + offset), square, xfPolygon);
    ASSERT_EQ(cornerTouch.pointCount, 1);
    EXPECT_NEAR(cornerTouch.penetration, 1.0e-3f, 1.0e-5f);
    EXPECT_NEAR(cornerTouch.normal.x, -std::sqrt(0.5f), 1.0e-5f);
    EXPECT_NEAR(cornerTouch.normal.y, -std::sqrt(0.5f), 1.0e-5f);

    // The edge half-planes are within the radius but the corner is not: the circle
    // sits diagonally outside the Voronoi regions, so there is no contact.
    const Manifold diagonalGap =
        p2::CollideCircleAndPolygon(circle, Translate(1.5f, 1.5f), square, xfPolygon);
    EXPECT_EQ(diagonalGap.pointCount, 0);
}

TEST(CollideCircleAndPolygon, SeparatedAndContainedCases) {
    const PolygonShape square = MakeSquare();
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPolygon = Transform{};

    // Clearly separated (right of the box, gap 1.5): no contact.
    const Manifold separated = p2::CollideCircleAndPolygon(circle, Translate(3.0f, 0.0f), square, xfPolygon);
    EXPECT_EQ(separated.pointCount, 0);
    EXPECT_FALSE(separated.IsTouching());

    // Degenerate case: the centre is inside the polygon. All edge separations are
    // -1, so penetration = r + 1 = 1.5; the deterministic normal is the negated
    // outward normal of the first (maximum, all-tied) edge, (-1,0), which makes the
    // solver push the circle OUT along +x. Anchor = projection onto that edge = (1,0).
    const Manifold contained = p2::CollideCircleAndPolygon(circle, Translate(0.0f, 0.0f), square, xfPolygon);
    ASSERT_EQ(contained.pointCount, 1);
    EXPECT_NEAR(contained.penetration, 1.5f, kTolerance);
    EXPECT_NEAR(contained.normal.x, -1.0f, kTolerance);
    EXPECT_NEAR(contained.normal.y, 0.0f, kTolerance);
    EXPECT_EQ(contained.points[0].id.referenceEdge, 0);
    ExpectNearVec2(contained.points[0].localPointB, Vec2{1.0f, 0.0f}, kTolerance);
    ExpectWorldAnchorsAgree(contained, Translate(0.0f, 0.0f), xfPolygon, kAnchorTolerance);
    EXPECT_TRUE(std::isfinite(contained.penetration));
    EXPECT_TRUE(std::isfinite(contained.normal.x));
}

TEST(CollidePolygonAndCircle, IsTheFlippedCircleAndPolygonManifold) {
    const PolygonShape square = MakeSquare();
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPolygon = Transform{};
    const Transform xfCircle = Translate(1.3f, 0.0f);

    const Manifold circleFirst = p2::CollideCircleAndPolygon(circle, xfCircle, square, xfPolygon);
    const Manifold polygonFirst = p2::CollidePolygonAndCircle(square, xfPolygon, circle, xfCircle);
    ASSERT_EQ(circleFirst.pointCount, 1);
    ASSERT_EQ(polygonFirst.pointCount, 1);

    EXPECT_FLOAT_EQ(polygonFirst.normal.x, -circleFirst.normal.x);
    EXPECT_FLOAT_EQ(polygonFirst.normal.y, -circleFirst.normal.y);
    EXPECT_FLOAT_EQ(polygonFirst.penetration, circleFirst.penetration);
    EXPECT_TRUE(polygonFirst.points[0].localPointA == circleFirst.points[0].localPointB);
    EXPECT_TRUE(polygonFirst.points[0].localPointB == circleFirst.points[0].localPointA);

    // `flip` records the A/B evaluation order; the Key is the orientation-independent
    // identity, so both directions produce the same Key.
    EXPECT_EQ(polygonFirst.points[0].id.type, circleFirst.points[0].id.type);
    EXPECT_EQ(polygonFirst.points[0].id.referenceEdge, circleFirst.points[0].id.referenceEdge);
    EXPECT_EQ(circleFirst.points[0].id.flip, 0);
    EXPECT_EQ(polygonFirst.points[0].id.flip, 1);
    EXPECT_EQ(polygonFirst.points[0].id.Key(), circleFirst.points[0].id.Key());

    ExpectWorldAnchorsAgree(polygonFirst, xfPolygon, xfCircle, kAnchorTolerance);
}

TEST(CollideCircleAndPolygon, KeyChangesOnlyAtFeatureTransitions) {
    // Slide the circle along the right face at x = 1.3 from y = -1.3 to +1.3.
    // Projection on edge 0 is (y + 1)/2: below 0 => vertex 0, in [0,1] => edge 0,
    // above 1 => vertex 1. The circle stays in contact throughout (straight
    // separation 0.3 < r and corner distance 0.424 < r).
    const PolygonShape square = MakeSquare();
    const CircleShape circle = MakeCircle(0.5f);
    const Transform xfPolygon = Transform{};
    constexpr int32_t kFrames = 100;

    int32_t previousRegion = -1;
    uint16_t previousKey = 0;
    int32_t regionChanges = 0;
    int32_t keyChanges = 0;
    int32_t lastKeyChangeFrame = -1;

    for (int32_t frame = 0; frame < kFrames; ++frame) {
        const float y = -1.3f + 2.6f * static_cast<float>(frame) / static_cast<float>(kFrames - 1);
        const Manifold manifold = p2::CollideCircleAndPolygon(circle, Translate(1.3f, y), square, xfPolygon);
        ASSERT_EQ(manifold.pointCount, 1) << "frame " << frame;

        // Independent classification of the expected Voronoi region.
        const int32_t region = (y < -1.0f) ? 0 : (y > 1.0f ? 2 : 1);
        const uint16_t key = manifold.points[0].id.Key();
        if (previousRegion >= 0) {
            if (region != previousRegion) {
                ++regionChanges;
                std::cout << "[circle-polygon] feature transition at frame " << frame << " (y=" << y
                          << ") key=0x" << std::hex << key << std::dec << "\n";
            }
            if (key != previousKey) {
                ++keyChanges;
                lastKeyChangeFrame = frame;
                // A Key change may only happen on a frame where the physical
                // feature changed - that is what makes warm starting possible.
                EXPECT_NE(region, previousRegion) << "frame " << frame;
                std::cout << "[circle-polygon] key change at frame " << frame << " (y=" << y << ")\n";
            }
        }
        if (region == previousRegion) {
            // While the Voronoi feature is unchanged the identity must be stable.
            EXPECT_EQ(key, previousKey) << "frame " << frame;
        }
        previousRegion = region;
        previousKey = key;
    }

    // Two physical feature transitions on this slide (vertex0->edge, edge->vertex1).
    EXPECT_EQ(regionChanges, 2);
    // Week-4 encodes edge 0 and vertex 0 both as referenceEdge 0 (the ContactID
    // header documents the shared field), so only the edge -> vertex 1 transition
    // changes the Key. It is exactly one change, on a feature-transition frame.
    EXPECT_EQ(keyChanges, 1);
    EXPECT_GT(lastKeyChangeFrame, 0);
    EXPECT_LT(lastKeyChangeFrame, kFrames);
}

// ---------------------------------------------------------------------------
// Polygon vs infinite plane: depth collection and the two-point support that
// keeps a resting box stable.
// ---------------------------------------------------------------------------

TEST(CollidePolygonAndPlane, FlatBoxOnPlaneHasTwoPointSupport) {
    const PlaneShape plane;                // ground: normal (0,1), offset 0
    const PolygonShape box = MakeSquare(); // 2 x 2 box
    const Transform xfPlane = Transform{};

    // Box centre at y = 1: the bottom vertices v0=(1,-1) and v3=(-1,-1) sit exactly
    // on the plane (signed distance 0). A flat face contact must produce TWO points:
    // that is what gives a resting box a stable support.
    const Transform resting = Translate(0.0f, 1.0f);
    const Manifold manifold = p2::CollidePolygonAndPlane(box, resting, plane, xfPlane);
    ASSERT_EQ(manifold.pointCount, 2);
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f); // exactly tangent
    EXPECT_FLOAT_EQ(manifold.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(manifold.normal.y, -1.0f);

    // Points are emitted in ascending vertex index: v0 then v3. Both belong to the
    // bottom face, edge 3 (v3 -> v0), so referenceEdge == 3 for both.
    EXPECT_EQ(manifold.points[0].id.type, p2::kContactTypeCirclePlane);
    EXPECT_EQ(manifold.points[0].id.incidentVertex, 0);
    EXPECT_EQ(manifold.points[1].id.incidentVertex, 3);
    EXPECT_EQ(manifold.points[0].id.referenceEdge, 3);
    EXPECT_EQ(manifold.points[1].id.referenceEdge, 3);
    // Key = type 3 (0x300) | referenceEdge | (vertex << 4):
    //   v0 -> 0x303, v3 -> 0x333.
    EXPECT_EQ(manifold.points[0].id.Key(), 0x0303);
    EXPECT_EQ(manifold.points[1].id.Key(), 0x0333);

    // Anchors are the vertex projections on the plane: world (1,0) and (-1,0);
    // in the resting box's local frame that is (1,-1) and (-1,-1).
    ExpectNearVec2(manifold.points[0].localPointA, Vec2{1.0f, -1.0f}, kTolerance);
    ExpectNearVec2(manifold.points[1].localPointA, Vec2{-1.0f, -1.0f}, kTolerance);
    ExpectNearVec2(manifold.points[0].localPointB, Vec2{1.0f, 0.0f}, kTolerance);
    ExpectWorldAnchorsAgree(manifold, resting, xfPlane, kAnchorTolerance);

    // Penetrating by 0.1: centre lowered to y = 0.9. Penetration is the deepest
    // vertex depth (-min signedDistance = 0.1), anchors stay on the surface y = 0,
    // so the local anchors are (1,-0.9) and (-1,-0.9).
    const Transform overlapping = Translate(0.0f, 0.9f);
    const Manifold overlap = p2::CollidePolygonAndPlane(box, overlapping, plane, xfPlane);
    ASSERT_EQ(overlap.pointCount, 2);
    EXPECT_NEAR(overlap.penetration, 0.1f, kTolerance);
    ExpectNearVec2(overlap.points[0].localPointA, Vec2{1.0f, -0.9f}, kTolerance);
    ExpectNearVec2(overlap.points[0].localPointB, Vec2{1.0f, 0.0f}, kTolerance);
    ExpectWorldAnchorsAgree(overlap, overlapping, xfPlane, kAnchorTolerance);
}

TEST(CollidePolygonAndPlane, RotatedBoxCornerGivesSinglePoint) {
    const PlaneShape plane;
    const PolygonShape box = MakeSquare();
    const Transform xfPlane = Transform{};

    // Box rotated 45 degrees: its corners lie at (+-sqrt(2), 0) and (0, +-sqrt(2)).
    // Sinking the centre 0.1 below the (0, sqrt(2)) position leaves exactly one
    // corner (v3) behind the plane, so a single contact point must be produced (a
    // corner cannot provide the two-point support of a flat face).
    const float sqrt2 = std::sqrt(2.0f);
    const Transform cornerDown{Vec2{0.0f, sqrt2 - 0.1f}, Rot(p2::kPi / 4.0f)};
    const Manifold overlap = p2::CollidePolygonAndPlane(box, cornerDown, plane, xfPlane);
    ASSERT_EQ(overlap.pointCount, 1);
    EXPECT_NEAR(overlap.penetration, 0.1f, 1.0e-5f);
    EXPECT_NEAR(overlap.normal.x, 0.0f, kTolerance);
    EXPECT_NEAR(overlap.normal.y, -1.0f, kTolerance);
    EXPECT_EQ(overlap.points[0].id.incidentVertex, 3); // v3 is the lowest corner
    // The feature edge is one of v3's two adjacent edges (2 or 3), whichever faces
    // the plane more directly; both are deterministic projections of the geometry.
    EXPECT_TRUE(overlap.points[0].id.referenceEdge == 2 || overlap.points[0].id.referenceEdge == 3);
    ExpectWorldAnchorsAgree(overlap, cornerDown, xfPlane, kAnchorTolerance);

    // Deeper corner contact keeps the same single feature and vertex index.
    const Transform deeper{Vec2{0.0f, sqrt2 - 0.25f}, Rot(p2::kPi / 4.0f)};
    const Manifold deeperManifold = p2::CollidePolygonAndPlane(box, deeper, plane, xfPlane);
    ASSERT_EQ(deeperManifold.pointCount, 1);
    EXPECT_NEAR(deeperManifold.penetration, 0.25f, 1.0e-5f);
    EXPECT_EQ(deeperManifold.points[0].id.incidentVertex, 3);
    ExpectWorldAnchorsAgree(deeperManifold, deeper, xfPlane, kAnchorTolerance);
}

TEST(CollidePolygonAndPlane, SeparatedAndBuriedCases) {
    const PlaneShape plane;
    const PolygonShape box = MakeSquare();
    const Transform xfPlane = Transform{};

    // Clearly above the plane: every vertex has signedDistance > 0 -> no contact.
    const Manifold separated = p2::CollidePolygonAndPlane(box, Translate(0.0f, 3.0f), plane, xfPlane);
    EXPECT_EQ(separated.pointCount, 0);

    // Fully buried with centre at y = -5: vertex depths are
    //   v0, v3 at world y = -6 -> depth 6
    //   v1, v2 at world y = -4 -> depth 4
    // so the two deepest vertices (0 and 3) become the contact points and the
    // penetration is the deepest depth 6.
    const Manifold buried = p2::CollidePolygonAndPlane(box, Translate(0.0f, -5.0f), plane, xfPlane);
    ASSERT_EQ(buried.pointCount, 2);
    EXPECT_NEAR(buried.penetration, 6.0f, kTolerance);
    EXPECT_EQ(buried.points[0].id.incidentVertex, 0);
    EXPECT_EQ(buried.points[1].id.incidentVertex, 3);
    ExpectWorldAnchorsAgree(buried, Translate(0.0f, -5.0f), xfPlane, kAnchorTolerance);
}

TEST(CollidePlaneAndPolygon, IsTheFlippedPolygonAndPlaneManifold) {
    const PlaneShape plane;
    const PolygonShape box = MakeSquare();
    const Transform xfPlane = Transform{};
    const Transform xfPolygon = Translate(0.0f, 0.9f); // 0.1 penetration

    const Manifold polygonFirst = p2::CollidePolygonAndPlane(box, xfPolygon, plane, xfPlane);
    const Manifold planeFirst = p2::CollidePlaneAndPolygon(plane, xfPlane, box, xfPolygon);
    ASSERT_EQ(polygonFirst.pointCount, 2);
    ASSERT_EQ(planeFirst.pointCount, 2);

    EXPECT_FLOAT_EQ(planeFirst.normal.x, -polygonFirst.normal.x);
    EXPECT_FLOAT_EQ(planeFirst.normal.y, -polygonFirst.normal.y);
    EXPECT_FLOAT_EQ(planeFirst.penetration, polygonFirst.penetration);
    for (int32_t i = 0; i < 2; ++i) {
        EXPECT_TRUE(planeFirst.points[i].localPointA == polygonFirst.points[i].localPointB);
        EXPECT_TRUE(planeFirst.points[i].localPointB == polygonFirst.points[i].localPointA);
        // The identity is flip-independent: same Key, different metadata.
        EXPECT_EQ(planeFirst.points[i].id.Key(), polygonFirst.points[i].id.Key());
        EXPECT_EQ(polygonFirst.points[i].id.flip, 0);
        EXPECT_EQ(planeFirst.points[i].id.flip, 1);
        EXPECT_EQ(planeFirst.points[i].id.incidentVertex, polygonFirst.points[i].id.incidentVertex);
    }
    ExpectWorldAnchorsAgree(planeFirst, xfPlane, xfPolygon, kAnchorTolerance);
}

TEST(CollidePolygonAndPlane, ContactIdsStayStableWhileSliding) {
    // Sliding parallel to the plane cannot change the polygon-plane feature set:
    // the two bottom vertices and their supporting edge stay the same, so the
    // ContactID keys must remain constant frame to frame (warm-start precondition).
    const PlaneShape plane;
    const PolygonShape box = MakeSquare();
    const Transform xfPlane = Transform{};

    uint16_t firstKey = 0;
    uint16_t secondKey = 0;
    for (int32_t frame = 0; frame < 12; ++frame) {
        const float x = -3.0f + 0.5f * static_cast<float>(frame);
        const Manifold manifold = p2::CollidePolygonAndPlane(box, Translate(x, 0.9f), plane, xfPlane);
        ASSERT_EQ(manifold.pointCount, 2) << "frame " << frame;
        EXPECT_EQ(manifold.points[0].id.incidentVertex, 0);
        EXPECT_EQ(manifold.points[1].id.incidentVertex, 3);
        if (frame == 0) {
            firstKey = manifold.points[0].id.Key();
            secondKey = manifold.points[1].id.Key();
        } else {
            EXPECT_EQ(manifold.points[0].id.Key(), firstKey) << "frame " << frame;
            EXPECT_EQ(manifold.points[1].id.Key(), secondKey) << "frame " << frame;
        }
    }
}

// ---------------------------------------------------------------------------
// Anchor reconstruction under non-trivial transforms (the solver invariant).
// ---------------------------------------------------------------------------

TEST(CollideCircleAndPolygon, WorldAnchorsAgreeForTransformedPolygon) {
    PolygonShape box;
    ASSERT_TRUE(p2::SetPolygonAsBox(box, 1.0f, 0.5f));
    const CircleShape circle = MakeCircle(0.4f);

    for (int32_t i = 1; i <= 10; ++i) {
        const float fi = static_cast<float>(i);
        const Transform xfPolygon{Vec2{0.31f * fi, -0.17f * fi}, Rot(0.23f * fi)};

        // Put the circle centre on the outward normal of edge 0, with a 0.1 m
        // overlap: contact is guaranteed and the anchor is not the trivial axis case.
        const Vec2 edgeMid =
            (p2::Mul(xfPolygon, box.vertices[0]) + p2::Mul(xfPolygon, box.vertices[1])) * 0.5f;
        const Vec2 worldEdgeNormal = p2::Mul(xfPolygon.q, box.normals[0]);
        const Vec2 center = edgeMid + worldEdgeNormal * (circle.radius - 0.1f);
        const Transform xfCircle{center, Rot(-0.11f * fi)};

        const Manifold manifold = p2::CollideCircleAndPolygon(circle, xfCircle, box, xfPolygon);
        ASSERT_EQ(manifold.pointCount, 1) << "iteration " << i;
        EXPECT_NEAR(manifold.penetration, 0.1f, 1.0e-5f);
        EXPECT_NEAR(p2::Length(manifold.normal), 1.0f, kTolerance);
        ExpectWorldAnchorsAgree(manifold, xfCircle, xfPolygon, kAnchorTolerance);
    }
}

TEST(CollidePolygonAndPlane, WorldAnchorsAgreeForTransformedPlane) {
    const PolygonShape box = MakeSquare();
    const PlaneShape plane;

    for (int32_t i = 1; i <= 10; ++i) {
        const float fi = static_cast<float>(i);
        const Transform xfPlane{Vec2{0.13f * fi, -0.07f * fi}, Rot(0.19f * fi)};
        const Vec2 worldNormal = p2::ComputePlaneWorldNormal(plane, xfPlane);
        const float worldOffset = p2::ComputePlaneWorldOffset(plane, xfPlane);
        const Vec2 tangent = p2::Skew(worldNormal);

        // Align the box's local +y with the world plane normal, then sink it 0.05 m:
        // both bottom vertices are behind the plane by 0.05 (two-point support).
        const float angle = std::atan2(worldNormal.y, worldNormal.x) - p2::kHalfPi;
        const Vec2 center = worldNormal * (worldOffset + 1.0f - 0.05f) + tangent * 0.37f;
        const Transform xfPolygon{center, Rot(angle)};

        const Manifold manifold = p2::CollidePolygonAndPlane(box, xfPolygon, plane, xfPlane);
        ASSERT_EQ(manifold.pointCount, 2) << "iteration " << i;
        EXPECT_NEAR(manifold.penetration, 0.05f, 1.0e-5f);
        EXPECT_NEAR(p2::Length(manifold.normal), 1.0f, kTolerance);
        ExpectWorldAnchorsAgree(manifold, xfPolygon, xfPlane, kAnchorTolerance);
    }
}
