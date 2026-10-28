// Unit tests for p2::SAT + p2::CollidePolygons (Week 4 D2/D3/D4/D5, P07).
//
// Coverage:
//   - FindMaxSeparation: hand-computed separation/edge index for translated and
//     rotated boxes, plus the degenerate-shape contract;
//   - the nine relative poses required by D2 (4 separated directions, edge-edge
//     face-face, vertex-edge, full containment, exact tangency, parallel faces);
//   - face-face -> 2 points and vertex-face -> 1 point manifolds;
//   - A/B swap: normal negated, identical penetration and an identical
//     ContactID::Key() set (flip is metadata only);
//   - SAT cache hysteresis: near-tied axes keep the previous reference face, a
//     clearly different axis is never overridden, and cache on/off leaves the
//     physical result unchanged (normal direction + penetration);
//   - degenerate geometry (1e-3 x 10 strip, 1e-4 box, collinear vertex, malformed
//     shape) never produces NaN;
//   - a rotating box on the ground: ContactID key statistics over 0..90 degrees.
//
// PolygonShape.cpp is implemented in parallel, so the fixtures below fill the
// PolygonShape storage contract from PolygonShape.h directly instead of calling
// SetPolygonAsBox(): CCW vertices starting at (+hx,-hy), unit outward normals in
// `normals[edge]` and the centroid in the local frame. This is exactly the
// documented SetPolygonAsBox ordering, so the edge indices (and therefore the
// expected ContactIDs) match what the engine produces.

#include <gtest/gtest.h>

#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Collision/SAT.h"
#include "physics2d/Collision/SATCache.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using p2::Manifold;
using p2::PolygonShape;
using p2::Rot;
using p2::SATCache;
using p2::SeparationResult;
using p2::Transform;
using p2::Vec2;

// Contact anchors and penetrations are sums of a few float operations on unit
// scale numbers, so 1e-5 m (0.01 mm) is a generous bound that still catches a
// systematic sign or half-depth error.
constexpr float kTolerance = 1.0e-5f;
// Direction/unit-length checks: a float unit vector is within a few 1e-7 of
// length 1 after rotation, so 1e-4 is loose and still meaningful.
constexpr float kDirectionTolerance = 1.0e-4f;

// ---------------------------------------------------------------------------
// fixtures
// ---------------------------------------------------------------------------

PolygonShape MakeBox(float halfWidth, float halfHeight) {
    PolygonShape box;
    box.count = 4;
    box.centroid = Vec2{0.0f, 0.0f};
    // SetPolygonAsBox ordering: CCW starting at (+hx, -hy).
    box.vertices[0] = Vec2{halfWidth, -halfHeight};
    box.vertices[1] = Vec2{halfWidth, halfHeight};
    box.vertices[2] = Vec2{-halfWidth, halfHeight};
    box.vertices[3] = Vec2{-halfWidth, -halfHeight};
    box.normals[0] = Vec2{1.0f, 0.0f};  // edge 0: right face
    box.normals[1] = Vec2{0.0f, 1.0f};  // edge 1: top face
    box.normals[2] = Vec2{-1.0f, 0.0f}; // edge 2: left face
    box.normals[3] = Vec2{0.0f, -1.0f}; // edge 3: bottom face
    return box;
}

// The ground is a wide box translated so that its top face (edge 1) lies exactly
// on the world line y = 0, which makes every expected penetration a plain
// difference of y coordinates.
PolygonShape MakeGround(float halfWidth) {
    return MakeBox(halfWidth, 0.5f);
}

Transform MakeGroundTransform() {
    return Transform{Vec2{0.0f, -0.5f}, Rot{}};
}

Transform MakeTransform(float x, float y, float angleRadians = 0.0f) {
    return Transform{Vec2{x, y}, Rot{angleRadians}};
}

// ---------------------------------------------------------------------------
// assertions
// ---------------------------------------------------------------------------

void ExpectFiniteManifold(const Manifold& manifold) {
    EXPECT_TRUE(std::isfinite(manifold.normal.x));
    EXPECT_TRUE(std::isfinite(manifold.normal.y));
    EXPECT_TRUE(std::isfinite(manifold.penetration));
    EXPECT_GE(manifold.pointCount, 0);
    EXPECT_LE(manifold.pointCount, p2::kMaxManifoldPoints);
    if (manifold.pointCount > 0) {
        // The solver's K_n derivation assumes a unit normal and a non-negative
        // depth, so both are part of the narrow-phase contract.
        EXPECT_NEAR(std::sqrt(p2::Dot(manifold.normal, manifold.normal)), 1.0f, kDirectionTolerance);
        EXPECT_GE(manifold.penetration, 0.0f);
    }
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        EXPECT_TRUE(std::isfinite(manifold.points[i].localPointA.x)) << "point " << i;
        EXPECT_TRUE(std::isfinite(manifold.points[i].localPointA.y)) << "point " << i;
        EXPECT_TRUE(std::isfinite(manifold.points[i].localPointB.x)) << "point " << i;
        EXPECT_TRUE(std::isfinite(manifold.points[i].localPointB.y)) << "point " << i;
        // The generator must leave the accumulated impulses at zero: only the
        // solver writes them (Manifold.h).
        EXPECT_FLOAT_EQ(manifold.points[i].normalImpulse, 0.0f);
        EXPECT_FLOAT_EQ(manifold.points[i].tangentImpulse, 0.0f);
    }
}

// Both anchors must describe the same world point: the solver rebuilds rA and rB
// from these local points every frame.
void ExpectWorldAnchorsAgree(const Manifold& manifold, const Transform& xfA, const Transform& xfB) {
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        const Vec2 worldA = p2::Mul(xfA, manifold.points[i].localPointA);
        const Vec2 worldB = p2::Mul(xfB, manifold.points[i].localPointB);
        EXPECT_NEAR(worldA.x, worldB.x, kTolerance) << "point " << i;
        EXPECT_NEAR(worldA.y, worldB.y, kTolerance) << "point " << i;
    }
}

std::vector<uint16_t> KeySet(const Manifold& manifold) {
    std::vector<uint16_t> keys;
    keys.reserve(static_cast<std::size_t>(manifold.pointCount));
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        keys.push_back(manifold.points[i].id.Key());
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

void ExpectSameKeySet(const Manifold& lhs, const Manifold& rhs) {
    const std::vector<uint16_t> lhsKeys = KeySet(lhs);
    const std::vector<uint16_t> rhsKeys = KeySet(rhs);
    ASSERT_EQ(lhsKeys.size(), rhsKeys.size());
    for (std::size_t i = 0; i < lhsKeys.size(); ++i) {
        EXPECT_EQ(lhsKeys[i], rhsKeys[i]) << "key " << i;
    }
}

void ExpectManifoldsEqual(const Manifold& lhs, const Manifold& rhs) {
    ASSERT_EQ(lhs.pointCount, rhs.pointCount);
    EXPECT_NEAR(lhs.normal.x, rhs.normal.x, kTolerance);
    EXPECT_NEAR(lhs.normal.y, rhs.normal.y, kTolerance);
    EXPECT_NEAR(lhs.penetration, rhs.penetration, kTolerance);
    for (int32_t i = 0; i < lhs.pointCount; ++i) {
        EXPECT_NEAR(lhs.points[i].localPointA.x, rhs.points[i].localPointA.x, kTolerance) << "point " << i;
        EXPECT_NEAR(lhs.points[i].localPointA.y, rhs.points[i].localPointA.y, kTolerance) << "point " << i;
        EXPECT_NEAR(lhs.points[i].localPointB.x, rhs.points[i].localPointB.x, kTolerance) << "point " << i;
        EXPECT_NEAR(lhs.points[i].localPointB.y, rhs.points[i].localPointB.y, kTolerance) << "point " << i;
        EXPECT_EQ(lhs.points[i].id, rhs.points[i].id) << "point " << i;
    }
}

// Swapping the arguments must return the flipped manifold: normal negated, same
// penetration, per-body anchors exchanged, and the *same* ContactID key set while
// the orientation bit flips (P07 D3/D5).
void ExpectFlippedManifold(const Manifold& ab, const Manifold& ba) {
    ASSERT_EQ(ab.pointCount, ba.pointCount);
    EXPECT_NEAR(ab.normal.x, -ba.normal.x, kTolerance);
    EXPECT_NEAR(ab.normal.y, -ba.normal.y, kTolerance);
    EXPECT_NEAR(ab.penetration, ba.penetration, kTolerance);
    for (int32_t i = 0; i < ab.pointCount; ++i) {
        // The reference/incident faces are the same physical faces in both calls,
        // so the points come out in the same order; only the per-body anchors are
        // exchanged.
        EXPECT_NEAR(ab.points[i].localPointA.x, ba.points[i].localPointB.x, kTolerance) << "point " << i;
        EXPECT_NEAR(ab.points[i].localPointA.y, ba.points[i].localPointB.y, kTolerance) << "point " << i;
        EXPECT_NEAR(ab.points[i].localPointB.x, ba.points[i].localPointA.x, kTolerance) << "point " << i;
        EXPECT_NEAR(ab.points[i].localPointB.y, ba.points[i].localPointA.y, kTolerance) << "point " << i;
        EXPECT_EQ(ab.points[i].id.Key(), ba.points[i].id.Key()) << "point " << i;
        EXPECT_NE(static_cast<int>(ab.points[i].id.flip), static_cast<int>(ba.points[i].id.flip))
            << "point " << i;
    }
    ExpectSameKeySet(ab, ba);
}

// ---------------------------------------------------------------------------
// FindMaxSeparation (P07 requirement A)
// ---------------------------------------------------------------------------

TEST(FindMaxSeparation, ReportsTheAxisOfMaximumSeparation) {
    // A: 2x2 box at the origin. B: identical box centred at (3,0).
    // Along A's right-face normal (1,0), A's face plane is x = +1 and B's closest
    // vertex is at x = 2, so separation = 2 - 1 = +1.
    // Every other axis is deeper: top face (0,1) -> min(yB) - 1 = -1 - 1 = -2,
    // so the right face (edge 0) wins.
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    const Transform xfA = MakeTransform(0.0f, 0.0f);
    const Transform xfB = MakeTransform(3.0f, 0.0f);

    const SeparationResult result = p2::FindMaxSeparation(box, xfA, box, xfB);
    EXPECT_EQ(result.edgeIndex, 0);
    EXPECT_NEAR(result.separation, 1.0f, kTolerance);

    // Symmetry check from B's point of view: B's left face is edge 2 and the
    // separation is the same +1.
    const SeparationResult reverse = p2::FindMaxSeparation(box, xfB, box, xfA);
    EXPECT_EQ(reverse.edgeIndex, 2);
    EXPECT_NEAR(reverse.separation, 1.0f, kTolerance);
}

TEST(FindMaxSeparation, AppliesRotationAndTranslation) {
    // A is rotated by 90 degrees, so its local +x axis becomes world +y: the
    // world normals are (0,1), (-1,0), (0,-1), (1,0) for edges 0..3. B sits one
    // metre above A's topmost point at (0,3) (B spans y in [2,4]).
    // Edge 0's world normal is now (0,1): separation = min(yB) - 1 = 2 - 1 = +1.
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    const Transform xfA = MakeTransform(0.0f, 0.0f, p2::kHalfPi);
    const Transform xfB = MakeTransform(0.0f, 3.0f);
    const SeparationResult rotated = p2::FindMaxSeparation(box, xfA, box, xfB);
    EXPECT_EQ(rotated.edgeIndex, 0);
    EXPECT_NEAR(rotated.separation, 1.0f, kTolerance);

    // Translating both polygons must not change the axis or the separation: SAT
    // only depends on relative placement.
    const Transform xfATranslated = MakeTransform(10.0f, -5.0f, p2::kHalfPi);
    const Transform xfBTranslated = MakeTransform(10.0f, -2.0f);
    const SeparationResult translated = p2::FindMaxSeparation(box, xfATranslated, box, xfBTranslated);
    EXPECT_EQ(translated.edgeIndex, 0);
    EXPECT_NEAR(translated.separation, rotated.separation, kTolerance);
}

TEST(FindMaxSeparation, MatchesTheHandComputedValueForRotatedBoxes) {
    // A is rotated 45 degrees, B stays axis aligned at (3,0).
    // A's edge 0 has world normal n = (cos45, sin45) and anchor
    //   v0 = Rot45 * (1,-1) = (2*cos45, 0) = (1.4142136, 0).
    // The minimum of Dot(n, vB - v0) over B's corners is at (2,-1):
    //   Dot(n, (2,-1) - (1.4142136, 0)) = cos45*0.5857864 - sin45*1
    //                                    = 0.4142136 - 0.7071068 = -0.2928932.
    // The other A faces are much deeper, so edge 0 wins.
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    const Transform xfA = MakeTransform(0.0f, 0.0f, p2::kPi * 0.25f);
    const Transform xfB = MakeTransform(3.0f, 0.0f);
    const SeparationResult result = p2::FindMaxSeparation(box, xfA, box, xfB);
    EXPECT_EQ(result.edgeIndex, 0);
    EXPECT_NEAR(result.separation, -0.2928932f, kTolerance);
}

TEST(FindMaxSeparation, DegenerateShapeReportsSeparated) {
    // A polygon with fewer than three vertices has no convex face set; the
    // function reports the positive "separated" sentinel instead of reading
    // uninitialised storage.
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    PolygonShape degenerate;
    degenerate.count = 2;
    degenerate.vertices[0] = Vec2{0.0f, 0.0f};
    degenerate.vertices[1] = Vec2{1.0f, 0.0f};
    degenerate.normals[0] = Vec2{0.0f, 1.0f};
    degenerate.normals[1] = Vec2{0.0f, -1.0f};

    const SeparationResult result =
        p2::FindMaxSeparation(degenerate, MakeTransform(0.0f, 0.0f), box, MakeTransform(0.0f, 0.0f));
    EXPECT_EQ(result.separation, std::numeric_limits<float>::max());
}

// ---------------------------------------------------------------------------
// The nine relative poses (D2)
// ---------------------------------------------------------------------------

TEST(CollidePolygons, SeparatedAlongTheFourAxisDirections) {
    // Two unit boxes (half extent 1). A at the origin spans [-1,1]^2; B is placed
    // three metres away in each direction, so the gap is 1 m on every axis and SAT
    // must find a separating axis (and report no contact points).
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    const Transform xfA = MakeTransform(0.0f, 0.0f);
    const Vec2 offsets[4] = {{0.0f, 3.0f}, {0.0f, -3.0f}, {-3.0f, 0.0f}, {3.0f, 0.0f}};
    for (const Vec2& offset : offsets) {
        const Transform xfB = Transform{offset, Rot{}};
        const SeparationResult separation = p2::FindMaxSeparation(box, xfA, box, xfB);
        EXPECT_GT(separation.separation, 0.0f) << "offset " << offset.x << ", " << offset.y;

        const Manifold manifold = p2::CollidePolygons(box, xfA, box, xfB, nullptr);
        EXPECT_EQ(manifold.pointCount, 0) << "offset " << offset.x << ", " << offset.y;
        EXPECT_FALSE(manifold.IsTouching());
        ExpectFiniteManifold(manifold);
    }
}

TEST(CollidePolygons, FaceFaceContactYieldsTwoPoints) {
    // Ground: 10 x 1 box whose top face (edge 1) is y = 0.
    // Box B: half extent 0.5, centred at y = 0.45, so its bottom face (edge 3) is
    // y = -0.05: penetration = 0 - (-0.05) = 0.05 along the ground's normal +y.
    // The two candidate axes are exactly tied (both separations are -0.05), so the
    // argument-order independent tie-break keeps the ground as the reference face.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.45f);

    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_EQ(manifold.pointCount, 2);
    // normal A -> B: B is above the ground.
    EXPECT_NEAR(manifold.normal.x, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.normal.y, 1.0f, kTolerance);
    EXPECT_NEAR(manifold.penetration, 0.05f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, groundXf, boxXf);

    // Incident edge = B's bottom edge (edge 3, vertices 3 -> 0), clipped inside the
    // ground face: the two box corners project to x = -0.5 and +0.5.
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(manifold.points[1].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.incidentVertex), 3);
    EXPECT_EQ(static_cast<int>(manifold.points[1].id.incidentVertex), 0);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.type), p2::kContactTypeFaceFace);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.flip), 0);
    // Point 0 = box vertex 3 world = (-0.5, -0.05) -> local A = (-0.5, 0.45),
    // local B = (-0.5, -0.5); point 1 is the mirror at +0.5.
    EXPECT_NEAR(manifold.points[0].localPointB.x, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointB.y, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.x, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.y, 0.45f, kTolerance);
    EXPECT_NEAR(manifold.points[1].localPointB.x, 0.5f, kTolerance);
}

TEST(CollidePolygons, VertexFaceContactYieldsOnePoint) {
    // Box B is rotated 45 degrees and lowered until its bottom corner touches the
    // ground plane with 0.02 m of penetration. With half extent 0.5 the corner is
    // at distance 0.5*(cos45 + sin45) = 0.70710678 from the centre, so the centre
    // is placed at y = 0.70710678 - 0.02 and the lowest vertex (local (-0.5,-0.5))
    // is at y = -0.02.
    //
    // Reference face = ground top (edge 1). Incident face = the box edge whose
    // world normal is most anti-parallel to (0,1): at 45 degrees edges 2 and 3 are
    // tied at -cos45, the first one (edge 2, vertices 2 -> 3) is chosen by the
    // strict-less loop. Its vertex 2 is at y = +0.6871 (above the plane, dropped)
    // and vertex 3 is at y = -0.02 (penetrating), so exactly one point survives:
    // a vertex-face contact.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Rot rotation(p2::kPi * 0.25f);
    const float cornerDistance = 0.5f * (rotation.c + rotation.s);
    const Transform boxXf = Transform{Vec2{0.0f, cornerDistance - 0.02f}, rotation};

    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_EQ(manifold.pointCount, 1);
    EXPECT_NEAR(manifold.normal.x, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.normal.y, 1.0f, kTolerance);
    EXPECT_NEAR(manifold.penetration, 0.02f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, groundXf, boxXf);

    // The surviving point is box vertex 3 in world space: (0, -0.02).
    EXPECT_NEAR(manifold.points[0].localPointB.x, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointB.y, -0.5f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.x, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.points[0].localPointA.y, 0.48f, kTolerance);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.incidentVertex), 3);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.type), p2::kContactTypeVertexFace);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.flip), 0);
}

TEST(CollidePolygons, TangentFaceFaceContactYieldsZeroPenetration) {
    // Box B rests exactly on the ground: centre y = 0.5 puts its bottom face on
    // y = 0, so both candidate separations are exactly 0 (touching, not apart)
    // and both bottom corners are contact points with penetration 0.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.5f);

    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_GE(manifold.pointCount, 1);
    EXPECT_EQ(manifold.pointCount, 2);
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
    EXPECT_NEAR(manifold.normal.y, 1.0f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, groundXf, boxXf);
}

TEST(CollidePolygons, FullyContainedPolygonStillYieldsPoints) {
    // A: 10 x 10 box at the origin; B: 1 x 1 box at (4,0), i.e. inside A's right
    // half. Penetrations along the axes: A's right face (edge 0) reaches x = 5,
    // B's left face is at x = 3.5, so the least-penetrating axis is 1.5 deep on
    // both sides (exact tie); the tie-break keeps A (smaller edge index 0 vs 2).
    // Incident edge = B's left face (vertices 2 -> 3) at x = 3.5, and
    // penetration = 5 - 3.5 = 1.5 for both points.
    const PolygonShape largeBox = MakeBox(5.0f, 5.0f);
    const Transform largeXf = MakeTransform(0.0f, 0.0f);
    const PolygonShape smallBox = MakeBox(0.5f, 0.5f);
    const Transform smallXf = MakeTransform(4.0f, 0.0f);

    const Manifold manifold = p2::CollidePolygons(largeBox, largeXf, smallBox, smallXf, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_GE(manifold.pointCount, 1);
    EXPECT_EQ(manifold.pointCount, 2);
    EXPECT_NEAR(manifold.normal.x, 1.0f, kTolerance);
    EXPECT_NEAR(manifold.normal.y, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.penetration, 1.5f, kTolerance);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 0);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.incidentVertex), 2);
    EXPECT_EQ(static_cast<int>(manifold.points[1].id.incidentVertex), 3);
    ExpectWorldAnchorsAgree(manifold, largeXf, smallXf);
}

TEST(CollidePolygons, ParallelFacesClipToTwoPoints) {
    // Edge-to-edge parallel pose: two unit boxes overlapping only along x.
    // A spans x in [-1,1]; B spans x in [0.9, 2.9] (centre x = 1.9), so the
    // x-overlap is 0.1 and both y faces are exactly tied. The clip keeps the part
    // of B's left face (edge 2, x = 0.9) that lies inside A's right face side
    // planes: the corners (0.9, 1) and (0.9, -1) sit exactly on the side planes and
    // survive, giving two points with penetration 1 - 0.9 = 0.1.
    const PolygonShape box = MakeBox(1.0f, 1.0f);
    const Transform xfA = MakeTransform(0.0f, 0.0f);
    const Transform xfB = MakeTransform(1.9f, 0.0f);

    const Manifold manifold = p2::CollidePolygons(box, xfA, box, xfB, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_EQ(manifold.pointCount, 2);
    EXPECT_NEAR(manifold.normal.x, 1.0f, kTolerance);
    EXPECT_NEAR(manifold.normal.y, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.penetration, 0.1f, kTolerance);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 0);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.incidentVertex), 2);
    EXPECT_EQ(static_cast<int>(manifold.points[1].id.incidentVertex), 3);
    ExpectWorldAnchorsAgree(manifold, xfA, xfB);
}

// ---------------------------------------------------------------------------
// A/B swap invariance (P07 D3/D5)
// ---------------------------------------------------------------------------

TEST(CollidePolygons, SwappingArgumentsFlipsTheFaceFaceManifold) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.45f);

    const Manifold ab = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    const Manifold ba = p2::CollidePolygons(box, boxXf, ground, groundXf, nullptr);
    ExpectFlippedManifold(ab, ba);
    ExpectFiniteManifold(ab);
    ExpectFiniteManifold(ba);
    // Ground is edge 1 in both calls and the incident box is the same physical
    // polygon, so the key set is identical even though the reference ownership bit
    // flips (0 when the ground is A, 1 when it is B).
    EXPECT_EQ(static_cast<int>(ab.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(ab.points[0].id.flip), 0);
    EXPECT_EQ(static_cast<int>(ba.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(ba.points[0].id.flip), 1);
    EXPECT_EQ(static_cast<int>(ba.points[0].id.incidentVertex), 3);
}

TEST(CollidePolygons, SwappingArgumentsFlipsTheVertexFaceManifold) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Rot rotation(p2::kPi * 0.25f);
    const Transform boxXf = Transform{Vec2{0.0f, 0.5f * (rotation.c + rotation.s) - 0.02f}, rotation};

    const Manifold ab = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    const Manifold ba = p2::CollidePolygons(box, boxXf, ground, groundXf, nullptr);
    ExpectFlippedManifold(ab, ba);
    ASSERT_EQ(ab.pointCount, 1);
    EXPECT_EQ(ab.points[0].id.Key(), ba.points[0].id.Key());
    EXPECT_EQ(static_cast<int>(ab.points[0].id.type), p2::kContactTypeVertexFace);
}

TEST(CollidePolygons, ReferenceOwnershipBitRecordsWhichPolygonOwnsTheFace) {
    // Caller order (box, ground): the ground still owns the reference face because
    // its separation is the larger one, so every point is flagged flip = 1 and the
    // normal points from the box (A) down towards the ground (B).
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.45f);

    const Manifold manifold = p2::CollidePolygons(box, boxXf, ground, groundXf, nullptr);
    ExpectFiniteManifold(manifold);
    ASSERT_EQ(manifold.pointCount, 2);
    EXPECT_NEAR(manifold.normal.x, 0.0f, kTolerance);
    EXPECT_NEAR(manifold.normal.y, -1.0f, kTolerance);
    EXPECT_NEAR(manifold.penetration, 0.05f, kTolerance);
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        EXPECT_EQ(static_cast<int>(manifold.points[i].id.flip), 1) << "point " << i;
        EXPECT_EQ(static_cast<int>(manifold.points[i].id.referenceEdge), 1) << "point " << i;
    }
    // incidentVertex is the absolute vertex index on the incident polygon, so the
    // box's bottom corners keep indices 3 and 0 in whichever argument slot it sits.
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.incidentVertex), 3);
    EXPECT_EQ(static_cast<int>(manifold.points[1].id.incidentVertex), 0);
}

// ---------------------------------------------------------------------------
// Degenerate geometry
// ---------------------------------------------------------------------------

TEST(CollidePolygons, ThinStripAndTinyBoxProduceFiniteResults) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();

    // A 1e-3 x 10 strip tilted 30 degrees with its lower end 0.01 m into the
    // ground: normals/edges are still well defined, so the generator must return a
    // usable manifold without NaN or Inf.
    const PolygonShape strip = MakeBox(0.0005f, 5.0f);
    const Rot tilt(30.0f * p2::kDegToRad);
    const Transform stripXf = Transform{Vec2{0.0f, 5.0f * tilt.c - 0.01f}, tilt};
    const Manifold stripManifold = p2::CollidePolygons(ground, groundXf, strip, stripXf, nullptr);
    ExpectFiniteManifold(stripManifold);
    EXPECT_GE(stripManifold.pointCount, 1);
    EXPECT_GT(stripManifold.penetration, 0.0f);

    // A 1e-4 x 1e-4 box resting 1e-5 m into the ground: the smallest geometry the
    // design document asks about (Section 6.3 degenerate handling).
    const PolygonShape tiny = MakeBox(5.0e-5f, 5.0e-5f);
    const Transform tinyXf = MakeTransform(0.0f, 4.0e-5f);
    const Manifold tinyManifold = p2::CollidePolygons(ground, groundXf, tiny, tinyXf, nullptr);
    ExpectFiniteManifold(tinyManifold);
    EXPECT_GE(tinyManifold.pointCount, 1);
    EXPECT_NEAR(tinyManifold.penetration, 1.0e-5f, 1.0e-6f);
}

TEST(CollidePolygons, CollinearVertexDoesNotBreakTheManifold) {
    // The same 2x2 box as MakeBox(1,1) but with a redundant vertex in the middle of
    // the bottom edge: (-1,-1), (0,-1), (1,-1), (1,1), (-1,1). SetPolygon rejects
    // this input (collinear vertex), so the shape is hand-built here to prove the
    // SAT path is total: it must not produce NaN or read out of bounds.
    PolygonShape collinear;
    collinear.count = 5;
    collinear.centroid = Vec2{0.0f, 0.0f};
    collinear.vertices[0] = Vec2{-1.0f, -1.0f};
    collinear.vertices[1] = Vec2{0.0f, -1.0f};
    collinear.vertices[2] = Vec2{1.0f, -1.0f};
    collinear.vertices[3] = Vec2{1.0f, 1.0f};
    collinear.vertices[4] = Vec2{-1.0f, 1.0f};
    collinear.normals[0] = Vec2{0.0f, -1.0f}; // (-1,-1) -> (0,-1)
    collinear.normals[1] = Vec2{0.0f, -1.0f}; // (0,-1)  -> (1,-1)
    collinear.normals[2] = Vec2{1.0f, 0.0f};  // (1,-1)  -> (1,1)
    collinear.normals[3] = Vec2{0.0f, 1.0f};  // (1,1)   -> (-1,1)
    collinear.normals[4] = Vec2{-1.0f, 0.0f}; // (-1,1)  -> (-1,-1)

    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    // Bottom edge 0.01 m below the ground face.
    const Transform collinearXf = MakeTransform(0.0f, 0.99f);

    const Manifold manifold = p2::CollidePolygons(ground, groundXf, collinear, collinearXf, nullptr);
    ExpectFiniteManifold(manifold);
    EXPECT_GE(manifold.pointCount, 1);
    EXPECT_NEAR(manifold.penetration, 0.01f, kTolerance);
    ExpectWorldAnchorsAgree(manifold, groundXf, collinearXf);
}

TEST(CollidePolygons, MalformedShapesAreRejected) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();

    PolygonShape tooFew;
    tooFew.count = 2;
    tooFew.vertices[0] = Vec2{-0.5f, -0.5f};
    tooFew.vertices[1] = Vec2{0.5f, -0.5f};
    tooFew.normals[0] = Vec2{0.0f, -1.0f};
    tooFew.normals[1] = Vec2{1.0f, 0.0f};

    const Manifold manifold =
        p2::CollidePolygons(ground, groundXf, tooFew, MakeTransform(0.0f, 0.0f), nullptr);
    EXPECT_EQ(manifold.pointCount, 0);
    const Manifold reversed =
        p2::CollidePolygons(tooFew, MakeTransform(0.0f, 0.0f), ground, groundXf, nullptr);
    EXPECT_EQ(reversed.pointCount, 0);
}

TEST(CollidePolygons, FixtureMatchesTheEngineBoxConstruction) {
    // The hand-filled fixtures above bypass SetPolygonAsBox(), so this cross-check
    // pins them to the real construction path: same vertex order, same outward
    // normals, same centroid. Without it the expected edge indices and ContactIDs
    // in this file could silently stop matching the shapes the engine builds.
    PolygonShape built;
    ASSERT_TRUE(p2::SetPolygonAsBox(built, 1.0f, 1.0f));
    const PolygonShape fixture = MakeBox(1.0f, 1.0f);
    ASSERT_EQ(built.count, fixture.count);
    for (int32_t i = 0; i < fixture.count; ++i) {
        EXPECT_EQ(built.vertices[i], fixture.vertices[i]) << "vertex " << i;
        EXPECT_EQ(built.normals[i], fixture.normals[i]) << "normal " << i;
    }
    EXPECT_NEAR(built.centroid.x, fixture.centroid.x, kTolerance);
    EXPECT_NEAR(built.centroid.y, fixture.centroid.y, kTolerance);

    // And the engine-built box yields the same manifold as the fixture.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const Transform boxXf = MakeTransform(0.0f, 0.45f);
    const Manifold fromFixture = p2::CollidePolygons(ground, groundXf, fixture, boxXf, nullptr);
    const Manifold fromEngine = p2::CollidePolygons(ground, groundXf, built, boxXf, nullptr);
    ExpectManifoldsEqual(fromFixture, fromEngine);
}

// ---------------------------------------------------------------------------
// SAT cache (D6)
// ---------------------------------------------------------------------------

// A pose where the two candidate reference axes are almost equally deep: a box
// rotated by only 1e-6 rad on the ground. The ground's top face is the best axis
// by 5*sin(theta) ~ 5e-6, which is inside the cache's relative epsilon
// (kRelativeEpsilon * max(1, |separation|) = 1e-5), so a cached "the box owns the
// reference face" hint must be kept - while a caller without a cache gets the
// ground reference face. The physical result is the same either way.
struct NearTiedScene {
    PolygonShape box;
    Transform boxXf;
    PolygonShape ground;
    Transform groundXf;
    SeparationResult boxSeparation;    // best axis of the box (cached hint)
    SeparationResult groundSeparation; // best axis of the ground (default choice)
};

NearTiedScene MakeNearTiedScene() {
    NearTiedScene scene;
    scene.ground = MakeGround(5.0f);
    scene.groundXf = MakeGroundTransform();

    scene.box = MakeBox(0.5f, 0.5f);
    const Rot tilt(1.0e-6f);
    // Lowest vertex y = centreY - 0.5*(cos + sin) = -0.02.
    scene.boxXf = Transform{Vec2{0.0f, 0.5f * (tilt.c + tilt.s) - 0.02f}, tilt};

    scene.boxSeparation = p2::FindMaxSeparation(scene.box, scene.boxXf, scene.ground, scene.groundXf);
    scene.groundSeparation = p2::FindMaxSeparation(scene.ground, scene.groundXf, scene.box, scene.boxXf);
    return scene;
}

TEST(SatCache, HysteresisKeepsThePreviousReferenceOnNearTiedAxes) {
    const NearTiedScene scene = MakeNearTiedScene();

    // Preconditions of the near-tie (asserted so a fixture change fails loudly):
    // the ground's axis (edge 1) is the best one, the box's bottom face (edge 3) is
    // the cached alternative, and the difference is inside the relative epsilon.
    ASSERT_EQ(scene.groundSeparation.edgeIndex, 1);
    ASSERT_EQ(scene.boxSeparation.edgeIndex, 3);
    ASSERT_GT(scene.groundSeparation.separation, scene.boxSeparation.separation);
    const float tolerance =
        p2::kRelativeEpsilon * std::max(1.0f, std::fabs(scene.groundSeparation.separation));
    ASSERT_LE(scene.groundSeparation.separation - scene.boxSeparation.separation, tolerance);

    // Without a cache the ground wins: reference edge 1 on B, flip = 1.
    const Manifold uncached =
        p2::CollidePolygons(scene.box, scene.boxXf, scene.ground, scene.groundXf, nullptr);
    ExpectFiniteManifold(uncached);
    ASSERT_EQ(uncached.pointCount, 2);
    EXPECT_EQ(static_cast<int>(uncached.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(uncached.points[0].id.flip), 1);

    // With the cached hint the previous reference face (the box's bottom edge 3,
    // owned by A) is kept.
    SATCache cache;
    cache.referenceBody = 0;
    cache.referenceEdge = 3;
    const Manifold cached = p2::CollidePolygons(scene.box, scene.boxXf, scene.ground, scene.groundXf, &cache);
    ExpectFiniteManifold(cached);
    ASSERT_EQ(cached.pointCount, 2);
    for (int32_t i = 0; i < cached.pointCount; ++i) {
        EXPECT_EQ(static_cast<int>(cached.points[i].id.referenceEdge), 3) << "point " << i;
        EXPECT_EQ(static_cast<int>(cached.points[i].id.flip), 0) << "point " << i;
    }
    EXPECT_EQ(static_cast<int>(cached.points[0].id.incidentVertex), 2);
    EXPECT_EQ(static_cast<int>(cached.points[1].id.incidentVertex), 1);
    // The cache was honoured and written back unchanged.
    EXPECT_EQ(cache.referenceBody, 0);
    EXPECT_EQ(cache.referenceEdge, 3);
}

TEST(SatCache, CachedAndUncachedResultsArePhysicallyEquivalent) {
    // Enabling or disabling hysteresis may only change which of the two near-tied
    // axes is reported: the normal direction and the penetration depth must agree
    // to within the hysteresis tolerance itself.
    const NearTiedScene scene = MakeNearTiedScene();
    SATCache cache;
    cache.referenceBody = 0;
    cache.referenceEdge = 3;

    const Manifold uncached =
        p2::CollidePolygons(scene.box, scene.boxXf, scene.ground, scene.groundXf, nullptr);
    const Manifold cached = p2::CollidePolygons(scene.box, scene.boxXf, scene.ground, scene.groundXf, &cache);

    ASSERT_EQ(uncached.pointCount, cached.pointCount);
    // Unrotated ground axis: (0,-1) (A = box is above B = ground). Rotated box
    // axis: (sin theta, -cos theta) = (1e-6, -1); both point A -> B.
    EXPECT_NEAR(uncached.normal.x, 0.0f, kDirectionTolerance);
    EXPECT_NEAR(uncached.normal.y, -1.0f, kDirectionTolerance);
    EXPECT_NEAR(cached.normal.x, 1.0e-6f, kDirectionTolerance);
    EXPECT_NEAR(cached.normal.y, -1.0f, kDirectionTolerance);
    EXPECT_NEAR(cached.normal.x, uncached.normal.x, kDirectionTolerance);
    EXPECT_NEAR(cached.normal.y, uncached.normal.y, kDirectionTolerance);
    EXPECT_NEAR(cached.penetration, uncached.penetration, kTolerance);

    // Numbers for the Week 4 D6 devlog entry: the same physical contact, reported
    // through two different near-tied axes, differs only at the 1e-6 level.
    std::cout << "[sat-cache] uncached (ground axis): refEdge="
              << static_cast<int>(uncached.points[0].id.referenceEdge)
              << " flip=" << static_cast<int>(uncached.points[0].id.flip) << " normal=(" << uncached.normal.x
              << ", " << uncached.normal.y << ") penetration=" << uncached.penetration << "\n";
    std::cout << "[sat-cache] cached   (box axis):    refEdge="
              << static_cast<int>(cached.points[0].id.referenceEdge)
              << " flip=" << static_cast<int>(cached.points[0].id.flip) << " normal=(" << cached.normal.x
              << ", " << cached.normal.y << ") penetration=" << cached.penetration << "\n";
    std::cout << "[sat-cache] delta |normal|=" << std::fabs(cached.normal.x - uncached.normal.x) << ", "
              << std::fabs(cached.normal.y - uncached.normal.y)
              << " |penetration|=" << std::fabs(cached.penetration - uncached.penetration)
              << " (hysteresis tolerance=" << p2::kRelativeEpsilon << ")\n";

    // The axis difference shows up only in the feature keys: the cached manifold
    // reports the box's bottom edge (3) as the reference, the uncached one the
    // ground's top edge (1).
    const std::vector<uint16_t> cachedKeys = KeySet(cached);
    const std::vector<uint16_t> uncachedKeys = KeySet(uncached);
    ASSERT_EQ(cachedKeys.size(), uncachedKeys.size());
    bool keysDiffer = false;
    for (std::size_t i = 0; i < cachedKeys.size(); ++i) {
        if (cachedKeys[i] != uncachedKeys[i]) {
            keysDiffer = true;
        }
    }
    EXPECT_TRUE(keysDiffer);
}

TEST(SatCache, CacheIsWrittenBackForTheNextFrame) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.45f);

    SATCache cache;
    EXPECT_FALSE(cache.IsValid());
    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, &cache);
    ASSERT_EQ(manifold.pointCount, 2);
    // The ground owns the reference face (edge 1), so the cache records body 0 /
    // edge 1 for the next frame.
    EXPECT_TRUE(cache.IsValid());
    EXPECT_EQ(cache.referenceBody, 0);
    EXPECT_EQ(cache.referenceEdge, 1);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.flip), 0);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 1);

    // A second solve with the now-valid cache produces the same manifold.
    const Manifold again = p2::CollidePolygons(ground, groundXf, box, boxXf, &cache);
    ExpectManifoldsEqual(manifold, again);
}

TEST(SatCache, InvalidCachedEdgeIsIgnored) {
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Transform boxXf = MakeTransform(0.0f, 0.45f);

    const Manifold reference = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);

    SATCache cache;
    cache.referenceBody = 1;  // the box
    cache.referenceEdge = 99; // out of range for a 4-vertex polygon
    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, &cache);
    // The bogus edge is discarded, the best axis is used, and the cache is repaired.
    ExpectManifoldsEqual(reference, manifold);
    EXPECT_EQ(cache.referenceBody, 0);
    EXPECT_EQ(cache.referenceEdge, 1);
}

TEST(SatCache, CacheIsIgnoredWhenTheAxesDifferClearly) {
    // 45-degree corner pose: the ground axis (-0.02) and the box axis (about -3.55)
    // are far apart, so a stale cached hint must not override the best axis.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    const Rot rotation(p2::kPi * 0.25f);
    const Transform boxXf = Transform{Vec2{0.0f, 0.5f * (rotation.c + rotation.s) - 0.02f}, rotation};

    const Manifold reference = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
    ASSERT_EQ(reference.pointCount, 1);

    SATCache cache;
    cache.referenceBody = 1; // the box, deliberately the wrong (deep) axis
    cache.referenceEdge = 0;
    const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, &cache);
    ExpectManifoldsEqual(reference, manifold);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.referenceEdge), 1);
    EXPECT_EQ(static_cast<int>(manifold.points[0].id.flip), 0);
}

// ---------------------------------------------------------------------------
// Rotating-box stability (P07 / D5): the devlog data
// ---------------------------------------------------------------------------

TEST(CollidePolygons, RotatingBoxKeepsItsContactFeaturePairStable) {
    // A unit box resting on the ground, rotated through 90 degrees in 0.5 degree
    // steps. The centre is placed so that the deepest corner stays exactly 0.01 m
    // below the ground face for every angle:
    //   lowest vertex y = centreY - 0.5*(cos + sin)  =>  centreY = 0.5*(cos+sin) - 0.01.
    //
    // Expected feature behaviour (hand-derived):
    //   * 0.0 deg and 0.5 deg: the bottom corner v0 is still below the plane
    //     (sin(theta) < 0.01), so the incident bottom edge gives two points, keys
    //     {(ref=1, inc=3), (ref=1, inc=0)};
    //   * 1.0 .. 89.0 deg: only the deepest corner v3 penetrates, so a single
    //     vertex-face point with (ref=1, inc=3) - the feature pair does not change
    //     when the incident edge switches from edge 3 to edge 2 at 45 degrees,
    //     because the deepest corner keeps its absolute vertex index 3;
    //   * 89.5 and 90.0 deg: the second bottom corner v2 re-enters
    //     (cos(theta) < 0.01), adding (ref=1, inc=2).
    // So the key *set* changes exactly twice while the deepest-corner pair is
    // present in every frame.
    const PolygonShape ground = MakeGround(5.0f);
    const Transform groundXf = MakeGroundTransform();
    const PolygonShape box = MakeBox(0.5f, 0.5f);
    constexpr int32_t kFrameCount = 181; // 0 .. 90 degrees in 0.5 degree steps

    std::vector<uint16_t> previousKeys;
    int32_t keySetChanges = 0;
    int32_t onePointFrames = 0;
    int32_t twoPointFrames = 0;
    int32_t framesWithoutDeepestCornerPair = 0;
    int32_t previousPointCount = 0;

    for (int32_t step = 0; step < kFrameCount; ++step) {
        const float angleDegrees = 0.5f * static_cast<float>(step);
        const Rot rotation(angleDegrees * p2::kDegToRad);
        const Transform boxXf = Transform{Vec2{0.0f, 0.5f * (rotation.c + rotation.s) - 0.01f}, rotation};

        const Manifold manifold = p2::CollidePolygons(ground, groundXf, box, boxXf, nullptr);
        ExpectFiniteManifold(manifold);
        ASSERT_GE(manifold.pointCount, 1) << "lost contact at " << angleDegrees << " deg";
        if (manifold.pointCount == 1) {
            ++onePointFrames;
        } else {
            ++twoPointFrames;
        }

        bool hasDeepestCornerPair = false;
        for (int32_t i = 0; i < manifold.pointCount; ++i) {
            const p2::ContactID& id = manifold.points[i].id;
            if (id.referenceEdge == 1 && id.incidentVertex == 3) {
                hasDeepestCornerPair = true;
            }
        }
        if (!hasDeepestCornerPair) {
            ++framesWithoutDeepestCornerPair;
        }

        const std::vector<uint16_t> keys = KeySet(manifold);
        if (!previousKeys.empty() && keys != previousKeys) {
            ++keySetChanges;
            std::cout << "[rotating-box] key-set change at " << angleDegrees << " deg (pointCount "
                      << previousPointCount << " -> " << manifold.pointCount << ")\n";
        }
        previousKeys = keys;
        previousPointCount = manifold.pointCount;
    }

    std::cout << "[rotating-box] frames=" << kFrameCount << " keySetChanges=" << keySetChanges
              << " onePointFrames=" << onePointFrames << " twoPointFrames=" << twoPointFrames
              << " framesWithoutDeepestCornerPair=" << framesWithoutDeepestCornerPair << "\n";

    EXPECT_EQ(previousPointCount, 2); // 90 degrees ends with both bottom corners inside
    EXPECT_EQ(framesWithoutDeepestCornerPair, 0);
    EXPECT_EQ(keySetChanges, 2);
    EXPECT_EQ(twoPointFrames, 4);
    EXPECT_EQ(onePointFrames, kFrameCount - 4);
}

} // namespace
