// Unit tests for p2::PolygonShape — strict validation, canonical vertex order,
// precomputed edge normals/centroid and the polygonal mass properties
// (design document Week 4 D1, prompt P04).
//
// Expected values are analytic and the derivation is written next to each
// assertion.

#include <gtest/gtest.h>

#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <limits>

namespace {

using p2::AABB;
using p2::MassData;
using p2::PolygonShape;
using p2::Rot;
using p2::Transform;
using p2::Vec2;

// Geometry in these tests has O(1..10) magnitudes; 1e-5 is a few hundred ulps,
// enough for the shoelace sums and the rotation of the box corners.
constexpr float kTolerance = 1.0e-5f;

} // namespace

TEST(PolygonShape, DefaultConstructionIsInvalid) {
    const PolygonShape shape;
    // A default shape has no memory: count == 0, and every consumer gates on it.
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());
}

TEST(PolygonShape, SetPolygonAsBoxProducesCanonicalVerticesAndNormals) {
    // Canonical order documented in the header: CCW starting at (+hx, -hy).
    PolygonShape shape;
    ASSERT_TRUE(p2::SetPolygonAsBox(shape, 1.0f, 1.0f));
    ASSERT_EQ(shape.count, 4);
    ASSERT_TRUE(shape.IsValid());

    // v0=(+1,-1), v1=(+1,+1), v2=(-1,+1), v3=(-1,-1).
    const Vec2 expectedVertices[4] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f},
                                      Vec2{-1.0f, -1.0f}};
    for (int32_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(shape.vertices[i].x, expectedVertices[i].x, kTolerance) << "vertex " << i;
        EXPECT_NEAR(shape.vertices[i].y, expectedVertices[i].y, kTolerance) << "vertex " << i;
    }

    // Edge i runs vertices[i] -> vertices[i+1]; its outward normal for CCW winding
    // is edge rotated -90 degrees. Edges: right (+x), top (+y), left (-x), bottom (-y).
    const Vec2 expectedNormals[4] = {Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}, Vec2{-1.0f, 0.0f},
                                     Vec2{0.0f, -1.0f}};
    for (int32_t i = 0; i < 4; ++i) {
        const Vec2& normal = shape.normals[i];
        const Vec2& vertex = shape.vertices[i];
        const Vec2 edge = shape.vertices[(i + 1) % 4] - vertex;
        EXPECT_NEAR(normal.x, expectedNormals[i].x, kTolerance) << "normal " << i;
        EXPECT_NEAR(normal.y, expectedNormals[i].y, kTolerance) << "normal " << i;
        // Unit length and perpendicular to the edge they belong to.
        EXPECT_NEAR(p2::Length(normal), 1.0f, kTolerance) << "normal " << i;
        EXPECT_NEAR(p2::Dot(normal, edge), 0.0f, kTolerance) << "normal " << i;
        // Outward: points away from the centroid.
        EXPECT_GT(p2::Dot(normal, vertex - shape.centroid), 0.0f) << "normal " << i;
    }

    // The symmetric box has its centroid exactly at the body origin.
    EXPECT_NEAR(shape.centroid.x, 0.0f, kTolerance);
    EXPECT_NEAR(shape.centroid.y, 0.0f, kTolerance);
}

TEST(PolygonShape, SquareMassMatchesAnalyticValues) {
    // Square side 2 (half extents 1), density 1: area A = 4.
    //   mass    = rho * A = 4
    //   inertia = mass * (w^2 + h^2) / 12 = 4 * (4 + 4) / 12 = 8/3 = 2.6666667
    // (P04's "I = 4*(2^2+2^2)/12 = 2.6667" with the factor 4 = mass already applied.)
    PolygonShape square;
    ASSERT_TRUE(p2::SetPolygonAsBox(square, 1.0f, 1.0f));
    const MassData squareMass = p2::ComputePolygonMass(square, 1.0f);
    EXPECT_NEAR(squareMass.mass, 4.0f, kTolerance);
    EXPECT_NEAR(squareMass.center.x, 0.0f, kTolerance);
    EXPECT_NEAR(squareMass.center.y, 0.0f, kTolerance);
    EXPECT_NEAR(squareMass.inertia, 8.0f / 3.0f, kTolerance);

    // Mass and inertia scale linearly with density; the centre does not move.
    const MassData doubled = p2::ComputePolygonMass(square, 2.0f);
    EXPECT_NEAR(doubled.mass, 8.0f, kTolerance);
    EXPECT_NEAR(doubled.inertia, 16.0f / 3.0f, kTolerance);
    EXPECT_NEAR(doubled.center.x, 0.0f, kTolerance);

    // Rectangle 4 x 2 (half extents 2, 1), density 1: A = 8,
    //   I = 8 * (4^2 + 2^2) / 12 = 8 * 20 / 12 = 40/3 = 13.333333.
    PolygonShape rectangle;
    ASSERT_TRUE(p2::SetPolygonAsBox(rectangle, 2.0f, 1.0f));
    const MassData rectangleMass = p2::ComputePolygonMass(rectangle, 1.0f);
    EXPECT_NEAR(rectangleMass.mass, 8.0f, kTolerance);
    EXPECT_NEAR(rectangleMass.inertia, 40.0f / 3.0f, 1.0e-4f);
}

TEST(PolygonShape, EquilateralTriangleMassMatchesAnalyticValues) {
    // Triangle (-1,0), (1,0), (0,sqrt(3)): equilateral with side 2.
    //   area = base*height/2 = 2*sqrt(3)/2 = sqrt(3)
    //   centroid = (v0 + v1 + v2)/3 = (0, sqrt(3)/3)
    // Hand derivation of the inertia from the P04 formula (rho = 1):
    //   sum cross_i(...) = 0 + sqrt(3)*4 + sqrt(3)*4 = 8*sqrt(3)  (only the two
    //   side pairs contribute, because the base pair has cross == 0)
    //   I_origin = 8*sqrt(3)/12 = 2*sqrt(3)/3
    //   m*|C|^2  = sqrt(3) * (sqrt(3)/3)^2 = sqrt(3)/3
    //   I_centroid = 2*sqrt(3)/3 - sqrt(3)/3 = sqrt(3)/3  (= m*s^2/12 for s=2)
    const float sqrt3 = std::sqrt(3.0f);
    const Vec2 vertices[3] = {Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, sqrt3}};
    PolygonShape triangle;
    ASSERT_TRUE(p2::SetPolygon(triangle, vertices, 3));

    const MassData mass = p2::ComputePolygonMass(triangle, 1.0f);
    EXPECT_NEAR(mass.mass, sqrt3, kTolerance);
    EXPECT_NEAR(mass.center.x, 0.0f, kTolerance);
    EXPECT_NEAR(mass.center.y, sqrt3 / 3.0f, kTolerance);
    EXPECT_NEAR(mass.inertia, sqrt3 / 3.0f, kTolerance);

    // The precomputed centroid must agree with the mass-property result.
    EXPECT_NEAR(triangle.centroid.x, mass.center.x, kTolerance);
    EXPECT_NEAR(triangle.centroid.y, mass.center.y, kTolerance);
}

TEST(PolygonShape, SetPolygonAsBoxWithCenterAndAngle) {
    // Box half extents (1,1) rotated 90 degrees CCW around (2,3):
    //   Rot90*(x,y) = (-y,x):
    //     ( 1,-1) -> ( 1, 1)
    //     ( 1, 1) -> (-1, 1)
    //     (-1, 1) -> (-1,-1)
    //     (-1,-1) -> ( 1,-1)
    //   plus the centre gives (3,4), (1,4), (1,2), (3,2).
    PolygonShape shape;
    ASSERT_TRUE(p2::SetPolygonAsBox(shape, 1.0f, 1.0f, Vec2{2.0f, 3.0f}, p2::kHalfPi));
    ASSERT_EQ(shape.count, 4);
    ASSERT_TRUE(shape.IsValid());
    EXPECT_NEAR(shape.vertices[0].x, 3.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[0].y, 4.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[1].x, 1.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[1].y, 4.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[2].x, 1.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[2].y, 2.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[3].x, 3.0f, 1.0e-5f);
    EXPECT_NEAR(shape.vertices[3].y, 2.0f, 1.0e-5f);
    EXPECT_NEAR(shape.centroid.x, 2.0f, 1.0e-5f);
    EXPECT_NEAR(shape.centroid.y, 3.0f, 1.0e-5f);

    // The transformed box's AABB is [1,3] x [2,4].
    const AABB box = p2::ComputePolygonAABB(shape, Transform{});
    EXPECT_NEAR(box.lowerBound.x, 1.0f, kTolerance);
    EXPECT_NEAR(box.lowerBound.y, 2.0f, kTolerance);
    EXPECT_NEAR(box.upperBound.x, 3.0f, kTolerance);
    EXPECT_NEAR(box.upperBound.y, 4.0f, kTolerance);

    // The rotated box keeps the mass properties of the original box (rotation and
    // translation do not change mass or centroidal inertia).
    const MassData mass = p2::ComputePolygonMass(shape, 1.0f);
    EXPECT_NEAR(mass.mass, 4.0f, kTolerance);
    EXPECT_NEAR(mass.center.x, 2.0f, kTolerance);
    EXPECT_NEAR(mass.center.y, 3.0f, kTolerance);
    EXPECT_NEAR(mass.inertia, 8.0f / 3.0f, kTolerance);
}

TEST(PolygonShape, AabbUnderRotationAndTranslation) {
    // Box 2 x 1 (half extents 1, 0.5) rotated 90 degrees: the AABB half extents
    // swap to (0.5, 1). With position (3, 4): [2.5, 3.5] x [3, 5].
    PolygonShape box;
    ASSERT_TRUE(p2::SetPolygonAsBox(box, 1.0f, 0.5f));
    const Transform rotated{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const AABB rotatedBounds = p2::ComputePolygonAABB(box, rotated);
    EXPECT_NEAR(rotatedBounds.lowerBound.x, 2.5f, kTolerance);
    EXPECT_NEAR(rotatedBounds.lowerBound.y, 3.0f, kTolerance);
    EXPECT_NEAR(rotatedBounds.upperBound.x, 3.5f, kTolerance);
    EXPECT_NEAR(rotatedBounds.upperBound.y, 5.0f, kTolerance);
    EXPECT_TRUE(rotatedBounds.IsValid());

    // Unit square rotated 45 degrees: its corners lie at distance sqrt(2) on the
    // axes, so the AABB is [-sqrt(2), sqrt(2)]^2.
    PolygonShape square;
    ASSERT_TRUE(p2::SetPolygonAsBox(square, 1.0f, 1.0f));
    const Transform diagonal{Vec2{0.0f, 0.0f}, Rot(p2::kPi / 4.0f)};
    const AABB diagonalBounds = p2::ComputePolygonAABB(square, diagonal);
    const float expectedExtent = std::sqrt(2.0f);
    EXPECT_NEAR(diagonalBounds.lowerBound.x, -expectedExtent, kTolerance);
    EXPECT_NEAR(diagonalBounds.lowerBound.y, -expectedExtent, kTolerance);
    EXPECT_NEAR(diagonalBounds.upperBound.x, expectedExtent, kTolerance);
    EXPECT_NEAR(diagonalBounds.upperBound.y, expectedExtent, kTolerance);
}

TEST(PolygonShape, ValidationRejectsDegenerateInput) {
    PolygonShape shape;
    const Vec2 square[4] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f}, Vec2{-1.0f, -1.0f}};

    // Null pointer and count outside [3, kMaxPolygonVertices].
    EXPECT_FALSE(p2::SetPolygon(shape, nullptr, 4));
    EXPECT_FALSE(p2::SetPolygon(shape, square, 2));

    Vec2 nine[9];
    for (int32_t i = 0; i < 9; ++i) {
        const float angle = p2::kTwoPi * static_cast<float>(i) / 9.0f;
        nine[i] = Vec2{std::cos(angle), std::sin(angle)};
    }
    EXPECT_FALSE(p2::SetPolygon(shape, nine, 9));

    // Non-finite coordinate.
    Vec2 withNaN[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f},
                       Vec2{0.0f, std::numeric_limits<float>::quiet_NaN()}};
    EXPECT_FALSE(p2::SetPolygon(shape, withNaN, 3));

    // Adjacent vertices closer than kLinearSlop (0.001 < 0.005): not silently
    // merged, rejected so the caller sees the bad data.
    const Vec2 tooClose[5] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, -0.999f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f},
                              Vec2{-1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, tooClose, 5));

    // Redundant collinear vertex on the right edge: (1,0) adds no area.
    const Vec2 collinear[5] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f},
                               Vec2{-1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, collinear, 5));

    // Concave L shape (the turn at (1,1) is negative for CCW traversal).
    const Vec2 concave[6] = {Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, Vec2{2.0f, 1.0f},
                             Vec2{1.0f, 1.0f}, Vec2{1.0f, 2.0f}, Vec2{0.0f, 2.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, concave, 6));

    // Clockwise winding: the shoelace area is negative.
    const Vec2 clockwise[4] = {Vec2{-1.0f, -1.0f}, Vec2{-1.0f, 1.0f}, Vec2{1.0f, 1.0f}, Vec2{1.0f, -1.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, clockwise, 4));

    // Zero area (three collinear points).
    const Vec2 flat[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{2.0f, 0.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, flat, 3));

    // Near-zero area: a sliver thinner than the contact slop squared is rejected
    // by the same guard that protects the centroid division.
    const Vec2 sliver[3] = {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.5f, 1.0e-6f}};
    EXPECT_FALSE(p2::SetPolygon(shape, sliver, 3));

    // Every failure left the shape in the invalid state.
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());
}

TEST(PolygonShape, RejectedInputInvalidatesAPreviouslyValidShape) {
    // A failed rebuild must not leave the old geometry half-replaced: the contract
    // is that rejection puts the shape back into the invalid state.
    PolygonShape shape;
    ASSERT_TRUE(p2::SetPolygonAsBox(shape, 1.0f, 1.0f));
    ASSERT_TRUE(shape.IsValid());

    const Vec2 concave[6] = {Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, Vec2{2.0f, 1.0f},
                             Vec2{1.0f, 1.0f}, Vec2{1.0f, 2.0f}, Vec2{0.0f, 2.0f}};
    EXPECT_FALSE(p2::SetPolygon(shape, concave, 6));
    EXPECT_EQ(shape.count, 0);
    EXPECT_FALSE(shape.IsValid());
}

TEST(PolygonShape, SetPolygonAsBoxRejectsInvalidParameters) {
    PolygonShape shape;
    const Vec2 square[4] = {Vec2{1.0f, -1.0f}, Vec2{1.0f, 1.0f}, Vec2{-1.0f, 1.0f}, Vec2{-1.0f, -1.0f}};
    ASSERT_TRUE(p2::SetPolygon(shape, square, 4));

    EXPECT_FALSE(p2::SetPolygonAsBox(shape, 0.0f, 1.0f));
    EXPECT_FALSE(shape.IsValid());
    EXPECT_FALSE(p2::SetPolygonAsBox(shape, 1.0f, -1.0f));
    EXPECT_FALSE(p2::SetPolygonAsBox(shape, std::numeric_limits<float>::quiet_NaN(), 1.0f));
    EXPECT_FALSE(
        p2::SetPolygonAsBox(shape, 1.0f, 1.0f, Vec2{0.0f, 0.0f}, std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(
        p2::SetPolygonAsBox(shape, 1.0f, 1.0f, Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f}, 0.0f));
    EXPECT_FALSE(shape.IsValid());
}

TEST(PolygonShape, MaxVertexCountOctagonIsAccepted) {
    // Regular octagon with circumradius 1: eight vertices 45 degrees apart. The
    // area of a regular n-gon is 0.5*n*R^2*sin(2*pi/n), so here
    //   0.5*8*sin(pi/4) = 4*sqrt(2)/2 = 2*sqrt(2) = 2.828427.
    constexpr int32_t kCount = 8;
    Vec2 vertices[kCount];
    for (int32_t i = 0; i < kCount; ++i) {
        const float angle = p2::kTwoPi * static_cast<float>(i) / static_cast<float>(kCount);
        vertices[i] = Vec2{std::cos(angle), std::sin(angle)};
    }

    PolygonShape octagon;
    ASSERT_TRUE(p2::SetPolygon(octagon, vertices, kCount));
    EXPECT_EQ(octagon.count, kCount);
    EXPECT_TRUE(octagon.IsValid());

    const float expectedArea = 2.0f * std::sqrt(2.0f);
    const MassData mass = p2::ComputePolygonMass(octagon, 1.0f);
    EXPECT_NEAR(mass.mass, expectedArea, 1.0e-4f);
    EXPECT_NEAR(mass.center.x, 0.0f, 1.0e-5f);
    EXPECT_NEAR(mass.center.y, 0.0f, 1.0e-5f);
}
