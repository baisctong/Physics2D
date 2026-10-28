// Unit tests for p2::ClipSegmentToLine (Week 4 D4 / P07).
//
// The clipping primitive is the geometric core of the SAT manifold generator, so
// every case below is hand-computed: the half-plane convention, which endpoints
// survive, the analytic intersection value, the exact-boundary case, the
// parallel/degenerate case (no division by a near-zero denominator) and
// non-axis-aligned normals. The convention under test is
//   a point survives when Dot(normal, x) <= offset
// with `normal` of unit length.
//
// Note on the return value: a segment with one endpoint inside and one outside
// yields *two* points - the surviving endpoint plus the boundary crossing. This is
// what the header specifies ("the other is replaced by the intersection point");
// the 0/1/2 result only returns 1 for an exact boundary touch (an endpoint exactly
// on the line with the other one discarded), where no crossing exists.

#include <gtest/gtest.h>

#include "physics2d/Collision/Clipping.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <cstdint>

namespace {

using p2::ClipSegmentToLine;
using p2::Vec2;

constexpr float kTolerance = 1.0e-6f;

void ExpectVec2Near(Vec2 actual, Vec2 expected, float tolerance = kTolerance) {
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
}

// A NaN anywhere in the narrow phase poisons a whole island within one frame, so
// every output point of every case is checked for finiteness.
void ExpectFinite(Vec2 value) {
    EXPECT_TRUE(std::isfinite(value.x)) << "x = " << value.x;
    EXPECT_TRUE(std::isfinite(value.y)) << "y = " << value.y;
}

} // namespace

TEST(ClipSegmentToLine, KeepsBothEndpointsWhenTheWholeSegmentIsInside) {
    // Line y = 1 (normal (0,1), offset 1). Both endpoints have y = -1, so their
    // signed distances are -2 <= 0: nothing is clipped and the input order is
    // preserved (the manifold generator relies on that order for stable points).
    const Vec2 input[2] = {{0.0f, -1.0f}, {1.0f, -1.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 1.0f);

    EXPECT_EQ(count, 2);
    EXPECT_EQ(output[0], input[0]);
    EXPECT_EQ(output[1], input[1]);
    ExpectFinite(output[0]);
    ExpectFinite(output[1]);
}

TEST(ClipSegmentToLine, ReplacesTheOutsideEndpointWithTheAnalyticIntersection) {
    // Line y = 0 (normal (0,1), offset 0). Segment (-1,-1) -> (1,1):
    //   distance0 = Dot((0,1), (-1,-1)) - 0 = -1  (inside)
    //   distance1 = Dot((0,1), (1,1)) - 0  = +1  (outside)
    //   interpolation = distance0 / (distance0 - distance1) = -1 / -2 = 0.5
    //   intersection = (-1,-1) + 0.5*((1,1) - (-1,-1)) = (0,0)
    const Vec2 input[2] = {{-1.0f, -1.0f}, {1.0f, 1.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 0.0f);

    EXPECT_EQ(count, 2);
    // Surviving interior endpoint first (in input order), then the intersection.
    ExpectVec2Near(output[0], Vec2{-1.0f, -1.0f});
    ExpectVec2Near(output[1], Vec2{0.0f, 0.0f});
    ExpectFinite(output[1]);
}

TEST(ClipSegmentToLine, EmitsTheInteriorEndpointFirstRegardlessOfInputOrder) {
    // Same geometry as above with the endpoints swapped:
    //   distance0 = +1 (outside), distance1 = -1 (inside)
    //   interpolation = +1 / (+1 - (-1)) = 0.5 -> intersection = (0,0)
    // The primitive writes surviving endpoints in input order, so the interior
    // endpoint (-1,-1) (input[1]) is written first and the intersection second.
    const Vec2 input[2] = {{1.0f, 1.0f}, {-1.0f, -1.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 0.0f);

    EXPECT_EQ(count, 2);
    ExpectVec2Near(output[0], Vec2{-1.0f, -1.0f});
    ExpectVec2Near(output[1], Vec2{0.0f, 0.0f});
}

TEST(ClipSegmentToLine, ReturnsZeroWhenBothEndpointsAreOnTheDiscardedSide) {
    // Line x = 0 (normal (1,0), offset 0), segment (1,0) -> (2,0): distances
    // +1 and +2, both > 0, so nothing survives and no intersection is created.
    const Vec2 input[2] = {{1.0f, 0.0f}, {2.0f, 0.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{1.0f, 0.0f}, 0.0f);

    EXPECT_EQ(count, 0);
}

TEST(ClipSegmentToLine, KeepsASinglePointWhenAnEndpointLiesExactlyOnTheBoundary) {
    // Line y = 0. Endpoint (0,0) has distance exactly 0, so it survives; (1,1) has
    // distance +1 and is discarded. The product is 0 (not < 0), so there is no
    // crossing and the result is exactly one point: the boundary touch case.
    const Vec2 input[2] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 0.0f);

    EXPECT_EQ(count, 1);
    ExpectVec2Near(output[0], Vec2{0.0f, 0.0f}, 0.0f);
}

TEST(ClipSegmentToLine, KeepsASegmentThatLiesExactlyOnTheBoundary) {
    // Both endpoints have distance exactly 0: both survive, no intersection is
    // synthesised (the product test keeps the denominator out of the division).
    const Vec2 input[2] = {{0.0f, 0.0f}, {3.0f, 0.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 0.0f);

    EXPECT_EQ(count, 2);
    EXPECT_EQ(output[0], input[0]);
    EXPECT_EQ(output[1], input[1]);
}

TEST(ClipSegmentToLine, DoesNotDivideByZeroForASegmentParallelToTheBoundary) {
    // Both endpoints at y = 2, i.e. parallel to the line y = 1 with distance
    // distance0 - distance1 == 0. The product test (both distances positive) keeps
    // the code out of the intersection branch, so no 0/0 NaN can be produced.
    const Vec2 parallelOutside[2] = {{0.0f, 2.0f}, {3.0f, 2.0f}};
    Vec2 output[2] = {};
    const int32_t outsideCount = ClipSegmentToLine(parallelOutside, output, Vec2{0.0f, 1.0f}, 1.0f);
    EXPECT_EQ(outsideCount, 0);

    // Parallel and exactly on the boundary: both points survive untouched.
    const Vec2 parallelOnLine[2] = {{-2.0f, 1.0f}, {2.0f, 1.0f}};
    const int32_t onLineCount = ClipSegmentToLine(parallelOnLine, output, Vec2{0.0f, 1.0f}, 1.0f);
    EXPECT_EQ(onLineCount, 2);
    EXPECT_EQ(output[0], parallelOnLine[0]);
    EXPECT_EQ(output[1], parallelOnLine[1]);
    ExpectFinite(output[0]);
    ExpectFinite(output[1]);
}

TEST(ClipSegmentToLine, AppliesTheOffset) {
    // Line x = 2 (normal (1,0), offset 2). Segment (1,5) -> (3,5):
    //   distance0 = 1 - 2 = -1 (inside), distance1 = 3 - 2 = +1 (outside)
    //   interpolation = -1 / (-1 - 1) = 0.5
    //   intersection = (1,5) + 0.5*((3,5) - (1,5)) = (2,5)
    const Vec2 input[2] = {{1.0f, 5.0f}, {3.0f, 5.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{1.0f, 0.0f}, 2.0f);

    EXPECT_EQ(count, 2);
    ExpectVec2Near(output[0], Vec2{1.0f, 5.0f});
    ExpectVec2Near(output[1], Vec2{2.0f, 5.0f});
}

TEST(ClipSegmentToLine, ClipsAgainstANonAxisAlignedBoundary) {
    // Diagonal boundary x + y = 0, i.e. normal = (1,1)/sqrt(2), offset = 0.
    //   point (1,0):  Dot =  1/sqrt(2) = +0.7071 (outside)
    //   point (-1,0): Dot = -1/sqrt(2) = -0.7071 (inside)
    //   interpolation = 0.7071 / (0.7071 + 0.7071) = 0.5
    //   intersection = (1,0) + 0.5*((-1,0) - (1,0)) = (0,0)
    constexpr float kInvSqrt2 = 0.70710678f;
    const Vec2 normal{kInvSqrt2, kInvSqrt2};
    const Vec2 input[2] = {{1.0f, 0.0f}, {-1.0f, 0.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, normal, 0.0f);

    EXPECT_EQ(count, 2);
    ExpectVec2Near(output[0], Vec2{-1.0f, 0.0f});
    ExpectVec2Near(output[1], Vec2{0.0f, 0.0f});
}

TEST(ClipSegmentToLine, HandlesANegativeNormal) {
    // normal (-1,0), offset -2 is the half-plane -x <= -2, i.e. x >= 2: the kept
    // side is the opposite one, which is exactly how SAT passes the reference
    // face's start plane (normal = -tangent).
    //   (1,0): distance = -1 + 2 = +1 (outside)
    //   (3,0): distance = -3 + 2 = -1 (inside)
    //   intersection = (2,0)
    const Vec2 input[2] = {{1.0f, 0.0f}, {3.0f, 0.0f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{-1.0f, 0.0f}, -2.0f);

    EXPECT_EQ(count, 2);
    ExpectVec2Near(output[0], Vec2{3.0f, 0.0f});
    ExpectVec2Near(output[1], Vec2{2.0f, 0.0f});
}

TEST(ClipSegmentToLine, DoesNotProduceNaNForExtremeCoordinates) {
    // Very long segment crossing the line y = 0: distances -1e30 and +1e30. Their
    // product overflows to -inf, which is still < 0, so the intersection is taken
    // with interpolation 0.5 (the ratio of two huge numbers stays finite).
    const Vec2 input[2] = {{1.0e30f, -1.0e30f}, {-1.0e30f, 1.0e30f}};
    Vec2 output[2] = {};
    const int32_t count = ClipSegmentToLine(input, output, Vec2{0.0f, 1.0f}, 0.0f);

    EXPECT_EQ(count, 2);
    ExpectFinite(output[0]);
    ExpectFinite(output[1]);
    ExpectVec2Near(output[0], Vec2{1.0e30f, -1.0e30f}, 1.0e24f);
    ExpectVec2Near(output[1], Vec2{0.0f, 0.0f}, 1.0e-3f);
}
