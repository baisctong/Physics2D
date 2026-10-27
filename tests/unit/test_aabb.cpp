// Unit tests for p2::AABB (Week 1 D4, design document P03).
//
// The overlap convention under test is inclusive: boxes that merely touch on a
// face or corner DO overlap, because a contact at zero separation is a real
// contact state and the broad phase must not drop that pair on the frame a
// stack settles. Every expected value is computed analytically in the comment.

#include <gtest/gtest.h>

#include "physics2d/Math/AABB.h"

#include <cmath>
#include <limits>

namespace {

using p2::AABB;
using p2::Vec2;

constexpr float kTolerance = 1.0e-6f;

struct GridCase {
    float dx;
    float dy;
    bool expectedOverlap;
};

} // namespace

TEST(AABB, AccessorsOnKnownBox) {
    // Box [1, 5] x [2, 8]: width = 5-1 = 4, height = 8-2 = 6.
    const AABB box{Vec2{1.0f, 2.0f}, Vec2{5.0f, 8.0f}};

    EXPECT_FLOAT_EQ(box.GetWidth(), 4.0f);
    EXPECT_FLOAT_EQ(box.GetHeight(), 6.0f);
    EXPECT_FLOAT_EQ(box.GetPerimeter(), 2.0f * (4.0f + 6.0f));

    // Center = ((1+5)/2, (2+8)/2) = (3, 5); extents = (4/2, 6/2) = (2, 3).
    const Vec2 center = box.GetCenter();
    EXPECT_FLOAT_EQ(center.x, 3.0f);
    EXPECT_FLOAT_EQ(center.y, 5.0f);
    const Vec2 extents = box.GetExtents();
    EXPECT_FLOAT_EQ(extents.x, 2.0f);
    EXPECT_FLOAT_EQ(extents.y, 3.0f);
}

TEST(AABB, DefaultBoxIsDegeneratePointAtOrigin) {
    const AABB box;
    EXPECT_TRUE((box.lowerBound == Vec2{0.0f, 0.0f}));
    EXPECT_TRUE((box.upperBound == Vec2{0.0f, 0.0f}));
    EXPECT_FLOAT_EQ(box.GetWidth(), 0.0f);
    EXPECT_FLOAT_EQ(box.GetHeight(), 0.0f);
    EXPECT_FLOAT_EQ(box.GetPerimeter(), 0.0f);
    // A zero-size box is a legitimate proxy for a point shape, so it is valid.
    EXPECT_TRUE(box.IsValid());
}

TEST(AABB, OverlapNinePositionGrid) {
    // A = [0, 2] x [0, 2]. B is a 1x1 box whose lower corner is (0.5 + dx, 0.5 + dy),
    // so B = [0.5+dx, 1.5+dx] x [0.5+dy, 1.5+dy].
    //
    // Per axis:
    //   dx = -2   -> B spans [-1.5, -0.5], strictly left of A  (separated)
    //   dx = 1.5  -> B spans [2.0, 3.0],   shares A's right face (touching)
    //   dx = 1    -> B spans [1.5, 2.5],   overlaps A            (penetrating)
    // A pair overlaps iff neither axis is separated, i.e. iff dx != -2 and
    // dy != -2. The 3x3 core grid below uses (dx, dy) in {-2, 0, 1.5}, which
    // covers separated / penetrating / face-touching states on each axis; the
    // extra rows add the remaining penetrating combinations and a diagonal
    // separated case. This is the nine-relative-position requirement of D4.
    const AABB a{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}};
    const GridCase grid[] = {
        // dx       dy       expected overlap
        {-2.0f, -2.0f, false}, // separated on both axes (diagonal)
        {-2.0f, 0.0f, false},  // separated on x, aligned on y
        {-2.0f, 1.5f, false},  // separated on x, touching on y
        {0.0f, -2.0f, false},  // aligned on x, separated on y
        {0.0f, 0.0f, true},    // penetrating on both axes (B inside A)
        {0.0f, 1.5f, true},    // aligned on x, touching on y (B.lower.y == A.upper.y)
        {1.5f, -2.0f, false},  // touching on x, separated on y
        {1.5f, 0.0f, true},    // touching on x (B.lower.x == A.upper.x)
        {1.5f, 1.5f, true},    // touching on both axes (corner contact)
        {1.0f, 1.0f, true},    // penetrating on both axes
        {1.0f, -2.0f, false},  // penetrating on x, separated on y
        {-2.0f, 1.0f, false},  // separated on x, penetrating on y
    };

    for (const GridCase& c : grid) {
        const AABB b{Vec2{0.5f + c.dx, 0.5f + c.dy}, Vec2{1.5f + c.dx, 1.5f + c.dy}};
        EXPECT_EQ(p2::Overlaps(a, b), c.expectedOverlap) << "dx=" << c.dx << " dy=" << c.dy;
        // Overlap is a symmetric relation.
        EXPECT_EQ(p2::Overlaps(b, a), c.expectedOverlap) << "swapped dx=" << c.dx << " dy=" << c.dy;
    }
}

TEST(AABB, FaceAndCornerTouchingCountsAsOverlap) {
    const AABB a{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}};

    // Each of these boxes touches A on exactly one face:
    //   left:  [-1, 0] x [0.5, 1.5]   (B.upper.x == A.lower.x == 0)
    //   right: [2, 3]  x [0.5, 1.5]   (B.lower.x == A.upper.x == 2)
    //   bottom: [0.5, 1.5] x [-1, 0]
    //   top:    [0.5, 1.5] x [2, 3]
    const AABB touches[] = {
        {Vec2{-1.0f, 0.5f}, Vec2{0.0f, 1.5f}},
        {Vec2{2.0f, 0.5f}, Vec2{3.0f, 1.5f}},
        {Vec2{0.5f, -1.0f}, Vec2{1.5f, 0.0f}},
        {Vec2{0.5f, 2.0f}, Vec2{1.5f, 3.0f}},
    };
    for (const AABB& touching : touches) {
        EXPECT_TRUE(p2::Overlaps(a, touching));
        EXPECT_TRUE(p2::Overlaps(touching, a));
    }

    // Corner contact at (0, 0): B = [-1, 0] x [-1, 0]. Inclusive rectangle
    // overlap treats it as overlapping (Box2D's b2AABB follows the same rule).
    const AABB corner{Vec2{-1.0f, -1.0f}, Vec2{0.0f, 0.0f}};
    EXPECT_TRUE(p2::Overlaps(a, corner));

    // A degenerate (zero-size) box strictly inside still overlaps A.
    const AABB point{Vec2{1.0f, 1.0f}, Vec2{1.0f, 1.0f}};
    EXPECT_TRUE(p2::Overlaps(a, point));

    // Strictly separated boxes must not overlap: a gap of even a tiny amount
    // (here the gap is 0.001 on x) is separation.
    const AABB justOutside{Vec2{-2.0f, 0.5f}, Vec2{-0.001f, 1.5f}};
    EXPECT_FALSE(p2::Overlaps(a, justOutside));
}

TEST(AABB, ContainsBoxIsInclusive) {
    // Outer = [0, 4] x [0, 4].
    const AABB outer{Vec2{0.0f, 0.0f}, Vec2{4.0f, 4.0f}};

    // Strictly inside: [1, 3] x [1, 3].
    EXPECT_TRUE((p2::Contains(outer, AABB{Vec2{1.0f, 1.0f}, Vec2{3.0f, 3.0f}})));
    // Identical boxes are contained.
    EXPECT_TRUE(p2::Contains(outer, outer));
    // Sharing edges is still containment (inclusive bounds): [0, 2] x [0, 4].
    EXPECT_TRUE((p2::Contains(outer, AABB{Vec2{0.0f, 0.0f}, Vec2{2.0f, 4.0f}})));
    // A zero-size box at the corner is contained.
    EXPECT_TRUE((p2::Contains(outer, AABB{Vec2{4.0f, 4.0f}, Vec2{4.0f, 4.0f}})));

    // Sticking out on the left: [-1, 3] x [1, 3].
    EXPECT_FALSE((p2::Contains(outer, AABB{Vec2{-1.0f, 1.0f}, Vec2{3.0f, 3.0f}})));
    // Sticking out on the top: [1, 3] x [1, 5].
    EXPECT_FALSE((p2::Contains(outer, AABB{Vec2{1.0f, 1.0f}, Vec2{3.0f, 5.0f}})));
    // The outer box is not contained in a smaller inner box.
    EXPECT_FALSE((p2::Contains(AABB{Vec2{1.0f, 1.0f}, Vec2{3.0f, 3.0f}}, outer)));
}

TEST(AABB, ContainsPointIsInclusive) {
    // Box = [0, 4] x [0, 4].
    const AABB box{Vec2{0.0f, 0.0f}, Vec2{4.0f, 4.0f}};

    EXPECT_TRUE((p2::Contains(box, Vec2{2.0f, 2.0f}))); // interior
    EXPECT_TRUE((p2::Contains(box, Vec2{0.0f, 0.0f}))); // lower corner
    EXPECT_TRUE((p2::Contains(box, Vec2{4.0f, 4.0f}))); // upper corner
    EXPECT_TRUE((p2::Contains(box, Vec2{2.0f, 0.0f}))); // on an edge

    EXPECT_FALSE((p2::Contains(box, Vec2{-0.001f, 2.0f})));
    EXPECT_FALSE((p2::Contains(box, Vec2{2.0f, 4.001f})));
    EXPECT_FALSE((p2::Contains(box, Vec2{5.0f, 5.0f})));
}

TEST(AABB, UnionContainsBothOperands) {
    // A = [0, 2] x [0, 2], B = [1, 4] x [1, 3].
    const AABB a{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}};
    const AABB b{Vec2{1.0f, 1.0f}, Vec2{4.0f, 3.0f}};

    // Union = [min(0,1), max(2,4)] x [min(0,1), max(2,3)] = [0, 4] x [0, 3].
    const AABB merged = p2::Union(a, b);
    EXPECT_TRUE((merged.lowerBound == Vec2{0.0f, 0.0f}));
    EXPECT_TRUE((merged.upperBound == Vec2{4.0f, 3.0f}));

    // The union contains both inputs and is commutative.
    EXPECT_TRUE(p2::Contains(merged, a));
    EXPECT_TRUE(p2::Contains(merged, b));
    const AABB swapped = p2::Union(b, a);
    EXPECT_TRUE(swapped.lowerBound == merged.lowerBound);
    EXPECT_TRUE(swapped.upperBound == merged.upperBound);

    // Union is idempotent.
    const AABB self = p2::Union(a, a);
    EXPECT_TRUE(self.lowerBound == a.lowerBound);
    EXPECT_TRUE(self.upperBound == a.upperBound);

    // Union never shrinks: its area/perimeter is at least that of each operand.
    EXPECT_GE(merged.GetPerimeter(), a.GetPerimeter());
    EXPECT_GE(merged.GetPerimeter(), b.GetPerimeter());
    EXPECT_GE(merged.GetWidth() * merged.GetHeight(), a.GetWidth() * a.GetHeight());
}

TEST(AABB, ExtendGrowsAreaMonotonically) {
    // Start from [0, 2] x [0, 2] and grow by ten increments of 0.1.
    // After i calls the box must be [-0.1*i, 2 + 0.1*i]^2, so its side is
    // 2 + 0.2*i and its area is (2 + 0.2*i)^2, which is strictly increasing.
    AABB box{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}};
    constexpr float kIncrement = 0.1f;
    constexpr int kSteps = 10;
    float previousArea = box.GetWidth() * box.GetHeight();

    for (int i = 1; i <= kSteps; ++i) {
        box.Extend(kIncrement);

        const float expectedOffset = kIncrement * static_cast<float>(i);
        const float expectedSide = 2.0f + 2.0f * expectedOffset;
        // 1e-5 tolerance: ten cumulative +=/-= on the bounds, magnitudes are
        // O(3) so the accumulated rounding is a few ulps.
        EXPECT_NEAR(box.lowerBound.x, -expectedOffset, 1.0e-5f);
        EXPECT_NEAR(box.lowerBound.y, -expectedOffset, 1.0e-5f);
        EXPECT_NEAR(box.upperBound.x, 2.0f + expectedOffset, 1.0e-5f);
        EXPECT_NEAR(box.upperBound.y, 2.0f + expectedOffset, 1.0e-5f);

        const float area = box.GetWidth() * box.GetHeight();
        EXPECT_NEAR(area, expectedSide * expectedSide, 1.0e-5f);
        // Extend must never shrink the box -> area is monotonic non-decreasing.
        EXPECT_GE(area, previousArea);
        previousArea = area;
    }
}

TEST(AABB, ExtendClampsNegativeMargin) {
    // Contract: margin must be >= 0. A negative argument is clamped to a no-op
    // so Extend stays monotonic even if a caller passes a "shrink" by mistake.
    AABB box{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}};
    box.Extend(-0.5f);
    EXPECT_TRUE((box.lowerBound == Vec2{0.0f, 0.0f}));
    EXPECT_TRUE((box.upperBound == Vec2{2.0f, 2.0f}));

    // A zero margin is also a no-op.
    box.Extend(0.0f);
    EXPECT_TRUE((box.lowerBound == Vec2{0.0f, 0.0f}));
}

TEST(AABB, IsValidRejectsInvertedAndNonFiniteBoxes) {
    EXPECT_TRUE((AABB{Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}}).IsValid());

    // Inverted on either axis: no interior, rejected.
    EXPECT_FALSE((AABB{Vec2{2.0f, 0.0f}, Vec2{1.0f, 2.0f}}).IsValid());
    EXPECT_FALSE((AABB{Vec2{0.0f, 2.0f}, Vec2{2.0f, 1.0f}}).IsValid());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FALSE((AABB{Vec2{nan, 0.0f}, Vec2{1.0f, 1.0f}}).IsValid());
    EXPECT_FALSE((AABB{Vec2{0.0f, 0.0f}, Vec2{inf, 1.0f}}).IsValid());
    EXPECT_FALSE((AABB{Vec2{0.0f, -inf}, Vec2{1.0f, 1.0f}}).IsValid());

    // Floating-point extreme but finite bounds are valid.
    const float max = std::numeric_limits<float>::max();
    EXPECT_TRUE((AABB{Vec2{-max, -max}, Vec2{max, max}}).IsValid());
}
