// Unit tests for p2::Vec2 (Week 1 D2, design document P02).
//
// Every expected value is derived analytically and the arithmetic is written in
// the comment next to the assertion, so the test can be reviewed independently
// of the implementation it exercises. EXPECT_NEAR is used only where float
// rounding is unavoidable, and the tolerance is justified in place.

#include <gtest/gtest.h>

#include "physics2d/Math/Vec2.h"

#include <cstdint>
#include <limits>
#include <random>

#ifdef PHYSICS2D_USE_EIGEN
// Eigen is third-party code included solely for independent cross-validation;
// warnings are suppressed locally so the project's own /W4 discipline is not
// weakened for code we do not own.
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#include <Eigen/Dense>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#endif

namespace {

using p2::Vec2;

// Float has ~7 decimal digits of precision; 1e-6 leaves one digit of headroom
// for the handful of rounding steps in each operation.
constexpr float kTolerance = 1.0e-6f;

// Fixed seed: a failure in a randomised section must be reproducible from the
// seed alone, the same determinism argument the engine applies to itself.
// Only the Eigen cross-validation section uses randomness, so the helper is
// compiled out when cross-validation is disabled (keeps /W4 clean either way).
#ifdef PHYSICS2D_USE_EIGEN
std::mt19937 MakeRng() {
    return std::mt19937(0xC0FFEEu);
}
#endif

} // namespace

TEST(Vec2, DefaultConstructionIsZero) {
    const Vec2 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
}

TEST(Vec2, AdditionAndSubtraction) {
    const Vec2 a{3.0f, -2.0f};
    const Vec2 b{-1.0f, 5.0f};

    // (3, -2) + (-1, 5) = (3-1, -2+5) = (2, 3)
    const Vec2 sum = a + b;
    EXPECT_FLOAT_EQ(sum.x, 2.0f);
    EXPECT_FLOAT_EQ(sum.y, 3.0f);

    // (3, -2) - (-1, 5) = (3+1, -2-5) = (4, -7)
    const Vec2 diff = a - b;
    EXPECT_FLOAT_EQ(diff.x, 4.0f);
    EXPECT_FLOAT_EQ(diff.y, -7.0f);

    // Compound assignment must match the binary operator.
    Vec2 compound = a;
    compound += b;
    EXPECT_TRUE(compound == sum);
    compound = a;
    compound -= b;
    EXPECT_TRUE(compound == diff);
}

TEST(Vec2, ScalarMultiplicationAndDivision) {
    const Vec2 a{3.0f, -2.0f};

    // 2 * (3, -2) = (6, -4); both operand orders must agree.
    EXPECT_TRUE((2.0f * a == Vec2{6.0f, -4.0f}));
    EXPECT_TRUE((a * 2.0f == Vec2{6.0f, -4.0f}));

    // (3, -2) / 2 = (1.5, -1); 3 / 2 and -2 / 2 are exactly representable.
    EXPECT_TRUE((a / 2.0f == Vec2{1.5f, -1.0f}));

    // Component-wise scalar/vector: 12 / (3, -2) = (4, -6).
    EXPECT_TRUE((12.0f / a == Vec2{4.0f, -6.0f}));

    // Compound forms must agree with the binary ones.
    Vec2 compound = a;
    compound *= 2.0f;
    EXPECT_TRUE(compound == a * 2.0f);
    compound = a;
    compound /= 2.0f;
    EXPECT_TRUE(compound == a / 2.0f);
}

TEST(Vec2, UnaryNegationAndEquality) {
    const Vec2 a{3.0f, -2.0f};

    // -(3, -2) = (-3, 2), and negating twice is the identity.
    const Vec2 negated = -a;
    EXPECT_TRUE((negated == Vec2{-3.0f, 2.0f}));
    EXPECT_TRUE(-negated == a);

    EXPECT_TRUE(a == a);
    EXPECT_FALSE(a != a);
    EXPECT_TRUE((a != Vec2{3.0f, -2.5f}));
}

TEST(Vec2, DotProduct) {
    // (3, 4) . (-2, 5) = 3*-2 + 4*5 = -6 + 20 = 14
    EXPECT_FLOAT_EQ(p2::Dot(Vec2{3.0f, 4.0f}, Vec2{-2.0f, 5.0f}), 14.0f);

    // Perpendicular vectors: (1, 0) . (0, 1) = 0
    EXPECT_FLOAT_EQ(p2::Dot(Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}), 0.0f);

    // A vector with itself: (1, 2) . (1, 2) = 1 + 4 = 5
    EXPECT_FLOAT_EQ(p2::Dot(Vec2{1.0f, 2.0f}, Vec2{1.0f, 2.0f}), 5.0f);

    // Dot is commutative and bilinear with scalars: (2a).b = 2(a.b)
    const Vec2 a{3.0f, 4.0f};
    const Vec2 b{-2.0f, 5.0f};
    EXPECT_FLOAT_EQ(p2::Dot(a, b), p2::Dot(b, a));
    EXPECT_FLOAT_EQ(p2::Dot(a * 2.0f, b), 2.0f * p2::Dot(a, b));
}

TEST(Vec2, CrossVectorVectorIsTheZComponent) {
    // Cross(a, b) = a.x*b.y - a.y*b.x (z of the 3D cross product).
    // (3, 4) x (-2, 5) = 3*5 - 4*(-2) = 15 + 8 = 23 > 0 (b is CCW from a).
    EXPECT_FLOAT_EQ(p2::Cross(Vec2{3.0f, 4.0f}, Vec2{-2.0f, 5.0f}), 23.0f);

    // Swapping the operands negates the result: (-2, 5) x (3, 4) = -2*4 - 5*3
    // = -8 - 15 = -23.
    EXPECT_FLOAT_EQ(p2::Cross(Vec2{-2.0f, 5.0f}, Vec2{3.0f, 4.0f}), -23.0f);

    // Parallel vectors span no area: (2, 3) x (4, 6) = 2*6 - 3*4 = 12 - 12 = 0.
    EXPECT_FLOAT_EQ(p2::Cross(Vec2{2.0f, 3.0f}, Vec2{4.0f, 6.0f}), 0.0f);

    // A vector with itself is always zero: (1, 2) x (1, 2) = 1*2 - 2*1 = 0.
    EXPECT_FLOAT_EQ(p2::Cross(Vec2{1.0f, 2.0f}, Vec2{1.0f, 2.0f}), 0.0f);
}

TEST(Vec2, CrossVectorScalar) {
    // Cross(a, s) = (s*a.y, -s*a.x).
    // (2, 3) x 5 = (5*3, -5*2) = (15, -10)
    EXPECT_TRUE((p2::Cross(Vec2{2.0f, 3.0f}, 5.0f) == Vec2{15.0f, -10.0f}));

    // (2, 3) x (-2) = (-2*3, -(-2)*2) = (-6, 4)
    EXPECT_TRUE((p2::Cross(Vec2{2.0f, 3.0f}, -2.0f) == Vec2{-6.0f, 4.0f}));

    // The zero vector is absorbing: (0, 0) x 7 = (0, 0).
    EXPECT_TRUE((p2::Cross(Vec2{0.0f, 0.0f}, 7.0f) == Vec2{0.0f, 0.0f}));

    // Cross(1, a) == Skew(a) and Cross(a, s) == -Cross(s, a).
    const Vec2 a{2.0f, 3.0f};
    EXPECT_TRUE(p2::Cross(1.0f, a) == p2::Skew(a));
    EXPECT_TRUE(p2::Cross(a, 5.0f) == -p2::Cross(5.0f, a));
}

TEST(Vec2, CrossScalarVector) {
    // Cross(s, a) = (-s*a.y, s*a.x).
    // 5 x (2, 3) = (-5*3, 5*2) = (-15, 10)
    EXPECT_TRUE((p2::Cross(5.0f, Vec2{2.0f, 3.0f}) == Vec2{-15.0f, 10.0f}));

    // (-2) x (2, 3) = (-(-2)*3, (-2)*2) = (6, -4)
    EXPECT_TRUE((p2::Cross(-2.0f, Vec2{2.0f, 3.0f}) == Vec2{6.0f, -4.0f}));

    // 7 x (0, 0) = (0, 0).
    EXPECT_TRUE((p2::Cross(7.0f, Vec2{0.0f, 0.0f}) == Vec2{0.0f, 0.0f}));
}

TEST(Vec2, SkewIsAQuarterTurnCounterClockwise) {
    // Skew(v) = (-v.y, v.x) is +90 degrees CCW.
    // (3, 4) -> (-4, 3)
    EXPECT_TRUE((p2::Skew(Vec2{3.0f, 4.0f}) == Vec2{-4.0f, 3.0f}));

    // Rotating twice is a half turn: Skew(Skew(v)) == -v.
    const Vec2 v{3.0f, 4.0f};
    EXPECT_TRUE(p2::Skew(p2::Skew(v)) == -v);

    // Skew preserves length: |(-4, 3)| = sqrt(16 + 9) = 5 = |(3, 4)|.
    EXPECT_FLOAT_EQ(p2::Length(p2::Skew(v)), 5.0f);

    // Skew(v) is v rotated CCW, so its dot product with v is 0.
    EXPECT_FLOAT_EQ(p2::Dot(p2::Skew(v), v), 0.0f);
}

TEST(Vec2, LengthDistanceAndSquaredForms) {
    // (3, 4): 3^2 + 4^2 = 9 + 16 = 25, sqrt(25) = 5.
    EXPECT_FLOAT_EQ(p2::LengthSquared(Vec2{3.0f, 4.0f}), 25.0f);
    EXPECT_FLOAT_EQ(p2::Length(Vec2{3.0f, 4.0f}), 5.0f);

    // Distance((1, 2), (4, 6)): delta = (3, 4) -> 5.
    EXPECT_FLOAT_EQ(p2::DistanceSquared(Vec2{1.0f, 2.0f}, Vec2{4.0f, 6.0f}), 25.0f);
    EXPECT_FLOAT_EQ(p2::Distance(Vec2{1.0f, 2.0f}, Vec2{4.0f, 6.0f}), 5.0f);

    // Distance is symmetric and zero for identical points.
    EXPECT_FLOAT_EQ(p2::Distance(Vec2{1.0f, 2.0f}, Vec2{4.0f, 6.0f}),
                    p2::Distance(Vec2{4.0f, 6.0f}, Vec2{1.0f, 2.0f}));
    EXPECT_FLOAT_EQ(p2::Distance(Vec2{1.0f, 2.0f}, Vec2{1.0f, 2.0f}), 0.0f);

    // The zero vector has zero length.
    EXPECT_FLOAT_EQ(p2::Length(Vec2{0.0f, 0.0f}), 0.0f);
}

TEST(Vec2, NormalizedHandlesZeroAndTinyVectorsWithoutNaN) {
    // (3, 4) / 5 = (0.6, 0.8); |(0.6, 0.8)| = sqrt(0.36 + 0.64) = 1.
    const Vec2 unit = p2::Normalized(Vec2{3.0f, 4.0f});
    EXPECT_NEAR(unit.x, 0.6f, kTolerance);
    EXPECT_NEAR(unit.y, 0.8f, kTolerance);
    EXPECT_NEAR(p2::Length(unit), 1.0f, kTolerance);

    // The exact zero vector is a legitimate degenerate input (coincident
    // contact points produce it); it must return (0, 0), never NaN.
    const Vec2 zero = p2::Normalized(Vec2{0.0f, 0.0f});
    EXPECT_FLOAT_EQ(zero.x, 0.0f);
    EXPECT_FLOAT_EQ(zero.y, 0.0f);
    EXPECT_TRUE(p2::IsValid(zero));

    // 1e-20 has squared length 1e-40, which underflows to 0 in float. The
    // normalisation guard is based on lengthSquared < 1e-12, so even a
    // representable 1e-20 must fall back to (0, 0) instead of dividing by a
    // denormal/zero length.
    const Vec2 tiny = p2::Normalized(Vec2{1.0e-20f, 0.0f});
    EXPECT_FLOAT_EQ(tiny.x, 0.0f);
    EXPECT_FLOAT_EQ(tiny.y, 0.0f);
    EXPECT_TRUE(p2::IsValid(tiny));

    // A vector just below the guard threshold also falls back cleanly.
    const Vec2 belowGuard = p2::Normalized(Vec2{1.0e-7f, 0.0f});
    EXPECT_TRUE(p2::IsValid(belowGuard));

    // Direction is preserved for an already-normalised vector.
    const Vec2 axis = p2::Normalized(Vec2{0.0f, 2.0f});
    EXPECT_NEAR(axis.x, 0.0f, kTolerance);
    EXPECT_NEAR(axis.y, 1.0f, kTolerance);
}

TEST(Vec2, IsValidRejectsNaNAndInfinity) {
    EXPECT_TRUE((p2::IsValid(Vec2{0.0f, 0.0f})));
    EXPECT_TRUE((p2::IsValid(Vec2{-1.5f, 2.5f})));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FALSE((p2::IsValid(Vec2{nan, 0.0f})));
    EXPECT_FALSE((p2::IsValid(Vec2{0.0f, nan})));
    EXPECT_FALSE((p2::IsValid(Vec2{inf, 0.0f})));
    EXPECT_FALSE((p2::IsValid(Vec2{0.0f, -inf})));
}

TEST(Vec2, ComponentWiseMinMaxAbs) {
    const Vec2 a{3.0f, -2.0f};
    const Vec2 b{-1.0f, 5.0f};

    // Min picks the smaller component per axis: (min(3,-1), min(-2,5)) = (-1, -2)
    EXPECT_TRUE((p2::Min(a, b) == Vec2{-1.0f, -2.0f}));
    // Max picks the larger component per axis: (max(3,-1), max(-2,5)) = (3, 5)
    EXPECT_TRUE((p2::Max(a, b) == Vec2{3.0f, 5.0f}));

    // Min/Max are symmetric.
    EXPECT_TRUE(p2::Min(a, b) == p2::Min(b, a));
    EXPECT_TRUE(p2::Max(a, b) == p2::Max(b, a));

    // Abs flips negative components: Abs(3, -2) = (3, 2).
    EXPECT_TRUE((p2::Abs(a) == Vec2{3.0f, 2.0f}));
    EXPECT_TRUE((p2::Abs(Vec2{-1.0f, -2.0f}) == Vec2{1.0f, 2.0f}));
    // Abs is a no-op on non-negative input.
    EXPECT_TRUE((p2::Abs(Vec2{1.0f, 2.0f}) == Vec2{1.0f, 2.0f}));
}

TEST(Vec2, ClampAndLerp) {
    // Clamp((-5, 7), (0, 0), (4, 10)) = (0, 7)
    EXPECT_TRUE((p2::Clamp(Vec2{-5.0f, 7.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 10.0f}) == Vec2{0.0f, 7.0f}));
    // Values already in range pass through unchanged.
    EXPECT_TRUE((p2::Clamp(Vec2{2.0f, 3.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 10.0f}) == Vec2{2.0f, 3.0f}));
    // The bounds themselves are inclusive.
    EXPECT_TRUE((p2::Clamp(Vec2{4.0f, 10.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 10.0f}) == Vec2{4.0f, 10.0f}));

    // Lerp((0, 0), (10, 20), 0.25) = (0, 0) + 0.25 * (10, 20) = (2.5, 5).
    const Vec2 midpoint = p2::Lerp(Vec2{0.0f, 0.0f}, Vec2{10.0f, 20.0f}, 0.25f);
    EXPECT_NEAR(midpoint.x, 2.5f, kTolerance);
    EXPECT_NEAR(midpoint.y, 5.0f, kTolerance);

    // t = 0 and t = 1 are exact endpoints for these values.
    EXPECT_TRUE((p2::Lerp(Vec2{0.0f, 0.0f}, Vec2{10.0f, 20.0f}, 0.0f) == Vec2{0.0f, 0.0f}));
    EXPECT_TRUE((p2::Lerp(Vec2{0.0f, 0.0f}, Vec2{10.0f, 20.0f}, 1.0f) == Vec2{10.0f, 20.0f}));
}

#ifdef PHYSICS2D_USE_EIGEN
TEST(Vec2, EigenCrossValidation) {
    // Independent implementation check on ~100 random values. Inputs are drawn
    // from [-1, 1], so |dot| and |cross| are <= 2: the float error of two
    // multiply-adds is about 2 * 2^-24 * 2 ~ 2.4e-7, comfortably inside the
    // 1e-6 tolerance. Eigen has no 2D cross product, so the cross check compares
    // the component formula Eigen's Vector2f stores.
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    for (int i = 0; i < 100; ++i) {
        const Vec2 a{dist(rng), dist(rng)};
        const Vec2 b{dist(rng), dist(rng)};

        const Eigen::Vector2f ea(a.x, a.y);
        const Eigen::Vector2f eb(b.x, b.y);

        EXPECT_NEAR(p2::Dot(a, b), ea.dot(eb), kTolerance);
        EXPECT_NEAR(p2::Cross(a, b), ea.x() * eb.y() - ea.y() * eb.x(), kTolerance);

        // The random range makes a zero-length vector a measure-zero event; the
        // fixed seed guarantees this test is reproducible when it passes.
        const Vec2 p2Unit = p2::Normalized(a);
        const Eigen::Vector2f eigenUnit = ea.normalized();
        EXPECT_NEAR(p2Unit.x, eigenUnit.x(), kTolerance);
        EXPECT_NEAR(p2Unit.y, eigenUnit.y(), kTolerance);
    }
}
#endif // PHYSICS2D_USE_EIGEN
