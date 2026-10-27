// Unit tests for p2::Mat2 (Week 1 D3, design document P03).
//
// Mat2 is column-major: `cx` is the image of the x basis vector and `cy` the
// image of the y basis vector, so Mat2 * Vec2 = cx*v.x + cy*v.y. Every expected
// matrix below is written first as a row-major 2x2 matrix, transposed into
// columns, and the arithmetic is shown in the comment.

#include <gtest/gtest.h>

#include "physics2d/Math/Mat2.h"

#include <cmath>
#include <random>

#ifdef PHYSICS2D_USE_EIGEN
// Eigen is only used to cross-check results; keep its warning noise out of the
// project's own /W4-clean build.
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#include <Eigen/Dense>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#endif

namespace {

using p2::Mat2;
using p2::Vec2;

constexpr float kTolerance = 1.0e-6f;

std::mt19937 MakeRng() {
    return std::mt19937(0x5EEDu);
}

bool NearlyEqual(const Mat2& a, const Mat2& b, float tolerance) {
    return std::abs(a.cx.x - b.cx.x) <= tolerance && std::abs(a.cx.y - b.cx.y) <= tolerance &&
           std::abs(a.cy.x - b.cy.x) <= tolerance && std::abs(a.cy.y - b.cy.y) <= tolerance;
}

} // namespace

TEST(Mat2, DefaultConstructionIsIdentity) {
    const Mat2 identity;
    // Identity columns: (1, 0) and (0, 1).
    EXPECT_FLOAT_EQ(identity.cx.x, 1.0f);
    EXPECT_FLOAT_EQ(identity.cx.y, 0.0f);
    EXPECT_FLOAT_EQ(identity.cy.x, 0.0f);
    EXPECT_FLOAT_EQ(identity.cy.y, 1.0f);
    EXPECT_TRUE(NearlyEqual(identity, Mat2::Identity(), 0.0f));
}

TEST(Mat2, MulWithVectorUsesColumns) {
    // Columns cx = (2, 3), cy = (4, 5) represent the row-major matrix
    // [[2, 4], [3, 5]].
    const Mat2 m{Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}};

    // Applying to the basis vectors must return the columns.
    EXPECT_TRUE((p2::Mul(m, Vec2{1.0f, 0.0f}) == Vec2{2.0f, 3.0f}));
    EXPECT_TRUE((p2::Mul(m, Vec2{0.0f, 1.0f}) == Vec2{4.0f, 5.0f}));

    // Mul(m, (2, 3)) = 2*(2, 3) + 3*(4, 5) = (4 + 12, 6 + 15) = (16, 21).
    // Row-major check: [[2, 4], [3, 5]] * (2, 3) = (2*2 + 4*3, 3*2 + 5*3) = (16, 21).
    EXPECT_TRUE((p2::Mul(m, Vec2{2.0f, 3.0f}) == Vec2{16.0f, 21.0f}));

    // Identity leaves vectors untouched.
    EXPECT_TRUE((p2::Mul(Mat2::Identity(), Vec2{-3.0f, 7.0f}) == Vec2{-3.0f, 7.0f}));
}

TEST(Mat2, MulComposesRightToLeft) {
    // A = [[1, 3], [2, 4]] -> columns cx = (1, 2), cy = (3, 4)
    // B = [[5, 7], [6, 8]] -> columns cx = (5, 6), cy = (7, 8)
    const Mat2 a{Vec2{1.0f, 2.0f}, Vec2{3.0f, 4.0f}};
    const Mat2 b{Vec2{5.0f, 6.0f}, Vec2{7.0f, 8.0f}};

    // A*B row-major: [[1*5 + 3*6, 1*7 + 3*8], [2*5 + 4*6, 2*7 + 4*8]]
    //                   = [[5 + 18, 7 + 24], [10 + 24, 14 + 32]]
    //                   = [[23, 31], [34, 46]]
    // Column-major columns: (23, 34) and (31, 46).
    const Mat2 product = p2::Mul(a, b);
    EXPECT_TRUE((product.cx == Vec2{23.0f, 34.0f}));
    EXPECT_TRUE((product.cy == Vec2{31.0f, 46.0f}));

    // Composition order: Mul(a, b) * v == a * (b * v) for (1, 1):
    //   b*(1,1) = (5+7, 6+8) = (12, 14); a*(12,14) = (12+42, 24+56) = (54, 80).
    const Vec2 v{1.0f, 1.0f};
    EXPECT_TRUE(p2::Mul(product, v) == p2::Mul(a, p2::Mul(b, v)));
    EXPECT_TRUE((p2::Mul(product, v) == Vec2{54.0f, 80.0f}));
}

TEST(Mat2, Transpose) {
    // M = [[2, 4], [3, 5]] (columns (2, 3), (4, 5)); M^T = [[2, 3], [4, 5]],
    // i.e. columns (2, 4) and (3, 5).
    const Mat2 m{Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}};
    const Mat2 transposed = p2::Transpose(m);
    EXPECT_TRUE((transposed.cx == Vec2{2.0f, 4.0f}));
    EXPECT_TRUE((transposed.cy == Vec2{3.0f, 5.0f}));

    // Transposition is an involution and keeps the determinant.
    EXPECT_TRUE(NearlyEqual(p2::Transpose(transposed), m, 0.0f));
    EXPECT_FLOAT_EQ(p2::Determinant(transposed), p2::Determinant(m));
}

TEST(Mat2, Determinant) {
    // M = [[2, 4], [3, 5]]: det = 2*5 - 4*3 = 10 - 12 = -2.
    EXPECT_FLOAT_EQ(p2::Determinant(Mat2{Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}}), -2.0f);

    // Identity determinant is 1.
    EXPECT_FLOAT_EQ(p2::Determinant(Mat2::Identity()), 1.0f);

    // det(A*B) = det(A) * det(B): det(A) = -2, det(B) = 5*8 - 7*6 = 40 - 42 = -2,
    // so det(A*B) must be 4.
    const Mat2 a{Vec2{1.0f, 2.0f}, Vec2{3.0f, 4.0f}};
    const Mat2 b{Vec2{5.0f, 6.0f}, Vec2{7.0f, 8.0f}};
    EXPECT_FLOAT_EQ(p2::Determinant(p2::Mul(a, b)), p2::Determinant(a) * p2::Determinant(b));

    // A matrix with duplicate columns is singular: det = 0.
    EXPECT_FLOAT_EQ(p2::Determinant(Mat2{Vec2{2.0f, 3.0f}, Vec2{4.0f, 6.0f}}), 0.0f);
}

TEST(Mat2, InverseOfKnownMatrix) {
    // M = [[2, 4], [3, 5]], det = -2.
    // Closed form: M^-1 = (1/det) [[5, -4], [-3, 2]] = [[-2.5, 2], [1.5, -1]].
    // Column-major: cx = (-2.5, 1.5), cy = (2, -1).
    const Mat2 m{Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}};
    Mat2 inverse;
    ASSERT_TRUE(p2::Inverse(m, inverse));
    EXPECT_NEAR(inverse.cx.x, -2.5f, kTolerance);
    EXPECT_NEAR(inverse.cx.y, 1.5f, kTolerance);
    EXPECT_NEAR(inverse.cy.x, 2.0f, kTolerance);
    EXPECT_NEAR(inverse.cy.y, -1.0f, kTolerance);

    // A * A^-1 == I. The 1e-6 tolerance covers the accumulation of rounding in
    // the eight products; the entries involved are O(10).
    EXPECT_TRUE(NearlyEqual(p2::Mul(m, inverse), Mat2::Identity(), 1.0e-5f));
    EXPECT_TRUE(NearlyEqual(p2::Mul(inverse, m), Mat2::Identity(), 1.0e-5f));
}

TEST(Mat2, InverseOfRotationIsTranspose) {
    // A rotation matrix is orthonormal, so R^-1 == R^T (det = c^2 + s^2 = 1).
    // R = [[0.6, -0.8], [0.8, 0.6]] (a 3-4-5 rotation): columns (0.6, 0.8), (-0.8, 0.6).
    const Mat2 rotation{Vec2{0.6f, 0.8f}, Vec2{-0.8f, 0.6f}};
    Mat2 inverse;
    ASSERT_TRUE(p2::Inverse(rotation, inverse));
    EXPECT_TRUE(NearlyEqual(inverse, p2::Transpose(rotation), kTolerance));

    // GetInverse (the assert-in-debug convenience form) must agree with Inverse.
    EXPECT_TRUE(NearlyEqual(p2::GetInverse(rotation), inverse, 0.0f));
}

TEST(Mat2, InverseRejectsSingularMatrixAndLeavesOutputUntouched) {
    // Columns (1, 2) and (2, 4) are parallel: det = 1*4 - 2*2 = 0.
    const Mat2 singular{Vec2{1.0f, 2.0f}, Vec2{2.0f, 4.0f}};
    Mat2 out{Vec2{7.0f, 8.0f}, Vec2{9.0f, 10.0f}};
    const Mat2 sentinel = out;

    EXPECT_FALSE(p2::Inverse(singular, out));
    // Contract: the output buffer is not written on failure.
    EXPECT_TRUE(out.cx == sentinel.cx);
    EXPECT_TRUE(out.cy == sentinel.cy);

    // The all-zero matrix is the worst singular case and must not divide by zero.
    Mat2 zeroOut{Vec2{7.0f, 8.0f}, Vec2{9.0f, 10.0f}};
    EXPECT_FALSE((p2::Inverse(Mat2{Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}}, zeroOut)));
    EXPECT_TRUE((zeroOut.cx == Vec2{7.0f, 8.0f}));

    // A matrix with two identical columns is singular too (det exactly 0).
    EXPECT_FALSE((p2::Inverse(Mat2{Vec2{2.0f, 3.0f}, Vec2{2.0f, 3.0f}}, out)));
}

TEST(Mat2, InverseAcceptsSmallButWellConditionedMatrix) {
    // Uniformly scaled identity: M = 1e-4 * I has det = 1e-8. The singularity
    // test is scale-relative (|det| > kEpsilon * maxComponent^2 = 1e-6 * 1e-8),
    // so this perfectly invertible matrix must be accepted; an absolute 1e-6
    // threshold would wrongly reject it.
    const Mat2 scaled{Vec2{1.0e-4f, 0.0f}, Vec2{0.0f, 1.0e-4f}};
    Mat2 inverse;
    ASSERT_TRUE(p2::Inverse(scaled, inverse));
    // (1e-4 * I)^-1 = 1e4 * I.
    EXPECT_NEAR(inverse.cx.x, 1.0e4f, 1.0e-2f);
    EXPECT_NEAR(inverse.cy.y, 1.0e4f, 1.0e-2f);
    EXPECT_NEAR(inverse.cx.y, 0.0f, kTolerance);
    EXPECT_NEAR(inverse.cy.x, 0.0f, kTolerance);
}

TEST(Mat2, RandomInvertibleMatricesRoundTripToIdentity) {
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> dist(-2.0f, 2.0f);

    int tested = 0;
    // Bounded loop: rejection sampling never runs long in practice, but a hard
    // attempt cap keeps a broken generator from hanging CI.
    for (int attempt = 0; attempt < 10000 && tested < 100; ++attempt) {
        const Mat2 m{Vec2{dist(rng), dist(rng)}, Vec2{dist(rng), dist(rng)}};
        // Skip nearly singular draws: their inverse entries blow up and would
        // make the absolute tolerance meaningless. 0.1 keeps |det| large enough
        // for the inverse to be well conditioned in float.
        if (std::abs(p2::Determinant(m)) < 0.1f) {
            continue;
        }
        Mat2 inverse;
        ASSERT_TRUE(p2::Inverse(m, inverse));
        // Entries are O(2), inverse entries O(20); the round-trip error is a few
        // ulps of the intermediate magnitudes, hence the 1e-4 absolute bound.
        EXPECT_TRUE(NearlyEqual(p2::Mul(m, inverse), Mat2::Identity(), 1.0e-4f));
        ++tested;
    }
}

#ifdef PHYSICS2D_USE_EIGEN
TEST(Mat2, EigenCrossValidation) {
    // Independent implementation check. Matrices are drawn from [-1, 1] with
    // |det| >= 0.25, so inverse entries are bounded by ~4 and 1e-6 absolute
    // tolerance is a relative bound of ~2.5e-7, just above one float ulp.
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    const Vec2 v{0.3f, -0.7f};
    const Eigen::Vector2f ev(v.x, v.y);

    int tested = 0;
    // Bounded rejection sampling, same rationale as the random inverse test.
    for (int attempt = 0; attempt < 10000 && tested < 100; ++attempt) {
        const Mat2 m{Vec2{dist(rng), dist(rng)}, Vec2{dist(rng), dist(rng)}};
        if (std::abs(p2::Determinant(m)) < 0.25f) {
            continue;
        }

        // Eigen::Matrix2f is filled in row-major comma-initializer order:
        // [[cx.x, cy.x], [cx.y, cy.y]].
        Eigen::Matrix2f em;
        em << m.cx.x, m.cy.x, m.cx.y, m.cy.y;

        EXPECT_NEAR(p2::Determinant(m), em.determinant(), kTolerance);

        const Vec2 mv = p2::Mul(m, v);
        const Eigen::Vector2f emv = em * ev;
        EXPECT_NEAR(mv.x, emv.x(), kTolerance);
        EXPECT_NEAR(mv.y, emv.y(), kTolerance);

        Mat2 inverse;
        ASSERT_TRUE(p2::Inverse(m, inverse));
        const Eigen::Matrix2f eigenInverse = em.inverse();
        EXPECT_NEAR(inverse.cx.x, eigenInverse(0, 0), kTolerance);
        EXPECT_NEAR(inverse.cx.y, eigenInverse(1, 0), kTolerance);
        EXPECT_NEAR(inverse.cy.x, eigenInverse(0, 1), kTolerance);
        EXPECT_NEAR(inverse.cy.y, eigenInverse(1, 1), kTolerance);
        ++tested;
    }
}
#endif // PHYSICS2D_USE_EIGEN
