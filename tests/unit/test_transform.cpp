// Unit tests for p2::Transform (Week 1 D4, design document P03).
//
// A Transform is a rigid motion: rotate by q, then translate by p. The tests
// pin down the exact convention (world = q*local + p) with hand-computed point
// mappings, then check the algebraic properties (inverse, composition order,
// associativity) on both fixed and random inputs.

#include <gtest/gtest.h>

#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Transform.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace {

using p2::Rot;
using p2::Transform;
using p2::Vec2;

constexpr float kTolerance = 1.0e-6f;

std::mt19937 MakeRng() {
    return std::mt19937(0x7A11u);
}

float ScaledTolerance(Vec2 v) {
    // Rounding error of a rigid transform grows with the magnitude of the point
    // (cancellation when subtracting the translation, then rotation); one ulp of
    // a coordinate of size M is M * 1.2e-7, so 1e-5 * max(1, |v|) covers a few
    // tens of ulps with margin.
    return 1.0e-5f * std::max(1.0f, std::max(std::fabs(v.x), std::fabs(v.y)));
}

} // namespace

TEST(Transform, DefaultIsIdentity) {
    const Transform identity;
    EXPECT_TRUE((identity.p == Vec2{0.0f, 0.0f}));
    EXPECT_FLOAT_EQ(identity.q.c, 1.0f);
    EXPECT_FLOAT_EQ(identity.q.s, 0.0f);

    // Both directions leave points untouched.
    EXPECT_TRUE((p2::Mul(identity, Vec2{2.5f, -1.5f}) == Vec2{2.5f, -1.5f}));
    EXPECT_TRUE((p2::MulT(identity, Vec2{2.5f, -1.5f}) == Vec2{2.5f, -1.5f}));
}

TEST(Transform, MulTransformsPointsIntoWorldSpace) {
    // xf: rotate 90 degrees CCW, then translate by (3, 4).
    const Transform xf{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};

    // local (1, 0) -> rotate -> (0, 1) -> translate -> (3, 5).
    const Vec2 worldX = p2::Mul(xf, Vec2{1.0f, 0.0f});
    EXPECT_NEAR(worldX.x, 3.0f, kTolerance);
    EXPECT_NEAR(worldX.y, 5.0f, kTolerance);

    // local (0, 2) -> rotate -> (-2, 0) -> translate -> (1, 4).
    const Vec2 worldY = p2::Mul(xf, Vec2{0.0f, 2.0f});
    EXPECT_NEAR(worldY.x, 1.0f, kTolerance);
    EXPECT_NEAR(worldY.y, 4.0f, kTolerance);

    // The translation itself is the image of the local origin: Mul(xf, 0) == p.
    EXPECT_TRUE((p2::Mul(xf, Vec2{0.0f, 0.0f}) == Vec2{3.0f, 4.0f}));
}

TEST(Transform, MulTIsTheInverse) {
    // Known case: world (3, 5) relative to xf{p=(3,4), q=90 deg}
    //   -> subtract p: (0, 1)
    //   -> inverse-rotate: Rot^T (0, 1) = [[0, 1], [-1, 0]] * (0, 1) = (1, 0).
    const Transform xf{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const Vec2 local = p2::MulT(xf, Vec2{3.0f, 5.0f});
    EXPECT_NEAR(local.x, 1.0f, kTolerance);
    EXPECT_NEAR(local.y, 0.0f, kTolerance);

    // Random round trip on 1000 rigid transforms: MulT(xf, Mul(xf, v)) == v.
    // Points are in [-1, 1] while translations are up to 10, so the absolute
    // error is dominated by the translation subtraction; ScaledTolerance keeps
    // the bound proportional to the magnitude involved.
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> translationDist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> angleDist(-p2::kPi, p2::kPi);
    std::uniform_real_distribution<float> pointDist(-1.0f, 1.0f);

    for (int i = 0; i < 1000; ++i) {
        const Transform randomXf{Vec2{translationDist(rng), translationDist(rng)}, Rot(angleDist(rng))};
        const Vec2 v{pointDist(rng), pointDist(rng)};
        const Vec2 roundTrip = p2::MulT(randomXf, p2::Mul(randomXf, v));
        const float tolerance = ScaledTolerance(v);
        EXPECT_NEAR(roundTrip.x, v.x, tolerance);
        EXPECT_NEAR(roundTrip.y, v.y, tolerance);
    }
}

TEST(Transform, CompositionAppliesRightOperandFirst) {
    // a: translate (1, 0), no rotation. b: rotate 90 degrees CCW, translate (0, 2).
    const Transform a{Vec2{1.0f, 0.0f}, Rot()};
    const Transform b{Vec2{0.0f, 2.0f}, Rot(p2::kHalfPi)};

    // Mul(a, b) applies b first, then a:
    //   p = a.q * b.p + a.p = (0, 2) + (1, 0) = (1, 2)
    //   q = a.q * b.q = 90 degrees.
    const Transform ab = p2::Mul(a, b);
    EXPECT_NEAR(ab.p.x, 1.0f, kTolerance);
    EXPECT_NEAR(ab.p.y, 2.0f, kTolerance);
    EXPECT_NEAR(ab.q.c, 0.0f, kTolerance);
    EXPECT_NEAR(ab.q.s, 1.0f, kTolerance);

    // Point check: local (1, 0) -> b: rotate -> (0, 1) -> + (0, 2) = (0, 3)
    //             -> a: + (1, 0) = (1, 3).
    const Vec2 world = p2::Mul(ab, Vec2{1.0f, 0.0f});
    EXPECT_NEAR(world.x, 1.0f, kTolerance);
    EXPECT_NEAR(world.y, 3.0f, kTolerance);

    // The composed transform agrees with applying the two transforms in turn.
    const Vec2 sequential = p2::Mul(a, p2::Mul(b, Vec2{1.0f, 0.0f}));
    EXPECT_NEAR(sequential.x, world.x, kTolerance);
    EXPECT_NEAR(sequential.y, world.y, kTolerance);
}

TEST(Transform, CompositionWithRotationOfTranslation) {
    // a rotates 90 degrees CCW around the origin and translates by (1, 2).
    // b is a pure translation by (3, 0).
    const Transform a{Vec2{1.0f, 2.0f}, Rot(p2::kHalfPi)};
    const Transform b{Vec2{3.0f, 0.0f}, Rot()};

    // p = a.q * b.p + a.p = Rot90(3, 0) + (1, 2) = (0, 3) + (1, 2) = (1, 5).
    const Transform ab = p2::Mul(a, b);
    EXPECT_NEAR(ab.p.x, 1.0f, kTolerance);
    EXPECT_NEAR(ab.p.y, 5.0f, kTolerance);
}

TEST(Transform, CompositionIsAssociativeAndRandomlyConsistent) {
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> translationDist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> angleDist(-p2::kPi, p2::kPi);
    std::uniform_real_distribution<float> pointDist(-5.0f, 5.0f);

    for (int i = 0; i < 100; ++i) {
        const Transform a{Vec2{translationDist(rng), translationDist(rng)}, Rot(angleDist(rng))};
        const Transform b{Vec2{translationDist(rng), translationDist(rng)}, Rot(angleDist(rng))};
        const Vec2 v{pointDist(rng), pointDist(rng)};

        const Vec2 stepwise = p2::Mul(a, p2::Mul(b, v));
        const Vec2 composed = p2::Mul(p2::Mul(a, b), v);
        const float tolerance = ScaledTolerance(v);
        EXPECT_NEAR(stepwise.x, composed.x, tolerance);
        EXPECT_NEAR(stepwise.y, composed.y, tolerance);
    }
}

TEST(Transform, MulTOfTransformsIsRelativeTransform) {
    // MulT(a, b) computes a^-1 * b. Two identities must hold:
    //   Mul(a, MulT(a, b)) == b   and   MulT(a, Mul(a, b)) == b.
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> translationDist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> angleDist(-p2::kPi, p2::kPi);

    for (int i = 0; i < 100; ++i) {
        const Transform a{Vec2{translationDist(rng), translationDist(rng)}, Rot(angleDist(rng))};
        const Transform b{Vec2{translationDist(rng), translationDist(rng)}, Rot(angleDist(rng))};

        const Transform first = p2::Mul(a, p2::MulT(a, b));
        EXPECT_NEAR(first.p.x, b.p.x, 1.0e-4f);
        EXPECT_NEAR(first.p.y, b.p.y, 1.0e-4f);
        EXPECT_NEAR(first.q.c, b.q.c, kTolerance);
        EXPECT_NEAR(first.q.s, b.q.s, kTolerance);

        const Transform second = p2::MulT(a, p2::Mul(a, b));
        EXPECT_NEAR(second.p.x, b.p.x, 1.0e-4f);
        EXPECT_NEAR(second.p.y, b.p.y, 1.0e-4f);
        EXPECT_NEAR(second.q.c, b.q.c, kTolerance);
        EXPECT_NEAR(second.q.s, b.q.s, kTolerance);
    }
}

TEST(Transform, IdentityCompositionAndInverse) {
    const Transform xf{Vec2{2.0f, -3.0f}, Rot(0.75f)};
    const Transform identity;

    // Identity is neutral on both sides.
    EXPECT_TRUE(p2::Mul(xf, identity).p == xf.p);
    EXPECT_TRUE(p2::Mul(identity, xf).p == xf.p);
    EXPECT_NEAR(p2::Mul(xf, identity).q.c, xf.q.c, kTolerance);
    EXPECT_NEAR(p2::Mul(identity, xf).q.c, xf.q.c, kTolerance);

    // xf^-1 == MulT(xf, identity); composing it with xf gives the identity
    // transform up to rounding.
    const Transform inverse = p2::MulT(xf, identity);
    const Transform roundTrip = p2::Mul(xf, inverse);
    EXPECT_NEAR(roundTrip.p.x, 0.0f, kTolerance);
    EXPECT_NEAR(roundTrip.p.y, 0.0f, kTolerance);
    EXPECT_NEAR(roundTrip.q.c, 1.0f, kTolerance);
    EXPECT_NEAR(roundTrip.q.s, 0.0f, kTolerance);
}
