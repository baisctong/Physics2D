// Unit tests for p2::Rot (Week 1 D3, design document P03).
//
// Rot stores (cos, sin) instead of an angle. The tests therefore check both
// faces of the contract: the trig conversions at the user-facing boundary
// (Rot(angle), GetAngle) and the algebraic properties of composition (round
// trips, angle addition, long-run unit-length stability).

#include <gtest/gtest.h>

#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"

#include <cmath>
#include <initializer_list>
#include <random>

namespace {

using p2::Rot;
using p2::Vec2;

constexpr float kTolerance = 1.0e-6f;

std::mt19937 MakeRng() {
    return std::mt19937(0xA11CEu);
}

} // namespace

TEST(Rot, ConstructionFromAngle) {
    // sin(0) = 0 and cos(0) = 1 are exact in IEEE-754, so no tolerance is needed.
    const Rot identity(0.0f);
    EXPECT_FLOAT_EQ(identity.c, 1.0f);
    EXPECT_FLOAT_EQ(identity.s, 0.0f);

    // 90 degrees: cos(pi/2) = 0, sin(pi/2) = 1. kHalfPi is a float approximation
    // of pi/2, so both values are within ~1e-7 of the analytic result; 1e-6
    // leaves headroom for the libm rounding.
    const Rot quarterTurn(p2::kHalfPi);
    EXPECT_NEAR(quarterTurn.c, 0.0f, kTolerance);
    EXPECT_NEAR(quarterTurn.s, 1.0f, kTolerance);

    // 180 degrees: (c, s) = (-1, 0).
    const Rot halfTurn(p2::kPi);
    EXPECT_NEAR(halfTurn.c, -1.0f, kTolerance);
    EXPECT_NEAR(halfTurn.s, 0.0f, kTolerance);

    // The default constructor is the identity rotation.
    const Rot defaultRot;
    EXPECT_FLOAT_EQ(defaultRot.c, 1.0f);
    EXPECT_FLOAT_EQ(defaultRot.s, 0.0f);
}

TEST(Rot, SetMatchesConstructorAndSetIdentity) {
    const float angle = 0.7f;
    const Rot byConstructor(angle);
    Rot bySetter;
    bySetter.Set(angle);

    // Same trig inputs, so the two paths must be bit-identical.
    EXPECT_FLOAT_EQ(bySetter.c, byConstructor.c);
    EXPECT_FLOAT_EQ(bySetter.s, byConstructor.s);

    // SetIdentity restores (1, 0).
    bySetter.SetIdentity();
    EXPECT_FLOAT_EQ(bySetter.c, 1.0f);
    EXPECT_FLOAT_EQ(bySetter.s, 0.0f);
}

TEST(Rot, GetAngleRoundTripsAndWraps) {
    // atan2(sin a, cos a) == a for a in (-pi, pi]. 1e-6 is about 100x the
    // rounding error of the sin/cos/atan2 round trip in float.
    for (const float angle : {0.0f, 0.25f, -0.25f, 1.0f, -1.0f, 2.5f, -2.5f, 3.0f, -3.0f}) {
        const Rot q(angle);
        EXPECT_NEAR(q.GetAngle(), angle, kTolerance);
    }

    // Angles outside the principal range come back wrapped:
    // 4 rad = 4 - 2*pi = -2.2831853 rad.
    const Rot wrapped(4.0f);
    EXPECT_NEAR(wrapped.GetAngle(), 4.0f - p2::kTwoPi, 1.0e-5f);

    // The identity angle is exactly zero (atan2(0, 1) == 0).
    EXPECT_FLOAT_EQ(Rot().GetAngle(), 0.0f);

    // The result always lies in (-pi, pi].
    for (const float angle : {-10.0f, -4.0f, 4.0f, 10.0f}) {
        const float normalized = Rot(angle).GetAngle();
        EXPECT_GT(normalized, -p2::kPi - kTolerance);
        EXPECT_LE(normalized, p2::kPi + kTolerance);
    }
}

TEST(Rot, GetAxisXAndY) {
    // Identity rotation: the axes are exactly the world basis vectors.
    const Rot identity;
    EXPECT_TRUE((identity.GetAxisX() == Vec2{1.0f, 0.0f}));
    EXPECT_TRUE((identity.GetAxisY() == Vec2{0.0f, 1.0f}));

    // 90 degrees CCW: x axis -> +y, y axis -> -x.
    const Rot quarterTurn(p2::kHalfPi);
    EXPECT_NEAR(quarterTurn.GetAxisX().x, 0.0f, kTolerance);
    EXPECT_NEAR(quarterTurn.GetAxisX().y, 1.0f, kTolerance);
    EXPECT_NEAR(quarterTurn.GetAxisY().x, -1.0f, kTolerance);
    EXPECT_NEAR(quarterTurn.GetAxisY().y, 0.0f, kTolerance);

    // For any rotation the axes are orthonormal and right-handed:
    // axisX . axisY = 0, axisX x axisY = c^2 + s^2 = 1.
    const Rot arbitrary(1.234f);
    EXPECT_NEAR(p2::Dot(arbitrary.GetAxisX(), arbitrary.GetAxisY()), 0.0f, kTolerance);
    EXPECT_NEAR(p2::Cross(arbitrary.GetAxisX(), arbitrary.GetAxisY()), 1.0f, kTolerance);
    EXPECT_NEAR(p2::Length(arbitrary.GetAxisX()), 1.0f, kTolerance);
    EXPECT_NEAR(p2::Length(arbitrary.GetAxisY()), 1.0f, kTolerance);
}

TEST(Rot, RotateVectorByKnownAngles) {
    // 90 degrees CCW maps (1, 0) -> (0, 1) and (0, 1) -> (-1, 0).
    const Rot quarterTurn(p2::kHalfPi);
    const Vec2 rotatedX = p2::Mul(quarterTurn, Vec2{1.0f, 0.0f});
    EXPECT_NEAR(rotatedX.x, 0.0f, kTolerance);
    EXPECT_NEAR(rotatedX.y, 1.0f, kTolerance);
    const Vec2 rotatedY = p2::Mul(quarterTurn, Vec2{0.0f, 1.0f});
    EXPECT_NEAR(rotatedY.x, -1.0f, kTolerance);
    EXPECT_NEAR(rotatedY.y, 0.0f, kTolerance);

    // 45 degrees CCW maps (1, 0) -> (sqrt(1/2), sqrt(1/2)) ~ (0.7071068, 0.7071068).
    const Rot eighthTurn(p2::kPi / 4.0f);
    const Vec2 diagonal = p2::Mul(eighthTurn, Vec2{1.0f, 0.0f});
    EXPECT_NEAR(diagonal.x, std::sqrt(0.5f), kTolerance);
    EXPECT_NEAR(diagonal.y, std::sqrt(0.5f), kTolerance);

    // MulT is the inverse rotation: backwards (1, 0) is (0, -1).
    const Vec2 backwards = p2::MulT(quarterTurn, Vec2{1.0f, 0.0f});
    EXPECT_NEAR(backwards.x, 0.0f, kTolerance);
    EXPECT_NEAR(backwards.y, -1.0f, kTolerance);

    // Rotation is an isometry: |Mul(q, v)| == |v|. |(3, 4)| = 5.
    EXPECT_NEAR(p2::Length(p2::Mul(quarterTurn, Vec2{3.0f, 4.0f})), 5.0f, kTolerance);
}

TEST(Rot, MulTInvertsMulForRandomInput) {
    std::mt19937 rng = MakeRng();
    std::uniform_real_distribution<float> angleDist(-p2::kPi, p2::kPi);
    std::uniform_real_distribution<float> valueDist(-1.0f, 1.0f);

    // 1000 groups is the D3 acceptance criterion. Inputs are O(1), so each
    // multiply costs at most ~1 ulp (1.2e-7); 1e-6 leaves ~8x headroom and is
    // the bound required by the design document.
    for (int i = 0; i < 1000; ++i) {
        const Rot q(angleDist(rng));
        const Vec2 v{valueDist(rng), valueDist(rng)};
        const Vec2 roundTrip = p2::MulT(q, p2::Mul(q, v));
        EXPECT_NEAR(roundTrip.x, v.x, 1.0e-6f);
        EXPECT_NEAR(roundTrip.y, v.y, 1.0e-6f);
    }
}

TEST(Rot, CompositionMatchesAngleAddition) {
    // cos(a+b) = cos a cos b - sin a sin b and sin(a+b) = sin a cos b + cos a sin b,
    // so Mul(0.3, 0.4) must equal Rot(0.7).
    const Rot a(0.3f);
    const Rot b(0.4f);
    const Rot combined = p2::Mul(a, b);
    EXPECT_NEAR(combined.c, std::cos(0.7f), kTolerance);
    EXPECT_NEAR(combined.s, std::sin(0.7f), kTolerance);
    EXPECT_NEAR(combined.GetAngle(), 0.7f, 1.0e-5f);

    // Composition order: applying b first then a equals applying the composite.
    const Vec2 v{1.5f, -0.5f};
    const Vec2 applyTwo = p2::Mul(a, p2::Mul(b, v));
    const Vec2 applyComposite = p2::Mul(combined, v);
    EXPECT_NEAR(applyTwo.x, applyComposite.x, kTolerance);
    EXPECT_NEAR(applyTwo.y, applyComposite.y, kTolerance);

    // Association of three rotations: (a*b)*c == a*(b*c).
    const Rot c(1.1f);
    const Rot left = p2::Mul(p2::Mul(a, b), c);
    const Rot right = p2::Mul(a, p2::Mul(b, c));
    EXPECT_NEAR(left.c, right.c, kTolerance);
    EXPECT_NEAR(left.s, right.s, kTolerance);
}

TEST(Rot, MulTOfRotationsIsRelativeRotation) {
    // MulT(a, b) computes a^-1 * b, so composing a afterwards recovers b:
    // a * (a^-1 * b) == b.
    const Rot a(0.8f);
    const Rot b(-1.3f);
    const Rot relative = p2::MulT(a, b);
    const Rot restored = p2::Mul(a, relative);
    EXPECT_NEAR(restored.c, b.c, kTolerance);
    EXPECT_NEAR(restored.s, b.s, kTolerance);

    // A rotation relative to itself is the identity: MulT(a, a) == (1, 0).
    const Rot selfRelative = p2::MulT(a, a);
    EXPECT_NEAR(selfRelative.c, 1.0f, kTolerance);
    EXPECT_NEAR(selfRelative.s, 0.0f, kTolerance);

    // MulT(a, b) also equals Mul(MulT(a, Identity), b); checked via the vector
    // form: MulT(q, v) == Mul(q^T, v).
    const Vec2 v{2.0f, -1.0f};
    const Vec2 direct = p2::MulT(a, v);
    const Vec2 viaTranspose = p2::Mul(Rot{a.c, -a.s}, v);
    EXPECT_NEAR(direct.x, viaTranspose.x, kTolerance);
    EXPECT_NEAR(direct.y, viaTranspose.y, kTolerance);
}

TEST(Rot, NormalizeRestoresUnitLength) {
    // A deliberately denormalised pair: (3, 4) has length 5; Normalize must
    // produce (0.6, 0.8), the same as Rot's own (c, s) invariant.
    Rot stretched{3.0f, 4.0f};
    stretched.Normalize();
    EXPECT_NEAR(stretched.c, 0.6f, kTolerance);
    EXPECT_NEAR(stretched.s, 0.8f, kTolerance);

    // Already-unit rotations pass through unchanged (within 1 ulp).
    Rot unit(0.7f);
    const float originalC = unit.c;
    const float originalS = unit.s;
    unit.Normalize();
    EXPECT_NEAR(unit.c, originalC, kTolerance);
    EXPECT_NEAR(unit.s, originalS, kTolerance);

    // The guard turns a corrupt pair into the identity rather than NaN.
    Rot corrupt{0.0f, 0.0f};
    corrupt.Normalize();
    EXPECT_FLOAT_EQ(corrupt.c, 1.0f);
    EXPECT_FLOAT_EQ(corrupt.s, 0.0f);
}

TEST(Rot, OneMillionCompositionsStayUnitLength) {
    // Section 6.1: the (c, s) representation stays stable because each
    // composition is followed by a renormalisation. 1e6 multiplies by a
    // 0.01 rad step would otherwise let the magnitude drift; the D3 acceptance
    // criterion is |c^2 + s^2 - 1| < 1e-4 after that chain.
    Rot q; // identity
    const Rot step(0.01f);
    constexpr int kIterations = 1000000;
    for (int i = 0; i < kIterations; ++i) {
        q = p2::Mul(q, step);
        q.Normalize();
    }

    const float lengthSquared = q.c * q.c + q.s * q.s;
    EXPECT_NEAR(lengthSquared, 1.0f, 1.0e-4f);

    // The accumulated angle is 1e6 * 0.01 = 10000 rad. GetAngle wraps it into
    // (-pi, pi]: 10000 - 1592*2*pi = -2.83098... rad. The float rounding of a
    // million steps is a random walk of ~1e-7*sqrt(1e6) = 1e-4 rad plus the
    // ~1e-9 rad per-step approximation of 0.01, so 1e-2 rad is a safe bound.
    const float expected = 10000.0f - 1592.0f * p2::kTwoPi;
    const float accumulated = q.GetAngle();
    EXPECT_NEAR(accumulated, expected, 1.0e-2f);
    EXPECT_GT(accumulated, -p2::kPi - kTolerance);
    EXPECT_LE(accumulated, p2::kPi + kTolerance);
}
