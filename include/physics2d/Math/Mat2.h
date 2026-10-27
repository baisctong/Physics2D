#pragma once

// p2::Mat2 — 2x2 matrix stored column-major (Section 4.2, P03).
//
// Column-major is the convention used by OpenGL and by the rest of this engine:
// cx is the image of the x axis, cy is the image of the y axis, so
//   Mat2 * Vec2 = cx * v.x + cy * v.y
// which is exactly the formula we want in Mul(Mat2, Vec2) below.

#include "physics2d/Math/Vec2.h"

namespace p2 {

struct Mat2 {
    Vec2 cx{1.0f, 0.0f}; // first column (image of the x basis vector)
    Vec2 cy{0.0f, 1.0f}; // second column (image of the y basis vector)

    constexpr Mat2() = default;
    constexpr Mat2(Vec2 cx_, Vec2 cy_) : cx(cx_), cy(cy_) {}

    [[nodiscard]] static constexpr Mat2 Identity() noexcept { return {}; }
};

// Matrix * vector.
[[nodiscard]] constexpr Vec2 Mul(const Mat2& m, Vec2 v) noexcept {
    return m.cx * v.x + m.cy * v.y;
}

// Matrix * matrix (compose: result * v == a * (b * v)).
[[nodiscard]] constexpr Mat2 Mul(const Mat2& a, const Mat2& b) noexcept {
    return {Mul(a, b.cx), Mul(a, b.cy)};
}

[[nodiscard]] constexpr Mat2 Transpose(const Mat2& m) noexcept {
    return {{m.cx.x, m.cy.x}, {m.cx.y, m.cy.y}};
}

// Determinant of [[cx.x, cy.x], [cx.y, cy.y]].
[[nodiscard]] constexpr float Determinant(const Mat2& m) noexcept {
    return Cross(m.cx, m.cy);
}

// Inverse with an explicit success flag.
// Contract (P03):
//  - returns false when |determinant| is below the singularity threshold and
//    leaves `out` *untouched* in that case;
//  - singular input is a caller error, but it is reported, never silently
//    "fixed" (Section 3.4 error-handling strategy).
[[nodiscard]] bool Inverse(const Mat2& m, Mat2& out) noexcept;

// Convenience inverse used when the matrix is known to be invertible.
// In debug builds a singular matrix trips an assert; in release it returns
// Identity after asserting nothing, because the engine never feeds singular
// matrices here (rotation matrices are orthonormal).
[[nodiscard]] Mat2 GetInverse(const Mat2& m) noexcept;

} // namespace p2
