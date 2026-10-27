#pragma once

// p2::Rot — 2D rotation stored as (cos, sin) instead of an angle.
//
// Why (c, s) and not an angle (Section 6.1):
//  - An angle accumulates error under repeated integration and needs wrapping;
//    a unit complex number can be renormalised cheaply and stays orthonormal.
//  - Rotating a vector becomes 4 multiplications with no sin/cos call on the
//    hot path. Trig is only paid when converting to/from a user-facing angle.
// Invariant: c*c + s*s == 1 up to floating-point error. Normalize() restores it.

#include "physics2d/Math/Vec2.h"

namespace p2 {

struct Rot {
    float c = 1.0f; // cos(angle)
    float s = 0.0f; // sin(angle)

    Rot() = default;

    // Explicit so that `Rot q = 0.5f;` (which reads like an angle but silently
    // constructs a rotation) is a compile error.
    explicit Rot(float angle) noexcept;
    constexpr Rot(float c_, float s_) noexcept : c(c_), s(s_) {}

    void Set(float angle) noexcept;
    void SetIdentity() noexcept {
        c = 1.0f;
        s = 0.0f;
    }

    [[nodiscard]] float GetAngle() const noexcept;
    [[nodiscard]] Vec2 GetAxisX() const noexcept { return {c, s}; }
    [[nodiscard]] Vec2 GetAxisY() const noexcept { return {-s, c}; }

    // Renormalise to restore c^2 + s^2 == 1 after long chains of Mul().
    void Normalize() noexcept;
};

// Rotate a vector by q (q * v).
[[nodiscard]] constexpr Vec2 Mul(const Rot& q, Vec2 v) noexcept {
    return {q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// Inverse-rotate a vector by q (q^T * v).
[[nodiscard]] constexpr Vec2 MulT(const Rot& q, Vec2 v) noexcept {
    return {q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

// Compose two rotations (q = a * b, i.e. apply b then a).
[[nodiscard]] constexpr Rot Mul(const Rot& a, const Rot& b) noexcept {
    return {a.c * b.c - a.s * b.s, a.s * b.c + a.c * b.s};
}

// a^T * b.
[[nodiscard]] constexpr Rot MulT(const Rot& a, const Rot& b) noexcept {
    return {a.c * b.c + a.s * b.s, a.c * b.s - a.s * b.c};
}

} // namespace p2
