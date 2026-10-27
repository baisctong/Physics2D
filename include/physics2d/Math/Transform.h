#pragma once

// p2::Transform — rigid transform: rotation then translation (Section 4.2).
//
//   world = Mul(xf, local)  ==  Mul(xf.q, local) + xf.p
//   local = MulT(xf, world) ==  MulT(xf.q, world - xf.p)
//
// The engine keeps body position/angle in this form. Note that `p` is the
// body origin (not necessarily the centre of mass); shape mass properties
// store the centre of mass in local coordinates and the solver works with it.

#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Vec2.h"

namespace p2 {

struct Transform {
    Vec2 p{0.0f, 0.0f}; // translation (body origin in world space)
    Rot q{};            // rotation

    constexpr Transform() = default;
    constexpr Transform(Vec2 p_, Rot q_) : p(p_), q(q_) {}
};

// Transform a local point into world space.
[[nodiscard]] constexpr Vec2 Mul(const Transform& xf, Vec2 v) noexcept {
    return Mul(xf.q, v) + xf.p;
}

// Transform a world point into local space.
[[nodiscard]] constexpr Vec2 MulT(const Transform& xf, Vec2 v) noexcept {
    return MulT(xf.q, v - xf.p);
}

// Compose: Mul(a, b) applies b first, then a.
[[nodiscard]] constexpr Transform Mul(const Transform& a, const Transform& b) noexcept {
    return {Mul(a.q, b.p) + a.p, Mul(a.q, b.q)};
}

// a^T * b (relative transform).
[[nodiscard]] constexpr Transform MulT(const Transform& a, const Transform& b) noexcept {
    return {MulT(a.q, b.p - a.p), MulT(a.q, b.q)};
}

} // namespace p2
