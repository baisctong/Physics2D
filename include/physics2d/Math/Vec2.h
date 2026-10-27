#pragma once

// p2::Vec2 — 2D vector, value semantics, SIMD-friendly layout.
//
// Design notes (Section 3.2 / P02 of the design document):
//  - `float`, not `double`: the whole engine is single precision. Mixing the
//    two silently in hot paths is a classic source of bad codegen.
//  - Everything that can be `constexpr` is header-inline. There is no Vec2.cpp:
//    one 8-byte POD with all operations visible to the optimiser is faster and
//    simpler than a translation unit boundary, and it makes the type usable in
//    `constexpr` test fixtures.
//  - Free functions (`Dot`, `Cross`, ...) follow the Box2D/P02 convention; the
//    design doc fixes the three Cross overloads because mixing them up is the
//    single most common AI-generated bug in a 2D engine (Section 4.2).

#include <cmath>
#include <cstddef>

namespace p2 {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2& operator+=(const Vec2& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    constexpr Vec2& operator-=(const Vec2& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    constexpr Vec2& operator*=(float s) noexcept {
        x *= s;
        y *= s;
        return *this;
    }
    constexpr Vec2& operator/=(float s) noexcept {
        // No protection here: dividing by zero is a caller bug and must be
        // loud (FP exception / NaN caught by World::CheckSane) rather than
        // silently producing a wrong vector.
        x /= s;
        y /= s;
        return *this;
    }
};

// ---- Vec2 op Vec2 -----------------------------------------------------------

[[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept {
    return {a.x + b.x, a.y + b.y};
}
[[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept {
    return {a.x - b.x, a.y - b.y};
}
[[nodiscard]] constexpr Vec2 operator-(Vec2 v) noexcept {
    return {-v.x, -v.y};
}
[[nodiscard]] constexpr bool operator==(Vec2 a, Vec2 b) noexcept {
    // Exact comparison on purpose: this is only used for tests / degenerate
    // checks, never for physical classification (which needs tolerances).
    return a.x == b.x && a.y == b.y;
}
[[nodiscard]] constexpr bool operator!=(Vec2 a, Vec2 b) noexcept {
    return !(a == b);
}

// ---- Vec2 op scalar (both sides) --------------------------------------------

[[nodiscard]] constexpr Vec2 operator*(Vec2 v, float s) noexcept {
    return {v.x * s, v.y * s};
}
[[nodiscard]] constexpr Vec2 operator*(float s, Vec2 v) noexcept {
    return {v.x * s, v.y * s};
}
[[nodiscard]] constexpr Vec2 operator/(Vec2 v, float s) noexcept {
    return {v.x / s, v.y / s};
}
[[nodiscard]] constexpr Vec2 operator/(float s, Vec2 v) noexcept {
    // Component-wise scalar/vector is not a standard operation; kept only so
    // the overload set is symmetric with operator*. Use it rarely.
    return {s / v.x, s / v.y};
}

// ---- products ---------------------------------------------------------------

[[nodiscard]] constexpr float Dot(Vec2 a, Vec2 b) noexcept {
    return a.x * b.x + a.y * b.y;
}

// Three Cross flavours. Section 4.2 of the design document is explicit about
// this because mixing the scalar and vector forms is the #1 AI bug here.
//   Cross(a, b)  -> float : z component of the 3D cross product (a.x*b.y - a.y*b.x)
//   Cross(a, s)  -> Vec2  : (s*a.y, -s*a.x)   == -(s * Skew(a))
//   Cross(s, a)  -> Vec2  : (-s*a.y, s*a.x)   ==  s * Skew(a)
[[nodiscard]] constexpr float Cross(Vec2 a, Vec2 b) noexcept {
    return a.x * b.y - a.y * b.x;
}
[[nodiscard]] constexpr Vec2 Cross(Vec2 a, float s) noexcept {
    return {s * a.y, -s * a.x};
}
[[nodiscard]] constexpr Vec2 Cross(float s, Vec2 a) noexcept {
    return {-s * a.y, s * a.x};
}

// Skew(v) = (-v.y, v.x) = 90 degrees CCW rotation = Cross(1, v).
[[nodiscard]] constexpr Vec2 Skew(Vec2 v) noexcept {
    return {-v.y, v.x};
}

// ---- lengths ----------------------------------------------------------------

[[nodiscard]] constexpr float LengthSquared(Vec2 v) noexcept {
    return Dot(v, v);
}

[[nodiscard]] inline float Length(Vec2 v) noexcept {
    return std::sqrt(LengthSquared(v));
}

[[nodiscard]] constexpr float DistanceSquared(Vec2 a, Vec2 b) noexcept {
    return LengthSquared(a - b);
}

[[nodiscard]] inline float Distance(Vec2 a, Vec2 b) noexcept {
    return std::sqrt(DistanceSquared(a, b));
}

// Normalized() returns (0,0) for a (near-)zero vector instead of NaN.
// Rationale (P02): every normalisation in the engine feeds a direction into a
// solver; a NaN here poisons the entire island within one frame and the root
// cause is very hard to find later. Zero-length input is a legitimate
// degenerate case (coincident contact points), so we define a total function.
[[nodiscard]] inline Vec2 Normalized(Vec2 v) noexcept {
    constexpr float kNormalizeEpsilonSquared = 1.0e-12f;
    const float lengthSquared = LengthSquared(v);
    if (lengthSquared < kNormalizeEpsilonSquared) {
        return {0.0f, 0.0f};
    }
    const float invLength = 1.0f / std::sqrt(lengthSquared);
    return v * invLength;
}

// ---- component-wise helpers --------------------------------------------------

[[nodiscard]] constexpr Vec2 Min(Vec2 a, Vec2 b) noexcept {
    return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y};
}
[[nodiscard]] constexpr Vec2 Max(Vec2 a, Vec2 b) noexcept {
    return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y};
}
[[nodiscard]] constexpr Vec2 Abs(Vec2 v) noexcept {
    return {v.x < 0.0f ? -v.x : v.x, v.y < 0.0f ? -v.y : v.y};
}
[[nodiscard]] constexpr Vec2 Clamp(Vec2 v, Vec2 lower, Vec2 upper) noexcept {
    return {v.x < lower.x ? lower.x : (v.x > upper.x ? upper.x : v.x),
            v.y < lower.y ? lower.y : (v.y > upper.y ? upper.y : v.y)};
}
[[nodiscard]] constexpr Vec2 Lerp(Vec2 a, Vec2 b, float t) noexcept {
    return a + (b - a) * t;
}

// IsValid() is the boundary between "user gave us garbage" and "we produced
// NaN from a solver explosion": callers use it to reject input, tests use it
// to assert output sanity.
[[nodiscard]] inline bool IsValid(Vec2 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

} // namespace p2
