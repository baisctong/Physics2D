#pragma once

// p2::MathUtils — scalar helpers and the deterministic hashing used by the
// headless regression tool (Section 7.3 / P03 of the design document).
//
// Hashing rules (important for determinism):
//  - Hash the IEEE-754 *bit pattern* of floats. Never route through double and
//    never use std::hash<float>: the former changes the byte sequence, the
//    latter is implementation-defined across standard libraries.
//  - FNV-1a over bytes is the project's single hash; it is fast, has no
//    external dependency and its output is fully specified by this header.

#include <cstdint>
#include <cstring>

namespace p2 {

// ---- numeric constants -------------------------------------------------------

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 2.0f * kPi;
inline constexpr float kHalfPi = 0.5f * kPi;
inline constexpr float kDegToRad = kPi / 180.0f;
inline constexpr float kRadToDeg = 180.0f / kPi;

// Absolute tolerance used by normalisation guards and division guards.
// Section 4.4 lists kEpsilon under Settings.h; it lives here because the Math
// layer must not depend on Core (dependency direction: Core -> Math).
inline constexpr float kEpsilon = 1.0e-6f;
inline constexpr float kRelativeEpsilon = 1.0e-5f;

// ---- scalar helpers ----------------------------------------------------------

[[nodiscard]] constexpr float Clamp(float value, float lower, float upper) noexcept {
    return value < lower ? lower : (value > upper ? upper : value);
}

[[nodiscard]] constexpr float Lerp(float a, float b, float t) noexcept {
    return a + (b - a) * t;
}

// Smallest power of two >= value. NextPowerOfTwo(0) == 1, NextPowerOfTwo(1) == 1.
// Used by future broad-phase / island arrays; defined now so the contract is
// stable and testable.
[[nodiscard]] uint32_t NextPowerOfTwo(uint32_t value) noexcept;

// True when both values are finite. (std::isfinite without <cmath> in headers
// keeps this file cheap to include.)
[[nodiscard]] bool IsFinite(float value) noexcept;

// ---- FNV-1a hashing ----------------------------------------------------------

inline constexpr uint64_t kFnv1aOffsetBasis = 14695981039346656037ull;
inline constexpr uint64_t kFnv1aPrime = 1099511628211ull;

// One FNV-1a step over `size` bytes.
[[nodiscard]] uint64_t Fnv1aStep(uint64_t hash, const void* data, size_t size) noexcept;

// FNV-1a of an arbitrary byte range.
[[nodiscard]] uint64_t Fnv1a(const void* data, size_t size, uint64_t seed = kFnv1aOffsetBasis) noexcept;

// Hash one float by its bit pattern (no double conversion, no libm).
[[nodiscard]] uint64_t HashFloat(uint64_t hash, float value) noexcept;

// "Weak" hash: quantise to `quantum` metres/radians first, then hash the bits.
// Used by the regression tool for cross-platform comparison where bit-exact
// equality is not guaranteed (different FMA contraction on MSVC vs GCC).
// See Section 7.4 / P16 of the design document.
[[nodiscard]] uint64_t HashQuantizedFloat(uint64_t hash, float value, float quantum) noexcept;

} // namespace p2
