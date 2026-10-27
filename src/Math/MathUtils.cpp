// p2::MathUtils — scalar helpers and deterministic FNV-1a hashing
// (Section 7.3 / P03 of the design document).
//
// Determinism is the whole point of this file: the headless regression tool
// (Week 6-7) compares hashes between runs and between platforms, so the byte
// sequence fed to FNV-1a must not depend on the host's endianness, on libm, or
// on the standard library's std::hash (which is implementation-defined).

#include "physics2d/Math/MathUtils.h"

#include <array>
#include <bit>
#include <cmath>

namespace p2 {
namespace {

constexpr uint32_t kByteMask = 0xFFu;
constexpr uint32_t kBitsPerByte = 8u;

// Serialise a 32-bit word least-significant byte first.
//
// FNV-1a is only specified over a byte sequence, not over an integer, so the
// byte order must be pinned down here. A little-endian layout is chosen because
// that is the native order of every platform this engine currently targets
// (x86-64, ARM64), which makes the common case free once the compiler inlines
// this helper, while a big-endian host would still produce the same documented
// hash instead of silently diverging from the recorded baselines.
[[nodiscard]] std::array<unsigned char, 4> ToLittleEndianBytes(uint32_t bits) noexcept {
    return {static_cast<unsigned char>(bits & kByteMask),
            static_cast<unsigned char>((bits >> kBitsPerByte) & kByteMask),
            static_cast<unsigned char>((bits >> (2u * kBitsPerByte)) & kByteMask),
            static_cast<unsigned char>((bits >> (3u * kBitsPerByte)) & kByteMask)};
}

} // namespace

uint32_t NextPowerOfTwo(uint32_t value) noexcept {
    // Contract: the smallest power of two >= value, with NextPowerOfTwo(0) == 1
    // by definition (an empty array still needs one slot).
    if (value <= 1u) {
        return 1u;
    }
    // A power of two >= value only fits in uint32_t while value <= 2^31
    // (2^31 itself is representable, 2^31 + 1 would need 2^32). Returning 0 is
    // the documented "not representable" sentinel; it is checked explicitly
    // instead of letting the smearing trick silently wrap to 0.
    constexpr uint32_t kLargestPowerOfTwo = 1u << 31u;
    if (value > kLargestPowerOfTwo) {
        return 0u;
    }
    // Smear the highest set bit downwards, then add one:
    //   0b0001'0100 -> 0b0001'1111 -> 0b0010'0000.
    --value;
    value |= value >> 1u;
    value |= value >> 2u;
    value |= value >> 4u;
    value |= value >> 8u;
    value |= value >> 16u;
    return value + 1u;
}

bool IsFinite(float value) noexcept {
    return std::isfinite(value);
}

uint64_t Fnv1aStep(uint64_t hash, const void* data, size_t size) noexcept {
    const auto* bytes = static_cast<const unsigned char*>(data);
    // FNV-1a: xor the byte in first, then multiply by the prime. The multiply is
    // done in uint64_t, so it wraps modulo 2^64 exactly as the reference
    // implementation specifies.
    for (size_t i = 0; i < size; ++i) {
        hash ^= static_cast<uint64_t>(bytes[i]);
        hash *= kFnv1aPrime;
    }
    return hash;
}

uint64_t Fnv1a(const void* data, size_t size, uint64_t seed) noexcept {
    return Fnv1aStep(seed, data, size);
}

uint64_t HashFloat(uint64_t hash, float value) noexcept {
    // Hash the IEEE-754 bit pattern, never the numeric value: converting through
    // a double or formatting the float would lose the sign of zero and the NaN
    // payload, and would be implementation-defined across standard libraries.
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    const std::array<unsigned char, 4> bytes = ToLittleEndianBytes(bits);
    return Fnv1aStep(hash, bytes.data(), bytes.size());
}

uint64_t HashQuantizedFloat(uint64_t hash, float value, float quantum) noexcept {
    // A non-positive or non-finite quantum cannot partition the number line, so
    // there is nothing to quantise; hashing the raw bits keeps the function
    // total (and deterministic) instead of returning a poisoned value.
    if (!(quantum > 0.0f) || !IsFinite(quantum)) {
        return HashFloat(hash, value);
    }
    // Round to the nearest multiple of the quantum first, then hash. Values in
    // the same bucket must produce the identical bit pattern for the hashes to
    // match, which is why the multiplication happens after rounding rather than
    // hashing `value / quantum`.
    const float quantized = std::round(value / quantum) * quantum;
    return HashFloat(hash, quantized);
}

} // namespace p2
