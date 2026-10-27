// Unit tests for p2::MathUtils (Week 1 D2-D4, design document P03).
//
// The FNV-1a constants below are the canonical 64-bit reference vectors from
// the FNV specification plus constants computed once with an independent
// big-integer implementation of the recurrence; the derivation of each is in
// the comment. The hashing tests double as the determinism contract for the
// future headless regression tool (Section 7.3): same bytes in, same hash out.

#include <gtest/gtest.h>

#include "physics2d/Math/MathUtils.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>

namespace {

constexpr uint64_t kOffset = p2::kFnv1aOffsetBasis;

} // namespace

TEST(MathUtils, NextPowerOfTwoBoundaries) {
    // Definition: the smallest power of two >= value, with NextPowerOfTwo(0) = 1
    // (an empty buffer still needs one slot).
    EXPECT_EQ(p2::NextPowerOfTwo(0u), 1u);
    EXPECT_EQ(p2::NextPowerOfTwo(1u), 1u);
    EXPECT_EQ(p2::NextPowerOfTwo(2u), 2u);
    EXPECT_EQ(p2::NextPowerOfTwo(3u), 4u);
    EXPECT_EQ(p2::NextPowerOfTwo(4u), 4u);
    EXPECT_EQ(p2::NextPowerOfTwo(5u), 8u);
    EXPECT_EQ(p2::NextPowerOfTwo(7u), 8u);
    EXPECT_EQ(p2::NextPowerOfTwo(8u), 8u);
    EXPECT_EQ(p2::NextPowerOfTwo(9u), 16u);
    EXPECT_EQ(p2::NextPowerOfTwo(1000u), 1024u);
    EXPECT_EQ(p2::NextPowerOfTwo(1024u), 1024u);
    EXPECT_EQ(p2::NextPowerOfTwo(1025u), 2048u);
    EXPECT_EQ(p2::NextPowerOfTwo(65535u), 65536u);
    EXPECT_EQ(p2::NextPowerOfTwo(65536u), 65536u);

    // 2^31 is the largest power of two representable in uint32_t.
    EXPECT_EQ(p2::NextPowerOfTwo(1u << 31u), 1u << 31u);
    // 2^31 + 1 would need 2^32, which does not fit: 0 is the documented
    // "not representable" sentinel.
    EXPECT_EQ(p2::NextPowerOfTwo((1u << 31u) + 1u), 0u);
    EXPECT_EQ(p2::NextPowerOfTwo(0xFFFFFFFFu), 0u);
}

TEST(MathUtils, NextPowerOfTwoProperties) {
    for (uint32_t k = 0u; k <= 31u; ++k) {
        const uint32_t power = 1u << k;
        // Powers of two are fixed points.
        EXPECT_EQ(p2::NextPowerOfTwo(power), power) << "k=" << k;
        // 2^k - 1 rounds up to 2^k for k >= 2; for k == 1 the input 1 is already
        // a power of two (NextPowerOfTwo(1) == 1 by definition).
        if (k > 1u) {
            EXPECT_EQ(p2::NextPowerOfTwo(power - 1u), power) << "k=" << k;
        }
        // 2^k + 1 rounds up to 2^(k+1) while that still fits.
        if (k < 31u) {
            EXPECT_EQ(p2::NextPowerOfTwo(power + 1u), power << 1u) << "k=" << k;
        }
    }

    // General property for a spread of representable inputs: the result is >=
    // the input and has exactly one bit set.
    for (uint32_t value = 1u; value < (1u << 31u); value += 7919u) {
        const uint32_t result = p2::NextPowerOfTwo(value);
        EXPECT_GE(result, value);
        EXPECT_EQ(result & (result - 1u), 0u) << "value=" << value;
    }
}

TEST(MathUtils, ClampAndLerp) {
    // Clamp is inclusive at both ends.
    EXPECT_FLOAT_EQ(p2::Clamp(5.0f, 0.0f, 10.0f), 5.0f);
    EXPECT_FLOAT_EQ(p2::Clamp(-3.0f, 0.0f, 10.0f), 0.0f);
    EXPECT_FLOAT_EQ(p2::Clamp(11.0f, 0.0f, 10.0f), 10.0f);
    EXPECT_FLOAT_EQ(p2::Clamp(0.0f, 0.0f, 10.0f), 0.0f);
    EXPECT_FLOAT_EQ(p2::Clamp(10.0f, 0.0f, 10.0f), 10.0f);

    // Lerp(a, b, 0) == a and Lerp(a, b, 1) == b for exactly representable
    // endpoints: a + (b - a) * t.
    EXPECT_FLOAT_EQ(p2::Lerp(2.0f, 8.0f, 0.0f), 2.0f);
    EXPECT_FLOAT_EQ(p2::Lerp(2.0f, 8.0f, 1.0f), 8.0f);
    // t = 0.25: 0 + (10 - 0) * 0.25 = 2.5.
    EXPECT_FLOAT_EQ(p2::Lerp(0.0f, 10.0f, 0.25f), 2.5f);
    // t = 0.5: -4 + (4 - -4) * 0.5 = -4 + 4 = 0.
    EXPECT_FLOAT_EQ(p2::Lerp(-4.0f, 4.0f, 0.5f), 0.0f);
    // Extrapolation is defined, not clamped: 0 + 10*2 = 20.
    EXPECT_FLOAT_EQ(p2::Lerp(0.0f, 10.0f, 2.0f), 20.0f);
}

TEST(MathUtils, IsFiniteRejectsNaNAndInfinity) {
    EXPECT_TRUE(p2::IsFinite(0.0f));
    EXPECT_TRUE(p2::IsFinite(-0.0f));
    EXPECT_TRUE(p2::IsFinite(1.0f));
    EXPECT_TRUE(p2::IsFinite(-123.5f));
    EXPECT_TRUE(p2::IsFinite(std::numeric_limits<float>::max()));
    EXPECT_TRUE(p2::IsFinite(std::numeric_limits<float>::lowest()));

    EXPECT_FALSE(p2::IsFinite(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(p2::IsFinite(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(p2::IsFinite(-std::numeric_limits<float>::infinity()));
}

TEST(MathUtils, Fnv1aMatchesReferenceVectors) {
    // Canonical FNV-1a 64-bit test vectors (FNV specification). Each starts
    // from the 64-bit offset basis 14695981039346656037 = 0xcbf29ce484222325.
    EXPECT_EQ(p2::Fnv1a("", 0u), 0xcbf29ce484222325ULL);
    EXPECT_EQ(p2::Fnv1a("a", 1u), 0xaf63dc4c8601ec8cULL);
    EXPECT_EQ(p2::Fnv1a("b", 1u), 0xaf63df4c8601f1a5ULL);
    EXPECT_EQ(p2::Fnv1a("abc", 3u), 0xe71fa2190541574bULL);
    EXPECT_EQ(p2::Fnv1a("foobar", 6u), 0x85944171f73967e8ULL);

    // The default seed is the offset basis, and the explicit form agrees.
    EXPECT_EQ(p2::Fnv1a("foobar", 6u), p2::Fnv1a("foobar", 6u, kOffset));
}

TEST(MathUtils, Fnv1aSeedAndIncrementalStepping) {
    // Seed 0 starts the recurrence from h = 0 instead of the offset basis:
    //   h = ((0 ^ 'a') * 1099511628211) ^ 'b' ... * 1099511628211
    // For "abc" that yields 0x47c3ec01db8517e0 (computed once with an
    // independent big-integer implementation).
    EXPECT_EQ(p2::Fnv1a("abc", 3u, 0u), 0x47c3ec01db8517e0ULL);

    // Stepwise hashing of "a" then "bc" must equal hashing "abc" in one call,
    // which is what lets the regression tool stream a float array.
    uint64_t streamed = 42u;
    streamed = p2::Fnv1aStep(streamed, "a", 1u);
    streamed = p2::Fnv1aStep(streamed, "bc", 2u);
    EXPECT_EQ(streamed, 0x886776016ee7fe5eULL); // seed 42, "abc"

    // Arbitrary bytes (not just text) hash deterministically.
    const unsigned char bytes[] = {1u, 2u, 3u, 4u};
    EXPECT_EQ(p2::Fnv1a(bytes, 4u), 0xbe7a5e775165785dULL);
}

TEST(MathUtils, HashFloatUsesIeee754BitPattern) {
    // 1.0f is 0x3F800000; hashing its little-endian bytes (00 00 80 3F) with
    // FNV-1a from the offset basis gives 0x4b72477f9c5c2f98 (independently
    // verified with a Python big-integer implementation of the recurrence).
    EXPECT_EQ(p2::HashFloat(kOffset, 1.0f), 0x4b72477f9c5c2f98ULL);
    // 2.0f is 0x40000000 -> 00 00 00 40.
    EXPECT_EQ(p2::HashFloat(kOffset, 2.0f), 0x4d25b67f9dce80b5ULL);

    // +0.0f == -0.0f numerically but their bit patterns differ (0x00000000 vs
    // 0x80000000), and the hasher must see the difference - this is exactly the
    // information a double conversion would destroy.
    EXPECT_EQ(p2::HashFloat(kOffset, 0.0f), 0x4d25767f9dce13f5ULL);
    EXPECT_EQ(p2::HashFloat(kOffset, -0.0f), 0x4d24f67f9dcd3a75ULL);
    EXPECT_NE(p2::HashFloat(kOffset, 0.0f), p2::HashFloat(kOffset, -0.0f));

    // Layout check: the four bytes consumed must be exactly the little-endian
    // pattern, in that order.
    const unsigned char pattern[] = {0x00u, 0x00u, 0x80u, 0x3Fu};
    EXPECT_EQ(p2::HashFloat(kOffset, 1.0f), p2::Fnv1a(pattern, 4u));

    // The seed is threaded through, so hashes can be chained.
    const unsigned char pattern2[] = {0x00u, 0x00u, 0x00u, 0x40u};
    EXPECT_EQ(p2::HashFloat(0u, 2.0f), p2::Fnv1a(pattern2, 4u, 0u));

    // Same bit pattern -> same hash (NaN payloads included), reproducibly.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(p2::HashFloat(kOffset, nan), p2::HashFloat(kOffset, nan));
}

TEST(MathUtils, HashQuantizedFloatBuckets) {
    // quantum 0.5 keeps the arithmetic exact in binary:
    //   1.1f / 0.5f = 2.2 -> round 2 -> 2 * 0.5 = 1.0
    //   0.9f / 0.5f = 1.8 -> round 2 -> 1.0
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.1f, 0.5f), p2::HashFloat(kOffset, 1.0f));
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 0.9f, 0.5f), p2::HashFloat(kOffset, 1.0f));
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.1f, 0.5f), p2::HashQuantizedFloat(kOffset, 0.9f, 0.5f));

    // 1.6f / 0.5f = 3.2 -> round 3 -> 1.5, a different bucket.
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.6f, 0.5f), p2::HashFloat(kOffset, 1.5f));
    EXPECT_NE(p2::HashQuantizedFloat(kOffset, 1.1f, 0.5f), p2::HashQuantizedFloat(kOffset, 1.6f, 0.5f));

    // 0.2f / 0.5f = 0.4 -> round 0 -> +0.0f.
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 0.2f, 0.5f), p2::HashFloat(kOffset, 0.0f));

    // A non-positive quantum cannot partition the line: the documented fallback
    // is the raw bit hash.
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.2345f, 0.0f), p2::HashFloat(kOffset, 1.2345f));
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.2345f, -1.0f), p2::HashFloat(kOffset, 1.2345f));

    // Quantisation is reproducible for a non-power-of-two quantum.
    EXPECT_EQ(p2::HashQuantizedFloat(kOffset, 1.2345f, 0.001f),
              p2::HashQuantizedFloat(kOffset, 1.2345f, 0.001f));
}

TEST(MathUtils, FloatSequenceHashIsReproducibleAndOrderSensitive) {
    const float values[] = {0.0f, 1.0f, -1.5f, 3.25f, -7.75f};

    const auto hashSequence = [&values]() {
        uint64_t hash = kOffset;
        for (const float value : values) {
            hash = p2::HashFloat(hash, value);
        }
        return hash;
    };
    const auto hashReversed = [&values]() {
        uint64_t hash = kOffset;
        for (auto it = std::rbegin(values); it != std::rend(values); ++it) {
            hash = p2::HashFloat(hash, *it);
        }
        return hash;
    };

    // Same byte sequence -> identical hash, run after run.
    EXPECT_EQ(hashSequence(), hashSequence());
    // FNV-1a is order-sensitive: reversing the sequence must change the hash.
    EXPECT_NE(hashSequence(), hashReversed());
}
