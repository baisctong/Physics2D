#pragma once

// p2::Handle — generation-counted handle instead of a raw pointer or index
// (Section 3.2 / P05).
//
// Why not pointers: bodies live in contiguous arrays that are compacted and
// reused; a pointer would dangle the moment a body is destroyed and a new body
// takes its slot. Why not a bare int32 index: the index alone cannot tell
// "slot 7 from generation 2" apart from "slot 7 from generation 5" (the ABA
// problem), so destroying and recreating a body would silently revive stale
// references.
//
// A Handle is therefore {index, generation}: the pool bumps `generation` on
// every release, so a stale handle fails IsValid() instead of corrupting a new
// body. Handles are trivially copyable and cheap to validate - they are what
// the public API exchanges.
//
// Reserved: ShapeHandle is used from Week 2 onwards (shape storage), so the
// tag type and typedef already exist here to keep the Body struct stable.

#include <cstddef>
#include <cstdint>

namespace p2 {

template<typename Tag> struct Handle {
    int32_t index = -1;
    uint32_t generation = 0;

    [[nodiscard]] constexpr bool IsValid() const noexcept { return index >= 0; }

    friend constexpr bool operator==(const Handle& a, const Handle& b) noexcept {
        return a.index == b.index && a.generation == b.generation;
    }
    friend constexpr bool operator!=(const Handle& a, const Handle& b) noexcept { return !(a == b); }
};

struct BodyTag;
struct ShapeTag;

using BodyHandle = Handle<BodyTag>;
using ShapeHandle = Handle<ShapeTag>;

// Hash functor for the (rare) places that need handles in hash containers.
// The engine's hot paths use vectors + explicit indices (determinism, Section
// 7.4), so this exists for tooling, not for the solver.
struct BodyHandleHash {
    [[nodiscard]] std::size_t operator()(const BodyHandle& handle) const noexcept {
        return (static_cast<std::size_t>(static_cast<uint32_t>(handle.index)) << 32) ^
               static_cast<std::size_t>(handle.generation);
    }
};

} // namespace p2
