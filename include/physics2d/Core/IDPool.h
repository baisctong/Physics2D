#pragma once

// p2::IDPool — O(1) slot allocation with generation counters (P05).
//
// The pool owns the mapping "dense external handle index -> slot state". Its
// only job is lifecycle bookkeeping; body data itself lives in World's vectors
// indexed by the same integer, so indexes stay compact (important for the
// future SoA split of Section 3.3).
//
// Layout is intentionally simple:
//   m_entries : {generation, alive} per slot
//   m_freeList: LIFO stack of released slots
// Allocation pops the free list (cache-friendly: recently freed slots come back
// first) or appends a new slot. Release bumps the slot generation, which is what
// makes stale handles detectable.

#include <cstdint>
#include <vector>

namespace p2 {

class IDPool {
  public:
    IDPool() = default;

    // Returns a slot index >= 0, always. The pool grows on demand; there is no
    // fixed capacity. (World bounds its own arrays by this index.)
    [[nodiscard]] int32_t Allocate();

    // Returns the slot to the free list. Releasing an already-dead slot is a
    // caller bug: it trips an assert in debug builds and is a no-op in release.
    void Release(int32_t index);

    // True when the slot is currently allocated.
    [[nodiscard]] bool IsValid(int32_t index) const;

    // Generation that a live handle for this slot must carry. For a dead slot
    // this is the generation that *was* valid (already bumped on release).
    [[nodiscard]] uint32_t GetGeneration(int32_t index) const;

    [[nodiscard]] bool IsValidIndex(int32_t index) const noexcept {
        return index >= 0 && index < static_cast<int32_t>(m_entries.size());
    }

    [[nodiscard]] int32_t GetAliveCount() const noexcept { return m_aliveCount; }

    [[nodiscard]] int32_t GetCapacity() const noexcept { return static_cast<int32_t>(m_entries.size()); }

    void Clear();

  private:
    struct Entry {
        uint32_t generation = 1;
        bool alive = false;
    };

    std::vector<Entry> m_entries;
    std::vector<int32_t> m_freeList;
    int32_t m_aliveCount = 0;
};

} // namespace p2
