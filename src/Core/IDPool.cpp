// p2::IDPool — O(1) slot allocation with generation counters (Week 1 D6, P05).
//
// See IDPool.h for the design rationale. The implementation is a slot array
// plus a LIFO free list, and its correctness rests on two invariants:
//   1. a slot generation is bumped exactly once per release, and
//   2. a released slot appears on the free list exactly once.
// Both are asserted in debug builds: violating either silently revives stale
// handles (the ABA problem), which is the exact bug this class exists to stop.

#include "physics2d/Core/IDPool.h"

#include <cassert>
#include <cstddef>

namespace p2 {

int32_t IDPool::Allocate() {
    int32_t index = -1;
    if (!m_freeList.empty()) {
        // LIFO reuse: recently freed slots come back first. That keeps the hot
        // body arrays dense on the common create/destroy/create pattern and
        // costs O(1) with no search.
        index = m_freeList.back();
        m_freeList.pop_back();
        assert(m_entries[static_cast<std::size_t>(index)].alive == false &&
               "IDPool::Allocate: free list must only contain dead slots");
        m_entries[static_cast<std::size_t>(index)].alive = true;
    } else {
        // Grow by exactly one slot: index == size() keeps the index space
        // contiguous, which lets World index its own body storage directly.
        index = static_cast<int32_t>(m_entries.size());
        m_entries.emplace_back();
        m_entries.back().alive = true;
    }
    ++m_aliveCount;
    return index;
}

void IDPool::Release(int32_t index) {
    assert(IsValidIndex(index) && "IDPool::Release: slot index out of range");
    if (!IsValidIndex(index)) {
        return;
    }

    Entry& entry = m_entries[static_cast<std::size_t>(index)];
    assert(entry.alive && "IDPool::Release: double release of an already-dead slot");
    if (!entry.alive) {
        return;
    }

    entry.alive = false;
    // Bump on release (not on allocate) so that every handle handed out for the
    // old occupant becomes stale immediately, before the slot is reused.
    ++entry.generation;
    if (entry.generation == 0U) {
        // Generation 0 is reserved for the default (never allocated) handle, so
        // a 32-bit wrap after 2^32 releases must not alias it.
        entry.generation = 1U;
    }
    m_freeList.push_back(index);
    --m_aliveCount;
}

bool IDPool::IsValid(int32_t index) const {
    return IsValidIndex(index) && m_entries[static_cast<std::size_t>(index)].alive;
}

uint32_t IDPool::GetGeneration(int32_t index) const {
    assert(IsValidIndex(index) && "IDPool::GetGeneration: slot index out of range");
    if (!IsValidIndex(index)) {
        return 0U;
    }
    return m_entries[static_cast<std::size_t>(index)].generation;
}

void IDPool::Clear() {
    m_entries.clear();
    m_freeList.clear();
    m_aliveCount = 0;
}

} // namespace p2
