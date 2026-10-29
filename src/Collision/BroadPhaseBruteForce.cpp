// p2::BroadPhaseBruteForce — Week 6 baseline broad phase (P11).
//
// The implementation is deliberately the simplest version with the production
// contract attached: slot array + free list, A < B dedupe, deterministic order.
// Every optimisation in Week 6 is graded against this file, so it is kept small,
// obvious and fast enough (the inner loop is two Overlaps() calls and nothing
// else).

#include "physics2d/Collision/BroadPhaseBruteForce.h"

#include <cassert>
#include <cstddef>

namespace p2 {

int32_t BroadPhaseBruteForce::AddProxy(const AABB& aabb) {
    assert(aabb.IsValid() && "BroadPhaseBruteForce::AddProxy: invalid AABB");

    int32_t proxyId = -1;
    if (!m_freeList.empty()) {
        // Reuse a released slot: the id stays bounded by the peak proxy count, so
        // a long-running scene does not grow the array without bound.
        proxyId = m_freeList.back();
        m_freeList.pop_back();
    } else {
        proxyId = static_cast<int32_t>(m_aabbs.size());
        m_aabbs.emplace_back();
        m_alive.push_back(0);
    }
    m_aabbs[static_cast<std::size_t>(proxyId)] = aabb;
    m_alive[static_cast<std::size_t>(proxyId)] = 1;
    ++m_proxyCount;
    return proxyId;
}

void BroadPhaseBruteForce::RemoveProxy(int32_t proxyId) {
    assert(proxyId >= 0 && static_cast<std::size_t>(proxyId) < m_aabbs.size() &&
           "BroadPhaseBruteForce::RemoveProxy: unknown proxy id");
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= m_aabbs.size()) {
        return;
    }
    if (m_alive[static_cast<std::size_t>(proxyId)] == 0) {
        return; // idempotent: releasing a dead proxy is a caller bug, not a crash
    }
    m_alive[static_cast<std::size_t>(proxyId)] = 0;
    m_aabbs[static_cast<std::size_t>(proxyId)] = AABB{};
    m_freeList.push_back(proxyId);
    --m_proxyCount;
}

void BroadPhaseBruteForce::UpdateAABB(int32_t proxyId, const AABB& aabb) {
    assert(proxyId >= 0 && static_cast<std::size_t>(proxyId) < m_aabbs.size() &&
           "BroadPhaseBruteForce::UpdateAABB: unknown proxy id");
    assert(aabb.IsValid() && "BroadPhaseBruteForce::UpdateAABB: invalid AABB");
    assert(m_alive[static_cast<std::size_t>(proxyId)] != 0 &&
           "BroadPhaseBruteForce::UpdateAABB: proxy has been removed");
    m_aabbs[static_cast<std::size_t>(proxyId)] = aabb;
}

void BroadPhaseBruteForce::QueryPairs(const PairCallback& callback) {
    // Fixed i < j iteration: each unordered pair is visited exactly once and the
    // emission order is ascending (i, j), which is exactly the canonical order the
    // World relies on to keep the narrow phase deterministic.
    const int32_t count = static_cast<int32_t>(m_aabbs.size());
    for (int32_t i = 0; i < count; ++i) {
        if (m_alive[static_cast<std::size_t>(i)] == 0) {
            continue;
        }
        for (int32_t j = i + 1; j < count; ++j) {
            if (m_alive[static_cast<std::size_t>(j)] == 0) {
                continue;
            }
            if (Overlaps(m_aabbs[static_cast<std::size_t>(i)], m_aabbs[static_cast<std::size_t>(j)])) {
                callback(i, j);
            }
        }
    }
}

} // namespace p2
