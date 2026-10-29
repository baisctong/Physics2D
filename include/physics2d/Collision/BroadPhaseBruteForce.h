#pragma once

// p2::BroadPhaseBruteForce — the O(N^2) reference broad phase (Week 6, P11).
//
// This is the baseline every optimisation is measured against: it stores one
// AABB per proxy in a flat slot array, and QueryPairs() compares every live pair.
// It exists for three reasons:
//   1. it is the correctness oracle for the SAP implementation ("the optimised
//      pair set must equal the naive pair set, bit for bit");
//   2. it is the fallback for tiny scenes, where SAP's endpoint maintenance costs
//      more than N^2/2 AABB comparisons (threshold measured, not guessed);
//   3. it keeps the bench honest: every number in the README has a naive baseline
//      measured on the same machine in the same build.
//
// Contract notes (see Core/BroadPhaseInterface.h):
//  - proxy ids are slot indices, reused through a free list; a removed proxy's
//    slot is never reported by QueryPairs again;
//  - QueryPairs emits (a, b) with a < b and each pair exactly once, in ascending
//    (a, b) order, so callers get a deterministic order for free;
//  - the class knows nothing about bodies/World: it only sees AABBs.
//
// Slot storage is intentionally the same shape as IDPool-free pools elsewhere in
// the engine (std::vector + free list instead of generation handles): proxy ids
// never escape the World's bookkeeping, so there is no ABA problem to solve here.

#include "physics2d/Core/BroadPhaseInterface.h"

#include <cstdint>
#include <vector>

namespace p2 {

class BroadPhaseBruteForce final : public IBroadPhase {
  public:
    BroadPhaseBruteForce() = default;
    ~BroadPhaseBruteForce() override = default;

    [[nodiscard]] int32_t AddProxy(const AABB& aabb) override;
    void RemoveProxy(int32_t proxyId) override;
    void UpdateAABB(int32_t proxyId, const AABB& aabb) override;
    void QueryPairs(const PairCallback& callback) override;

    [[nodiscard]] int32_t GetProxyCount() const override { return m_proxyCount; }
    [[nodiscard]] const char* Name() const override { return "BruteForce"; }

  private:
    // Slot arrays. A released slot keeps its (stale) AABB but m_alive[id] is
    // false, so it is skipped by the pair loop and the id is recycled from
    // m_freeList by the next AddProxy.
    std::vector<AABB> m_aabbs;
    std::vector<uint8_t> m_alive;
    std::vector<int32_t> m_freeList;
    int32_t m_proxyCount = 0;
};

} // namespace p2
