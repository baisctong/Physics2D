#pragma once

// p2::BroadPhaseSAP — Sweep and Prune broad phase (Week 6, P12 / Section 6.2).
//
// One-dimensional sort plus active-set sweep:
//  1. every live proxy contributes two endpoints on the x axis:
//     (aabb.lowerBound.x, id, MIN) and (aabb.upperBound.x, id, MAX);
//  2. the endpoint array is sorted once (std::sort on the first frame), then kept
//     sorted incrementally: UpdateAABB moves the two endpoints by bubbling them
//     left/right, which is insertion sort exploiting temporal coherence (a
//     resting body's endpoints do not move at all, so a frame is O(N + K));
//  3. scan: MIN -> test against every proxy in the active set with the full
//     AABB::Overlaps (the y axis is the second filter; skipping it produces a
//     flood of false positives that the narrow phase has to pay for); MAX ->
//     remove from the active set.
//
// Why endpoints are packed into ONE array instead of three parallel arrays (SoA):
// the incremental update is a sequence of swaps, and swapping one 12-byte
// Endpoint keeps value/id/isMin together - with SoA every swap would touch three
// arrays and the sort comparator would be cache-hostile.
//
// Why the incremental path exists at all: rebuilding with std::sort every frame
// is O(N log N) regardless of coherence; after warm-up the array is nearly
// sorted, so bubble moves cost O(1)-O(log N) per endpoint on average. The bench
// (bench/bench_broadphase.cpp) measures both paths so the claim is data, not
// folklore.
//
// Contract notes (see Core/BroadPhaseInterface.h):
//  - proxy ids are slot indices, reused through a free list; removed proxies are
//    never reported again;
//  - QueryPairs emits (a, b) with a < b, each pair exactly once, in ascending
//    (a, b) order (the active set is kept sorted by proxy id for exactly this
//    reason: callers get deterministic order for free);
//  - the implementation is point-swept: this class knows nothing about bodies.
//
// Scale note: the active set is a std::vector<int32_t> kept sorted; for the
// 50-2000 proxy range of the milestone this is faster than a hash set and keeps
// the order deterministic. A fixed-capacity bitset is the documented upgrade path
// for >10k proxies (P12 requirement: document the scale limit).

#include "physics2d/Core/BroadPhaseInterface.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace p2 {

class BroadPhaseSAP final : public IBroadPhase {
  public:
    BroadPhaseSAP() = default;
    ~BroadPhaseSAP() override = default;

    [[nodiscard]] int32_t AddProxy(const AABB& aabb) override;
    void RemoveProxy(int32_t proxyId) override;
    void UpdateAABB(int32_t proxyId, const AABB& aabb) override;
    void QueryPairs(const PairCallback& callback) override;

    [[nodiscard]] int32_t GetProxyCount() const override { return m_proxyCount; }
    [[nodiscard]] const char* Name() const override { return "SAP"; }

    // Debug/test hook required by P12: proves the internal representation is
    // consistent at any time:
    //  - every live proxy has exactly one MIN and one MAX endpoint;
    //  - the endpoint array is sorted by value (ties broken consistently);
    //  - the endpoint indices cached in each proxy agree with the real positions;
    //  - the number of MIN endpoints == number of MAX endpoints == live proxies.
    // Compiled in all builds (it is a contract, not an assert); O(N) per call, so
    // only tests and debug UI use it.
    [[nodiscard]] bool ValidateInternalState() const;

  private:
    // One axis endpoint. 12 bytes: value + id + flag, all swapped together by the
    // incremental insertion moves.
    struct Endpoint {
        float value = 0.0f;
        int32_t proxyId = -1;
        uint8_t isMin = 0;
    };

    struct Proxy {
        AABB aabb{};
        int32_t minIndex = -1; // position of the MIN endpoint in m_endpoints
        int32_t maxIndex = -1; // position of the MAX endpoint in m_endpoints
        bool alive = false;
    };

    // Ordering of the endpoint array. Value first; on equal values a MIN endpoint
    // sorts BEFORE a MAX endpoint (the inclusive AABB overlap convention: a pair
    // whose upper.x equals the other's lower.x must still be reported, so the
    // active set may not drop the closing interval before the opening one is
    // processed). Ties of the same kind sort by proxy id so the array order is
    // deterministic and the cached indices are reproducible.
    [[nodiscard]] static bool EndpointLess(const Endpoint& a, const Endpoint& b) noexcept;

    // Restores the "endpoints sorted + cached indices correct" invariant after a
    // bulk of AddProxy calls (which append at the tail and mark the array dirty).
    // const on purpose: ValidateInternalState is a const contract hook and must be
    // able to restore the invariant it is about to check; the two arrays are
    // mutable only for this lazy-sort path.
    void EnsureSorted() const;

    // Insertion-sort step for one endpoint whose value just changed. The array was
    // sorted before the change, so the endpoint only needs a contiguous bubble walk
    // (left if it shrank, right if it grew) - that is the temporal-coherence win.
    void BubbleEndpoint(int32_t endpointIndex);

    // Swaps two endpoints and keeps the endpoint indices cached in their proxies
    // pointing at the new positions.
    void SwapEndpoints(int32_t firstIndex, int32_t secondIndex);

    // Refresh one proxy's cached endpoint index from the endpoint at `endpointIndex`.
    void RefreshCachedIndex(int32_t endpointIndex) const;

    // Bulk-insert path: endpoints are appended at the tail and m_endpointsSorted is
    // cleared; the next UpdateAABB/QueryPairs/ValidateInternalState pays ONE
    // std::sort for the whole batch instead of an O(N^2) insertion sort during the
    // first frame's proxy creation.
    mutable std::vector<Endpoint> m_endpoints;
    mutable std::vector<Proxy> m_proxies;
    std::vector<int32_t> m_freeList;
    int32_t m_proxyCount = 0;
    mutable bool m_endpointsSorted = true;

    // Scan scratch, reused every frame: QueryPairs must not allocate on the hot
    // path (the benchmark measures exactly this path).
    std::vector<int32_t> m_active; // active set, kept sorted by proxy id
    std::vector<std::pair<int32_t, int32_t>> m_pairScratch;
};

} // namespace p2
