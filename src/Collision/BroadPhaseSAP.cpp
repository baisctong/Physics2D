// p2::BroadPhaseSAP — Sweep and Prune, real implementation (Week 6, P12 / 6.2).
//
// Data layout (one packed array):
//   m_endpoints: [(x_min, id, MIN), (x_max, id, MAX)] for every live proxy, kept
//   sorted by x. Every proxy caches the two array indices of its endpoints, so a
//   swap keeps O(1) bookkeeping per moved endpoint.
//
// Two update paths, chosen by the coherence of the scene:
//  - BULK (first frame / many new bodies): AddProxy appends both endpoints at the
//    tail and clears the sorted flag; the next UpdateAABB/QueryPairs/Validate pays
//    ONE std::sort for the whole batch. Sorting an unsorted array of 2N endpoints
//    is O(N log N); inserting them one by one would be O(N^2).
//  - INCREMENTAL (steady state): UpdateAABB writes the new x into the two existing
//    endpoints and bubbles each one to its sorted position. A resting body's
//    endpoints do not move at all (zero swaps), and a slowly moving body moves a
//    few slots, so the frame cost is O(N + K) instead of O(N log N). This is the
//    temporal coherence the SAP milestone claims; bench/bench_broadphase.cpp
//    measures it.
//
// Sweep:
//   walk the sorted endpoints; on MIN(x, id) test the proxy's full AABB against
//   every proxy currently in the active set (AABB::Overlaps is the y-axis second
//   filter: the x axis already overlaps by construction), then insert the proxy
//   id into the active set; on MAX remove it. The active set is a std::vector kept
//   sorted by proxy id: for 50-2000 proxies this beats a hash set, and it makes the
//   emitted pair order deterministic.
//
// Output ordering: the sweep visits pairs in x-overlap order, but the interface
// contract (and the World's determinism contract) is strictly ascending (a, b)
// with a < b. The pairs are therefore collected in a reusable scratch buffer and
// sorted before the callback fires; the sort is O(K log K) on the candidate count
// K, which is tiny compared to N once the y filter is applied.
//
// Removal: erasing the two endpoints from a sorted array keeps it sorted, so
// RemoveProxy only has to refresh the cached indices of the shifted tail. Swap-pop
// would be cheaper but would break the order invariant and force a re-sort;
// removals are rare (body destroy / shape detach), so the O(N) shift is the right
// trade.

#include "physics2d/Collision/BroadPhaseSAP.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>

namespace p2 {

namespace {

constexpr uint8_t kEndpointMin = 1;
constexpr uint8_t kEndpointMax = 0;

} // namespace

bool BroadPhaseSAP::EndpointLess(const Endpoint& a, const Endpoint& b) noexcept {
    if (a.value != b.value) {
        return a.value < b.value;
    }
    if (a.isMin != b.isMin) {
        // Inclusive overlap: the closing MAX of one interval and the opening MIN of
        // another at the same coordinate must keep the first interval in the active
        // set, otherwise a touching pair would be lost. MIN therefore sorts first.
        return a.isMin > b.isMin;
    }
    return a.proxyId < b.proxyId;
}

void BroadPhaseSAP::RefreshCachedIndex(int32_t endpointIndex) const {
    const Endpoint& endpoint = m_endpoints[static_cast<std::size_t>(endpointIndex)];
    Proxy& proxy = m_proxies[static_cast<std::size_t>(endpoint.proxyId)];
    if (endpoint.isMin != 0) {
        proxy.minIndex = endpointIndex;
    } else {
        proxy.maxIndex = endpointIndex;
    }
}

void BroadPhaseSAP::EnsureSorted() const {
    if (m_endpointsSorted) {
        return;
    }
    // Bulk path: one sort for every endpoint appended since the last observation.
    std::sort(m_endpoints.begin(), m_endpoints.end(), EndpointLess);
    const int32_t endpointCount = static_cast<int32_t>(m_endpoints.size());
    for (int32_t index = 0; index < endpointCount; ++index) {
        RefreshCachedIndex(index);
    }
    m_endpointsSorted = true;
}

void BroadPhaseSAP::SwapEndpoints(int32_t firstIndex, int32_t secondIndex) {
    std::swap(m_endpoints[static_cast<std::size_t>(firstIndex)],
              m_endpoints[static_cast<std::size_t>(secondIndex)]);
    RefreshCachedIndex(firstIndex);
    RefreshCachedIndex(secondIndex);
}

void BroadPhaseSAP::BubbleEndpoint(int32_t endpointIndex) {
    const int32_t endpointCount = static_cast<int32_t>(m_endpoints.size());
    int32_t position = endpointIndex;

    // The array was sorted before this endpoint's value changed, so its new slot is
    // a contiguous shift: bubble left while it is too small, otherwise bubble right
    // while the next endpoint is too small. One walk is enough (the two loops never
    // both move the same endpoint), but keeping both makes the helper total.
    while (position > 0 && EndpointLess(m_endpoints[static_cast<std::size_t>(position)],
                                        m_endpoints[static_cast<std::size_t>(position - 1)])) {
        SwapEndpoints(position, position - 1);
        --position;
    }
    while (position + 1 < endpointCount && EndpointLess(m_endpoints[static_cast<std::size_t>(position + 1)],
                                                        m_endpoints[static_cast<std::size_t>(position)])) {
        SwapEndpoints(position, position + 1);
        ++position;
    }
}

int32_t BroadPhaseSAP::AddProxy(const AABB& aabb) {
    assert(aabb.IsValid() && "BroadPhaseSAP::AddProxy: invalid AABB");

    int32_t proxyId = -1;
    if (!m_freeList.empty()) {
        // Reuse a released slot: ids stay bounded by the peak proxy count.
        proxyId = m_freeList.back();
        m_freeList.pop_back();
    } else {
        proxyId = static_cast<int32_t>(m_proxies.size());
        m_proxies.emplace_back();
    }

    Proxy& proxy = m_proxies[static_cast<std::size_t>(proxyId)];
    proxy.aabb = aabb;
    proxy.alive = true;
    proxy.minIndex = static_cast<int32_t>(m_endpoints.size());
    m_endpoints.push_back(Endpoint{aabb.lowerBound.x, proxyId, kEndpointMin});
    proxy.maxIndex = static_cast<int32_t>(m_endpoints.size());
    m_endpoints.push_back(Endpoint{aabb.upperBound.x, proxyId, kEndpointMax});
    // Defer ordering to the one bulk std::sort (see file comment).
    m_endpointsSorted = false;
    ++m_proxyCount;
    return proxyId;
}

void BroadPhaseSAP::RemoveProxy(int32_t proxyId) {
    assert(proxyId >= 0 && static_cast<std::size_t>(proxyId) < m_proxies.size() &&
           "BroadPhaseSAP::RemoveProxy: unknown proxy id");
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= m_proxies.size()) {
        return;
    }
    Proxy& proxy = m_proxies[static_cast<std::size_t>(proxyId)];
    if (!proxy.alive) {
        return; // idempotent
    }

    // The cached indices are only meaningful on a sorted array (the lazy bulk path
    // may still be pending); sort before trusting them.
    EnsureSorted();

    const int32_t minIndex = proxy.minIndex;
    const int32_t maxIndex = proxy.maxIndex;
    assert(minIndex >= 0 && maxIndex > minIndex && "BroadPhaseSAP::RemoveProxy: corrupt proxy indices");
    // Erase the larger index first so the smaller one is still valid. Removing
    // elements from a sorted array keeps it sorted; only the cached indices of the
    // shifted tail need refreshing.
    m_endpoints.erase(m_endpoints.begin() + maxIndex);
    m_endpoints.erase(m_endpoints.begin() + minIndex);
    const int32_t endpointCount = static_cast<int32_t>(m_endpoints.size());
    for (int32_t index = minIndex; index < endpointCount; ++index) {
        RefreshCachedIndex(index);
    }

    proxy.aabb = AABB{};
    proxy.minIndex = -1;
    proxy.maxIndex = -1;
    proxy.alive = false;
    m_freeList.push_back(proxyId);
    --m_proxyCount;
}

void BroadPhaseSAP::UpdateAABB(int32_t proxyId, const AABB& aabb) {
    assert(proxyId >= 0 && static_cast<std::size_t>(proxyId) < m_proxies.size() &&
           "BroadPhaseSAP::UpdateAABB: unknown proxy id");
    assert(aabb.IsValid() && "BroadPhaseSAP::UpdateAABB: invalid AABB");

    // A pending bulk insert must be ordered before the incremental bubbles can
    // interpret the cached indices as sorted positions.
    EnsureSorted();

    Proxy& proxy = m_proxies[static_cast<std::size_t>(proxyId)];
    assert(proxy.alive && "BroadPhaseSAP::UpdateAABB: proxy has been removed");
    proxy.aabb = aabb;

    // Move the MIN endpoint, then the MAX endpoint. Refreshing the cached indices
    // during every swap keeps proxy.maxIndex correct even if the MIN walk displaced
    // it.
    m_endpoints[static_cast<std::size_t>(proxy.minIndex)].value = aabb.lowerBound.x;
    BubbleEndpoint(proxy.minIndex);
    m_endpoints[static_cast<std::size_t>(proxy.maxIndex)].value = aabb.upperBound.x;
    BubbleEndpoint(proxy.maxIndex);
}

void BroadPhaseSAP::QueryPairs(const PairCallback& callback) {
    EnsureSorted();

    m_pairScratch.clear();
    m_active.clear();

    for (const Endpoint& endpoint : m_endpoints) {
        const int32_t proxyId = endpoint.proxyId;
        if (endpoint.isMin != 0) {
            const Proxy& proxy = m_proxies[static_cast<std::size_t>(proxyId)];
            // Active set = proxies whose x interval contains the sweep position.
            // The full AABB::Overlaps adds the y-axis filter; skipping it would
            // flood the narrow phase with false positives (P12 requirement).
            for (const int32_t activeId : m_active) {
                const Proxy& active = m_proxies[static_cast<std::size_t>(activeId)];
                if (Overlaps(proxy.aabb, active.aabb)) {
                    m_pairScratch.emplace_back(std::min(proxyId, activeId), std::max(proxyId, activeId));
                }
            }
            const auto position = std::lower_bound(m_active.begin(), m_active.end(), proxyId);
            m_active.insert(position, proxyId);
        } else {
            const auto position = std::lower_bound(m_active.begin(), m_active.end(), proxyId);
            // The MAX endpoint is always preceded by its own MIN endpoint, so the
            // proxy must be present; guard in release instead of erasing garbage.
            if (position != m_active.end() && *position == proxyId) {
                m_active.erase(position);
            } else {
                assert(false && "BroadPhaseSAP::QueryPairs: MAX endpoint without active MIN");
            }
        }
    }

    // Deterministic contract: ascending (a, b), a < b. The sweep emits pairs in
    // x-overlap order, so sort once on the (small) candidate set before firing.
    std::sort(m_pairScratch.begin(), m_pairScratch.end());
    for (const std::pair<int32_t, int32_t>& pair : m_pairScratch) {
        callback(pair.first, pair.second);
    }
}

bool BroadPhaseSAP::ValidateInternalState() const {
    // Restore the lazy bulk insert first: the contract is "valid at any time", not
    // "valid only after QueryPairs".
    EnsureSorted();

    const int32_t endpointCount = static_cast<int32_t>(m_endpoints.size());
    if (m_proxyCount < 0 || endpointCount != 2 * m_proxyCount) {
        return false;
    }

    int32_t aliveCount = 0;
    int32_t minCount = 0;
    int32_t maxCount = 0;
    for (int32_t index = 0; index < endpointCount; ++index) {
        const Endpoint& endpoint = m_endpoints[static_cast<std::size_t>(index)];
        if (endpoint.proxyId < 0 || static_cast<std::size_t>(endpoint.proxyId) >= m_proxies.size()) {
            return false;
        }
        const Proxy& proxy = m_proxies[static_cast<std::size_t>(endpoint.proxyId)];
        if (!proxy.alive) {
            return false;
        }
        if (endpoint.isMin != 0) {
            ++minCount;
            if (proxy.minIndex != index) {
                return false;
            }
        } else {
            ++maxCount;
            if (proxy.maxIndex != index) {
                return false;
            }
        }
        if (index > 0 && EndpointLess(endpoint, m_endpoints[static_cast<std::size_t>(index - 1)])) {
            return false; // array not sorted (previous > current)
        }
    }

    const int32_t proxySlotCount = static_cast<int32_t>(m_proxies.size());
    for (int32_t proxyId = 0; proxyId < proxySlotCount; ++proxyId) {
        const Proxy& proxy = m_proxies[static_cast<std::size_t>(proxyId)];
        if (!proxy.alive) {
            continue;
        }
        ++aliveCount;
        if (!proxy.aabb.IsValid()) {
            return false;
        }
        if (proxy.minIndex < 0 || proxy.maxIndex < 0 || proxy.minIndex >= endpointCount ||
            proxy.maxIndex >= endpointCount) {
            return false;
        }
        if (proxy.minIndex >= proxy.maxIndex) {
            return false;
        }
        // The cached indices must round-trip back to this proxy.
        if (m_endpoints[static_cast<std::size_t>(proxy.minIndex)].proxyId != proxyId ||
            m_endpoints[static_cast<std::size_t>(proxy.maxIndex)].proxyId != proxyId) {
            return false;
        }
    }

    return aliveCount == m_proxyCount && minCount == m_proxyCount && maxCount == m_proxyCount;
}

} // namespace p2
