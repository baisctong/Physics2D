#pragma once

// p2::BroadPhaseInterface — pluggable broad phase (Section 4.5).
//
// RESERVED INTERFACE (Week 6): header only, no implementation in Week 1.
// BruteForce (Week 6 D1/D2) and Sweep-and-Prune (Week 6 D3-D5) will implement
// this interface; World already owns a `std::unique_ptr<IBroadPhase>` and the
// testbed will gain a dropdown to switch implementations at runtime.
//
// Contract:
//  - proxies are integer ids owned by the implementation;
//  - World maps body handle index -> proxy id (it keeps the storage);
//  - QueryPairs may emit (A,B) in either order; implementations of the
//    callback must tolerate that. The concrete implementations are expected to
//    normalise to A < B, but the interface does not guarantee it, so callers
//    still de-duplicate defensively (Section 6.2 "配对去重").

#include "physics2d/Math/AABB.h"

#include <cstdint>
#include <functional>

namespace p2 {

// Called once per overlapping proxy pair. Arguments are proxy ids, not body
// handles: the broad phase knows nothing about bodies.
using PairCallback = std::function<void(int32_t proxyIdA, int32_t proxyIdB)>;

class IBroadPhase {
  public:
    IBroadPhase() = default;
    virtual ~IBroadPhase() = default;

    IBroadPhase(const IBroadPhase&) = delete;
    IBroadPhase& operator=(const IBroadPhase&) = delete;

    // Inserts a fat AABB and returns its proxy id (>= 0).
    [[nodiscard]] virtual int32_t AddProxy(const AABB& aabb) = 0;

    virtual void RemoveProxy(int32_t proxyId) = 0;

    // Moves an existing proxy. The implementation may keep a fat AABB and skip
    // internal updates until the real AABB escapes it (Section 6.2).
    virtual void UpdateAABB(int32_t proxyId, const AABB& aabb) = 0;

    // Enumerates every currently overlapping candidate pair.
    virtual void QueryPairs(const PairCallback& callback) = 0;

    [[nodiscard]] virtual int32_t GetProxyCount() const = 0;

    // Used by the testbed dropdown and by bench output.
    [[nodiscard]] virtual const char* Name() const = 0;
};

} // namespace p2
