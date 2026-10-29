// Week 6 P11 — p2::BroadPhaseBruteForce contract tests.
//
// BruteForce is the correctness oracle for the rest of Week 6, so its contract is
// pinned before SAP is compared against it:
//  - proxy ids are slot indices reused through a free list; GetProxyCount is the
//    number of live proxies;
//  - QueryPairs emits every overlapping pair exactly once with a < b, in ascending
//    (a, b) order (the World's determinism depends on that order);
//  - touching boxes overlap (inclusive AABB convention) and UpdateAABB changes the
//    reported set immediately;
//  - the pair set equals a direct double loop over the live AABBs - the "reference
//    implementation" every optimisation is graded against.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Vec2.h"

namespace {

using p2::AABB;
using p2::BroadPhaseBruteForce;
using p2::Vec2;

using ProxyPair = std::pair<int32_t, int32_t>;
using PairList = std::vector<ProxyPair>;

AABB MakeBox(float lowerX, float lowerY, float upperX, float upperY) {
    return AABB{Vec2{lowerX, lowerY}, Vec2{upperX, upperY}};
}

PairList CollectPairs(p2::IBroadPhase& broadPhase) {
    PairList pairs;
    broadPhase.QueryPairs(
        [&pairs](int32_t proxyIdA, int32_t proxyIdB) { pairs.emplace_back(proxyIdA, proxyIdB); });
    return pairs;
}

// Reference implementation: the naive double loop the broad phase replaces. `alive`
// is indexed by proxy id; ids are discovered by insertion order (AddProxy returns
// the next slot or a recycled one).
PairList ReferencePairs(const std::vector<AABB>& aabbs, const std::vector<uint8_t>& alive) {
    PairList pairs;
    const int32_t count = static_cast<int32_t>(aabbs.size());
    for (int32_t proxyIdA = 0; proxyIdA < count; ++proxyIdA) {
        if (alive[static_cast<std::size_t>(proxyIdA)] == 0) {
            continue;
        }
        for (int32_t proxyIdB = proxyIdA + 1; proxyIdB < count; ++proxyIdB) {
            if (alive[static_cast<std::size_t>(proxyIdB)] == 0) {
                continue;
            }
            if (Overlaps(aabbs[static_cast<std::size_t>(proxyIdA)],
                         aabbs[static_cast<std::size_t>(proxyIdB)])) {
                pairs.emplace_back(proxyIdA, proxyIdB);
            }
        }
    }
    return pairs;
}

void ExpectCanonicalPairs(const PairList& pairs) {
    std::set<ProxyPair> uniquePairs;
    for (const ProxyPair& pair : pairs) {
        EXPECT_LT(pair.first, pair.second) << "callback must hand out a < b";
        EXPECT_TRUE(uniquePairs.insert(pair).second)
            << "duplicate pair (" << pair.first << ", " << pair.second << ")";
    }
    // The implementation iterates i < j in slot order, so the emission order is
    // already the canonical ascending (a, b) order.
    PairList sorted = pairs;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(pairs, sorted) << "QueryPairs must emit ascending (a, b) order";
}

AABB RandomBox(std::mt19937& rng, float areaSize, float minHalfExtent, float maxHalfExtent) {
    std::uniform_real_distribution<float> position(0.0f, areaSize);
    std::uniform_real_distribution<float> halfExtent(minHalfExtent, maxHalfExtent);
    const Vec2 center{position(rng), position(rng)};
    const Vec2 extent{halfExtent(rng), halfExtent(rng)};
    return AABB{center - extent, center + extent};
}

} // namespace

// ---------------------------------------------------------------------------
// Five hand-computed AABB cases, one per overlap class.
// ---------------------------------------------------------------------------

TEST(BroadPhaseBruteForce, HandCraftedOverlapClasses) {
    struct Case {
        const char* label;
        AABB first;
        AABB second;
        bool expectedOverlap;
    };
    const Case cases[] = {
        // Touching on a face: A.max.x == B.min.x. The inclusive convention says
        // touching is a real contact (the narrow phase makes the final call).
        {"touching-face", MakeBox(0.0f, 0.0f, 1.0f, 1.0f), MakeBox(1.0f, 0.0f, 2.0f, 1.0f), true},
        // Containment: B is entirely inside A.
        {"containment", MakeBox(0.0f, 0.0f, 2.0f, 2.0f), MakeBox(0.5f, 0.5f, 1.5f, 1.5f), true},
        // Completely separated on both axes.
        {"separated", MakeBox(0.0f, 0.0f, 1.0f, 1.0f), MakeBox(2.0f, 2.0f, 3.0f, 3.0f), false},
        // Partial overlap on both axes.
        {"partial", MakeBox(0.0f, 0.0f, 1.0f, 1.0f), MakeBox(0.5f, 0.5f, 1.5f, 1.5f), true},
        // Identical boxes: a legal degenerate state (two bodies at the same place).
        {"identical", MakeBox(0.0f, 0.0f, 1.0f, 1.0f), MakeBox(0.0f, 0.0f, 1.0f, 1.0f), true},
        // Corner touch only (equal x and y at one point): still inclusive overlap.
        {"corner-touch", MakeBox(0.0f, 0.0f, 1.0f, 1.0f), MakeBox(1.0f, 1.0f, 2.0f, 2.0f), true},
    };

    for (const Case& testCase : cases) {
        BroadPhaseBruteForce broadPhase;
        EXPECT_EQ(broadPhase.AddProxy(testCase.first), 0) << testCase.label;
        EXPECT_EQ(broadPhase.AddProxy(testCase.second), 1) << testCase.label;
        const PairList pairs = CollectPairs(broadPhase);
        if (testCase.expectedOverlap) {
            ASSERT_EQ(pairs.size(), 1u) << testCase.label;
            EXPECT_EQ(pairs[0], ProxyPair(0, 1)) << testCase.label;
        } else {
            EXPECT_TRUE(pairs.empty()) << testCase.label;
        }
    }
}

TEST(BroadPhaseBruteForce, FiveBoxSceneMatchesHandComputedPairSet) {
    // A = [0,1]x[0,1], B = [1,2]x[0,1], C = [0.25,0.75]x[0.25,0.75],
    // D = [5,6]x[5,6] (isolated), E = [0,1]x[1,2].
    // Expected pairs:
    //   (A,B) touching at x = 1, y ranges overlap;
    //   (A,C) C is inside A;
    //   (A,E) touching at y = 1, x ranges overlap;
    //   (B,E) corner touch at (1, 1) - inclusive overlap;
    //   (B,C) no x overlap; (C,E) no y overlap; D touches nothing.
    BroadPhaseBruteForce broadPhase;
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f)), 0);
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(1.0f, 0.0f, 2.0f, 1.0f)), 1);
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(0.25f, 0.25f, 0.75f, 0.75f)), 2);
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(5.0f, 5.0f, 6.0f, 6.0f)), 3);
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(0.0f, 1.0f, 1.0f, 2.0f)), 4);

    const PairList pairs = CollectPairs(broadPhase);
    const PairList expected{{0, 1}, {0, 2}, {0, 4}, {1, 4}};
    EXPECT_EQ(pairs, expected);
    ExpectCanonicalPairs(pairs);
    EXPECT_EQ(broadPhase.GetProxyCount(), 5);
    EXPECT_STREQ(broadPhase.Name(), "BruteForce");
}

// ---------------------------------------------------------------------------
// 100 random AABBs: the pair set must equal the direct double loop exactly.
// ---------------------------------------------------------------------------

TEST(BroadPhaseBruteForce, HundredRandomAabbsMatchTheReferenceLoop) {
    std::mt19937 rng(1234u);
    constexpr int32_t kProxyCount = 100;
    std::vector<AABB> aabbs;
    aabbs.reserve(kProxyCount);

    BroadPhaseBruteForce broadPhase;
    for (int32_t index = 0; index < kProxyCount; ++index) {
        const AABB aabb = RandomBox(rng, 10.0f, 0.1f, 1.0f);
        aabbs.push_back(aabb);
        EXPECT_EQ(broadPhase.AddProxy(aabb), index);
    }

    const PairList pairs = CollectPairs(broadPhase);
    std::vector<uint8_t> alive(static_cast<std::size_t>(kProxyCount), 1);
    const PairList reference = ReferencePairs(aabbs, alive);
    EXPECT_EQ(pairs, reference);
    ExpectCanonicalPairs(pairs);
    std::printf("[broadphase] brute-force 100 random AABBs: %d pairs, %d reference pairs\n",
                static_cast<int>(pairs.size()), static_cast<int>(reference.size()));
}

// ---------------------------------------------------------------------------
// Add/remove churn: ids are reused, counts stay right, removed proxies vanish.
// ---------------------------------------------------------------------------

TEST(BroadPhaseBruteForce, AddRemoveChurnKeepsIdsAndPairsConsistent) {
    std::mt19937 rng(99u);
    BroadPhaseBruteForce broadPhase;
    std::vector<AABB> aabbs;
    std::vector<uint8_t> alive;
    std::vector<int32_t> liveIds;

    int32_t addCount = 0;
    int32_t removeCount = 0;
    for (int32_t operation = 0; operation < 500; ++operation) {
        const bool doAdd = liveIds.empty() || (operation % 3 != 0) || liveIds.size() <= 20u;
        if (doAdd) {
            const AABB aabb = RandomBox(rng, 10.0f, 0.1f, 1.0f);
            const int32_t proxyId = broadPhase.AddProxy(aabb);
            ASSERT_GE(proxyId, 0);
            if (static_cast<std::size_t>(proxyId) >= aabbs.size()) {
                aabbs.resize(static_cast<std::size_t>(proxyId) + 1);
                alive.resize(static_cast<std::size_t>(proxyId) + 1, 0);
            }
            aabbs[static_cast<std::size_t>(proxyId)] = aabb;
            alive[static_cast<std::size_t>(proxyId)] = 1;
            liveIds.push_back(proxyId);
            ++addCount;
        } else {
            std::uniform_int_distribution<size_t> pick(0, liveIds.size() - 1);
            const size_t position = pick(rng);
            const int32_t proxyId = liveIds[position];
            liveIds[position] = liveIds.back();
            liveIds.pop_back();
            broadPhase.RemoveProxy(proxyId);
            alive[static_cast<std::size_t>(proxyId)] = 0;
            ++removeCount;
        }

        EXPECT_EQ(broadPhase.GetProxyCount(), static_cast<int32_t>(liveIds.size()));
        const PairList pairs = CollectPairs(broadPhase);
        EXPECT_EQ(pairs, ReferencePairs(aabbs, alive)) << "after operation " << operation;
        ExpectCanonicalPairs(pairs);
        for (const ProxyPair& pair : pairs) {
            EXPECT_EQ(alive[static_cast<std::size_t>(pair.first)], 1) << "removed proxy reported";
            EXPECT_EQ(alive[static_cast<std::size_t>(pair.second)], 1) << "removed proxy reported";
        }
    }

    std::printf("[broadphase] brute-force churn: %d adds, %d removes, %d live proxies\n", addCount,
                removeCount, static_cast<int>(liveIds.size()));

    // Free-list reuse: releasing the most recently allocated id must hand the same
    // slot back on the next AddProxy (LIFO free list), so ids stay bounded.
    const int32_t recycled = liveIds.front();
    broadPhase.RemoveProxy(recycled);
    const int32_t reused = broadPhase.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f));
    EXPECT_EQ(reused, recycled);
    EXPECT_EQ(broadPhase.GetProxyCount(), static_cast<int32_t>(liveIds.size()));
}

// ---------------------------------------------------------------------------
// UpdateAABB must be visible to the very next QueryPairs (no stale AABBs).
// ---------------------------------------------------------------------------

TEST(BroadPhaseBruteForce, UpdateAabbChangesThePairSetImmediately) {
    BroadPhaseBruteForce broadPhase;
    const int32_t first = broadPhase.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f));
    const int32_t second = broadPhase.AddProxy(MakeBox(3.0f, 0.0f, 4.0f, 1.0f));
    EXPECT_TRUE(CollectPairs(broadPhase).empty());

    broadPhase.UpdateAABB(second, MakeBox(0.5f, 0.5f, 1.5f, 1.5f));
    ASSERT_EQ(CollectPairs(broadPhase).size(), 1u);
    EXPECT_EQ(CollectPairs(broadPhase)[0], ProxyPair(first, second));

    broadPhase.UpdateAABB(second, MakeBox(9.0f, 9.0f, 10.0f, 10.0f));
    EXPECT_TRUE(CollectPairs(broadPhase).empty());

    // Removing the first proxy leaves only the second: still no pair, and the
    // removed id never shows up again.
    broadPhase.RemoveProxy(first);
    EXPECT_EQ(broadPhase.GetProxyCount(), 1);
    EXPECT_TRUE(CollectPairs(broadPhase).empty());
    EXPECT_EQ(broadPhase.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f)), first);
}
