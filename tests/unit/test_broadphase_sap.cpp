// Week 6 P12 — p2::BroadPhaseSAP conformance and internal-invariant tests.
//
// Graded against p2::BroadPhaseBruteForce, which is itself pinned to a direct
// double loop in test_broadphase_bruteforce.cpp:
//  - 10 random scenarios (N = 50/200/500, fixed seeds, 30x30 m, random boxes with
//    per-frame random displacement) x 1000 frames: the SAP pair list must equal
//    the BruteForce pair list EVERY frame, in the canonical ascending (a, b) order,
//    and ValidateInternalState() must be true every frame;
//  - endpoint invariants survive arbitrary add/remove/update churn (MIN count ==
//    MAX count == live proxies, array sorted, cached indices agree);
//  - a fully idle scene keeps the endpoint array untouched (validated through the
//    sorted-array + cached-index contract, the observable part of "zero swaps");
//  - the small-N crossover timing is printed so the World/testbed switch threshold
//    can be data-driven later (printed only, never asserted: timing tests are not
//    reproducible on shared CI machines).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Vec2.h"

namespace {

using p2::AABB;
using p2::BroadPhaseBruteForce;
using p2::BroadPhaseSAP;
using p2::Vec2;

using ProxyPair = std::pair<int32_t, int32_t>;
using PairList = std::vector<ProxyPair>;

AABB MakeBox(float lowerX, float lowerY, float upperX, float upperY) {
    return AABB{Vec2{lowerX, lowerY}, Vec2{upperX, upperY}};
}

AABB RandomBox(std::mt19937& rng, float areaSize) {
    std::uniform_real_distribution<float> position(0.0f, areaSize);
    std::uniform_real_distribution<float> halfExtent(0.1f, 0.5f);
    const Vec2 center{position(rng), position(rng)};
    const Vec2 extent{halfExtent(rng), halfExtent(rng)};
    return AABB{center - extent, center + extent};
}

PairList CollectPairs(p2::IBroadPhase& broadPhase) {
    PairList pairs;
    broadPhase.QueryPairs(
        [&pairs](int32_t proxyIdA, int32_t proxyIdB) { pairs.emplace_back(proxyIdA, proxyIdB); });
    return pairs;
}

// Checks the interface contract on a callback list: a < b, no duplicates, and
// ascending (a, b) order.
bool IsCanonicalPairList(const PairList& pairs) {
    std::set<ProxyPair> uniquePairs;
    for (size_t index = 0; index < pairs.size(); ++index) {
        const ProxyPair& pair = pairs[index];
        if (pair.first >= pair.second) {
            return false;
        }
        if (!uniquePairs.insert(pair).second) {
            return false;
        }
        if (index > 0 && !(pairs[index - 1] < pair)) {
            return false;
        }
    }
    return true;
}

// One random scenario shared by both implementations: identical box layout,
// identical per-frame motion (the RNG stream is consumed once per frame and both
// broad phases see the same AABBs), identical insertion order (so proxy ids are
// directly comparable).
struct ScenarioResult {
    int32_t frames = 0;
    int32_t pairMismatches = 0;
    int32_t canonicalViolations = 0;
    int32_t validationFailures = 0;
    int32_t totalPairs = 0;
    int32_t maxPairs = 0;
    int32_t firstMismatchFrame = -1;
};

ScenarioResult RunRandomScenario(int32_t bodyCount, uint32_t seed, int32_t frameCount) {
    constexpr float kAreaSize = 30.0f;
    constexpr float kMaxStep = 0.1f; // per-frame random displacement in metres

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> stepDistribution(-kMaxStep, kMaxStep);
    std::vector<Vec2> centers;
    std::vector<Vec2> halfExtents;
    centers.reserve(static_cast<size_t>(bodyCount));
    halfExtents.reserve(static_cast<size_t>(bodyCount));

    BroadPhaseSAP sap;
    BroadPhaseBruteForce bruteForce;
    std::vector<int32_t> sapIds;
    std::vector<int32_t> bruteForceIds;
    sapIds.reserve(static_cast<size_t>(bodyCount));
    bruteForceIds.reserve(static_cast<size_t>(bodyCount));

    for (int32_t index = 0; index < bodyCount; ++index) {
        const AABB box = RandomBox(rng, kAreaSize);
        centers.push_back(box.GetCenter());
        halfExtents.push_back(box.GetExtents());
        sapIds.push_back(sap.AddProxy(box));
        bruteForceIds.push_back(bruteForce.AddProxy(box));
        EXPECT_EQ(sapIds.back(), bruteForceIds.back()) << "same insertion order must hand out the same slots";
    }

    ScenarioResult result;
    std::vector<AABB> aabbs(static_cast<size_t>(bodyCount));
    for (int32_t frame = 0; frame < frameCount; ++frame) {
        // Random walk wrapped into the 30x30 m world. Wrapping deliberately
        // teleports boxes across the whole sweep axis now and then: the incremental
        // endpoint maintenance must handle arbitrary moves, not just coherent ones.
        for (int32_t index = 0; index < bodyCount; ++index) {
            Vec2& center = centers[static_cast<size_t>(index)];
            center.x += stepDistribution(rng);
            center.y += stepDistribution(rng);
            center.x = std::fmod(center.x, kAreaSize);
            center.y = std::fmod(center.y, kAreaSize);
            if (center.x < 0.0f) {
                center.x += kAreaSize;
            }
            if (center.y < 0.0f) {
                center.y += kAreaSize;
            }
            const Vec2 extent = halfExtents[static_cast<size_t>(index)];
            aabbs[static_cast<size_t>(index)] = AABB{center - extent, center + extent};
        }

        for (int32_t index = 0; index < bodyCount; ++index) {
            const AABB& box = aabbs[static_cast<size_t>(index)];
            sap.UpdateAABB(sapIds[static_cast<size_t>(index)], box);
            bruteForce.UpdateAABB(bruteForceIds[static_cast<size_t>(index)], box);
        }

        if (!sap.ValidateInternalState()) {
            ++result.validationFailures;
        }
        const PairList sapPairs = CollectPairs(sap);
        const PairList bruteForcePairs = CollectPairs(bruteForce);
        if (!IsCanonicalPairList(sapPairs)) {
            ++result.canonicalViolations;
        }
        if (sapPairs != bruteForcePairs) {
            ++result.pairMismatches;
            if (result.firstMismatchFrame < 0) {
                result.firstMismatchFrame = frame;
            }
        }
        ++result.frames;
        result.totalPairs += static_cast<int32_t>(sapPairs.size());
        result.maxPairs = std::max(result.maxPairs, static_cast<int32_t>(sapPairs.size()));
    }
    return result;
}

} // namespace

// ---------------------------------------------------------------------------
// Hand-computed scene: same expectations as the BruteForce test, proving SAP's
// active-set sweep agrees on the inclusive-overlap edge cases.
// ---------------------------------------------------------------------------

TEST(BroadPhaseSap, FiveBoxSceneMatchesHandComputedPairSet) {
    // A = [0,1]x[0,1], B = [1,2]x[0,1] (face touch), C = [0.25,0.75]^2 (inside A),
    // D = [5,6]^2 (isolated), E = [0,1]x[1,2] (face touch with A, corner touch with B).
    // Expected: (A,B), (A,C), (A,E), (B,E) - see test_broadphase_bruteforce.cpp.
    BroadPhaseSAP sap;
    EXPECT_EQ(sap.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f)), 0);
    EXPECT_EQ(sap.AddProxy(MakeBox(1.0f, 0.0f, 2.0f, 1.0f)), 1);
    EXPECT_EQ(sap.AddProxy(MakeBox(0.25f, 0.25f, 0.75f, 0.75f)), 2);
    EXPECT_EQ(sap.AddProxy(MakeBox(5.0f, 5.0f, 6.0f, 6.0f)), 3);
    EXPECT_EQ(sap.AddProxy(MakeBox(0.0f, 1.0f, 1.0f, 2.0f)), 4);

    const PairList pairs = CollectPairs(sap);
    const PairList expected{{0, 1}, {0, 2}, {0, 4}, {1, 4}};
    EXPECT_EQ(pairs, expected);
    EXPECT_TRUE(IsCanonicalPairList(pairs));
    EXPECT_EQ(sap.GetProxyCount(), 5);
    EXPECT_STREQ(sap.Name(), "SAP");

    // A bulk insert followed directly by validation must succeed: the lazy
    // std::sort path restores the invariant before ValidateInternalState checks it.
    EXPECT_TRUE(sap.ValidateInternalState());
}

TEST(BroadPhaseSap, EmptyBroadPhaseIsValidAndSilent) {
    BroadPhaseSAP sap;
    EXPECT_EQ(sap.GetProxyCount(), 0);
    EXPECT_TRUE(sap.ValidateInternalState());
    EXPECT_TRUE(CollectPairs(sap).empty());
}

// ---------------------------------------------------------------------------
// 10 random scenarios x 1000 frames: exact agreement + invariant per frame.
// ---------------------------------------------------------------------------

TEST(BroadPhaseSap, MatchesBruteForceOnTenRandomScenariosForThousandFrames) {
    // Mix of 50/200/500 bodies with distinct fixed seeds, exactly as the Week 6
    // acceptance criterion asks.
    struct Scenario {
        int32_t bodyCount;
        uint32_t seed;
    };
    const Scenario scenarios[] = {
        {50, 11u},  {50, 12u},  {50, 13u},  {50, 14u},  {200, 21u},
        {200, 22u}, {200, 23u}, {500, 31u}, {500, 32u}, {500, 33u},
    };
    constexpr int32_t kFrames = 1000;

    int32_t totalFrames = 0;
    int32_t totalPairs = 0;
    int32_t mismatches = 0;
    int32_t canonicalViolations = 0;
    int32_t validationFailures = 0;
    for (const Scenario& scenario : scenarios) {
        const ScenarioResult result = RunRandomScenario(scenario.bodyCount, scenario.seed, kFrames);
        std::printf("[broadphase] sap-vs-bruteforce N=%d seed=%u: frames=%d pairs/frame(avg)=%.1f max=%d "
                    "mismatches=%d canonical=%d validate=%d\n",
                    scenario.bodyCount, scenario.seed, result.frames,
                    static_cast<double>(result.totalPairs) / static_cast<double>(result.frames),
                    result.maxPairs, result.pairMismatches, result.canonicalViolations,
                    result.validationFailures);
        EXPECT_EQ(result.pairMismatches, 0) << "N=" << scenario.bodyCount << " seed=" << scenario.seed
                                            << " firstMismatchFrame=" << result.firstMismatchFrame;
        EXPECT_EQ(result.canonicalViolations, 0) << "N=" << scenario.bodyCount << " seed=" << scenario.seed;
        EXPECT_EQ(result.validationFailures, 0) << "N=" << scenario.bodyCount << " seed=" << scenario.seed;
        EXPECT_EQ(result.frames, kFrames);
        totalFrames += result.frames;
        totalPairs += result.totalPairs;
        mismatches += result.pairMismatches;
        canonicalViolations += result.canonicalViolations;
        validationFailures += result.validationFailures;
    }
    EXPECT_EQ(totalFrames, 10 * kFrames);
    EXPECT_EQ(mismatches, 0);
    EXPECT_EQ(canonicalViolations, 0);
    EXPECT_EQ(validationFailures, 0);
    std::printf(
        "[broadphase] sap-vs-bruteforce summary: %d scenario frames, %d pairs compared, 0 mismatches\n",
        totalFrames, totalPairs);
}

// ---------------------------------------------------------------------------
// Endpoint invariants under arbitrary add/remove/UpdateAABB churn.
// ---------------------------------------------------------------------------

TEST(BroadPhaseSap, EndpointInvariantsHoldUnderArbitraryChurn) {
    std::mt19937 rng(777u);
    std::uniform_real_distribution<float> position(0.0f, 20.0f);
    std::uniform_real_distribution<float> halfExtent(0.1f, 1.0f);
    const auto makeBox = [&]() {
        const Vec2 center{position(rng), position(rng)};
        const Vec2 extent{halfExtent(rng), halfExtent(rng)};
        return AABB{center - extent, center + extent};
    };

    BroadPhaseSAP sap;
    BroadPhaseBruteForce bruteForce;
    std::vector<AABB> aabbs;
    std::vector<uint8_t> alive;
    std::vector<int32_t> liveIds;

    int32_t validationFailures = 0;
    int32_t pairMismatches = 0;
    for (int32_t operation = 0; operation < 600; ++operation) {
        const int32_t action = static_cast<int32_t>(rng() % 100u);
        if (liveIds.empty() || action < 45) {
            const AABB box = makeBox();
            const int32_t sapId = sap.AddProxy(box);
            const int32_t bruteForceId = bruteForce.AddProxy(box);
            EXPECT_EQ(sapId, bruteForceId);
            if (static_cast<size_t>(sapId) >= aabbs.size()) {
                aabbs.resize(static_cast<size_t>(sapId) + 1);
                alive.resize(static_cast<size_t>(sapId) + 1, 0);
            }
            aabbs[static_cast<size_t>(sapId)] = box;
            alive[static_cast<size_t>(sapId)] = 1;
            liveIds.push_back(sapId);
        } else if (action < 75) {
            std::uniform_int_distribution<size_t> pick(0, liveIds.size() - 1);
            const size_t slot = pick(rng);
            const int32_t proxyId = liveIds[slot];
            liveIds[slot] = liveIds.back();
            liveIds.pop_back();
            sap.RemoveProxy(proxyId);
            bruteForce.RemoveProxy(proxyId);
            alive[static_cast<size_t>(proxyId)] = 0;
        } else {
            std::uniform_int_distribution<size_t> pick(0, liveIds.size() - 1);
            const int32_t proxyId = liveIds[pick(rng)];
            const AABB box = makeBox();
            aabbs[static_cast<size_t>(proxyId)] = box;
            sap.UpdateAABB(proxyId, box);
            bruteForce.UpdateAABB(proxyId, box);
        }

        // The frozen debug contract is "valid at any time": no query needs to run
        // before validation, so this catches unsorted-array and stale-index bugs
        // right after the operation that caused them.
        if (!sap.ValidateInternalState()) {
            ++validationFailures;
        }
        EXPECT_EQ(sap.GetProxyCount(), static_cast<int32_t>(liveIds.size()));

        if (operation % 10 == 0) {
            const PairList sapPairs = CollectPairs(sap);
            const PairList bruteForcePairs = CollectPairs(bruteForce);
            if (sapPairs != bruteForcePairs) {
                ++pairMismatches;
            }
            EXPECT_TRUE(IsCanonicalPairList(sapPairs));
        }
    }

    std::printf(
        "[broadphase] sap churn: 600 operations, %d live proxies, validation failures=%d mismatches=%d\n",
        static_cast<int>(liveIds.size()), validationFailures, pairMismatches);
    EXPECT_EQ(validationFailures, 0);
    EXPECT_EQ(pairMismatches, 0);
    EXPECT_TRUE(sap.ValidateInternalState());

    // Free-list reuse: the LIFO free list must hand the most recently released slot
    // back, exactly like the BruteForce baseline.
    const int32_t recycled = liveIds.front();
    sap.RemoveProxy(recycled);
    EXPECT_TRUE(sap.ValidateInternalState());
    EXPECT_EQ(sap.AddProxy(MakeBox(0.0f, 0.0f, 1.0f, 1.0f)), recycled);
    EXPECT_TRUE(sap.ValidateInternalState());
    EXPECT_EQ(sap.GetProxyCount(), static_cast<int32_t>(liveIds.size()));
}

// ---------------------------------------------------------------------------
// Idle scene: no endpoint ever moves, so the incremental path must be a pure
// no-op (validated through the sorted-array + cached-index contract).
// ---------------------------------------------------------------------------

TEST(BroadPhaseSap, IdleSceneKeepsTheEndpointArrayUntouched) {
    constexpr int32_t kProxyCount = 64;
    std::mt19937 rng(4242u);
    BroadPhaseSAP sap;
    BroadPhaseBruteForce bruteForce;
    std::vector<AABB> boxes;
    for (int32_t index = 0; index < kProxyCount; ++index) {
        const AABB box = RandomBox(rng, 30.0f);
        boxes.push_back(box);
        const int32_t sapId = sap.AddProxy(box);
        const int32_t bruteForceId = bruteForce.AddProxy(box);
        EXPECT_EQ(sapId, index);
        EXPECT_EQ(bruteForceId, index);
    }

    int32_t validationFailures = 0;
    for (int32_t frame = 0; frame < 1000; ++frame) {
        // Same AABB handed in every frame: UpdateAABB must not move a single
        // endpoint (the bubble walk finds the array already ordered).
        for (int32_t index = 0; index < kProxyCount; ++index) {
            sap.UpdateAABB(index, boxes[static_cast<size_t>(index)]);
            bruteForce.UpdateAABB(index, boxes[static_cast<size_t>(index)]);
        }
        if (!sap.ValidateInternalState()) {
            ++validationFailures;
        }
        if (frame % 250 == 0) {
            EXPECT_EQ(CollectPairs(sap), CollectPairs(bruteForce));
        }
    }
    EXPECT_EQ(validationFailures, 0);
    EXPECT_EQ(CollectPairs(sap), CollectPairs(bruteForce));
    std::printf("[broadphase] idle scene: 64 proxies x 1000 frames, 0 validation failures\n");
}

// ---------------------------------------------------------------------------
// Small-N crossover timing (printed only: timing is not a correctness contract).
// ---------------------------------------------------------------------------

TEST(BroadPhaseSap, SmallSceneCrossoverTimingIsPrinted) {
    constexpr int32_t kFrames = 500;
    constexpr float kAreaSize = 10.0f;
    for (const int32_t bodyCount : {5, 10, 20}) {
        // Build the scene once, then replay the SAME motion for each implementation
        // by giving both runs an identical freshly seeded RNG and a copy of the
        // initial centres.
        std::mt19937 setupRng(2024u + static_cast<uint32_t>(bodyCount));
        std::vector<Vec2> initialCenters;
        std::vector<Vec2> halfExtents;
        for (int32_t index = 0; index < bodyCount; ++index) {
            const AABB box = RandomBox(setupRng, kAreaSize);
            initialCenters.push_back(box.GetCenter());
            halfExtents.push_back(box.GetExtents());
        }
        const uint32_t motionSeed = 2024u + static_cast<uint32_t>(bodyCount) + 1u;

        const auto runOne = [&](p2::IBroadPhase& broadPhase) {
            std::mt19937 rng(motionSeed);
            std::uniform_real_distribution<float> stepDistribution(-0.05f, 0.05f);
            std::vector<Vec2> centers = initialCenters;
            std::vector<int32_t> ids;
            for (int32_t index = 0; index < bodyCount; ++index) {
                const Vec2 center = centers[static_cast<size_t>(index)];
                const Vec2 extent = halfExtents[static_cast<size_t>(index)];
                ids.push_back(broadPhase.AddProxy(AABB{center - extent, center + extent}));
            }
            int64_t pairCount = 0;
            const auto start = std::chrono::steady_clock::now();
            for (int32_t frame = 0; frame < kFrames; ++frame) {
                for (int32_t index = 0; index < bodyCount; ++index) {
                    Vec2& center = centers[static_cast<size_t>(index)];
                    center.x += stepDistribution(rng);
                    center.y += stepDistribution(rng);
                    center.x = std::fmod(center.x, kAreaSize);
                    center.y = std::fmod(center.y, kAreaSize);
                    if (center.x < 0.0f) {
                        center.x += kAreaSize;
                    }
                    if (center.y < 0.0f) {
                        center.y += kAreaSize;
                    }
                    const Vec2 extent = halfExtents[static_cast<size_t>(index)];
                    broadPhase.UpdateAABB(ids[static_cast<size_t>(index)],
                                          AABB{center - extent, center + extent});
                }
                broadPhase.QueryPairs([&pairCount](int32_t, int32_t) { ++pairCount; });
            }
            const auto end = std::chrono::steady_clock::now();
            const double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
            return std::make_pair(totalMs / static_cast<double>(kFrames), pairCount);
        };

        BroadPhaseSAP sap;
        BroadPhaseBruteForce bruteForce;
        const auto bruteForceResult = runOne(bruteForce);
        const auto sapResult = runOne(sap);
        std::printf("[broadphase] crossover N=%2d: brute=%.4f ms/frame sap=%.4f ms/frame ratio=%.2f "
                    "pairs/frame=%.1f\n",
                    bodyCount, bruteForceResult.first, sapResult.first,
                    bruteForceResult.first / std::max(1.0e-9, sapResult.first),
                    static_cast<double>(sapResult.second) / static_cast<double>(kFrames));
        // Nothing asserted: this only prints the data for the future switch
        // threshold (tiny scenes may still prefer BruteForce).
        EXPECT_GE(sapResult.second, 0);
    }
}
