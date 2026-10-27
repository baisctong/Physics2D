// Week 1 D6 (P05): p2::IDPool lifecycle and generation-counter unit tests.
//
// Contract under test (see IDPool.h): O(1) LIFO slot reuse, generation bumped
// exactly once per release, so a generation-checked handle can never alias the
// slot's next occupant (no ABA). Every expected value below is hand-computed
// from that contract, not captured from a previous run.

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Core/Handle.h"
#include "physics2d/Core/IDPool.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using p2::BodyHandle;
using p2::IDPool;

// A World-style validity check: index must be alive AND the generation must
// match, exactly what World::IsValid does with the pool.
[[nodiscard]] bool HandleResolves(const IDPool& pool, const BodyHandle& handle) {
    return handle.index >= 0 && pool.IsValid(handle.index) &&
           pool.GetGeneration(handle.index) == handle.generation;
}

TEST(IDPoolTest, NewPoolIsEmpty) {
    IDPool pool;
    EXPECT_EQ(pool.GetAliveCount(), 0);
    EXPECT_EQ(pool.GetCapacity(), 0);
    EXPECT_FALSE(pool.IsValidIndex(-1));
    EXPECT_FALSE(pool.IsValidIndex(0));
    EXPECT_FALSE(pool.IsValid(-1));
    EXPECT_FALSE(pool.IsValid(0));
}

TEST(IDPoolTest, AllocateReturnsContiguousSlots) {
    IDPool pool;
    // Without a release the pool grows by one slot per call, so a call sequence
    // of 0, 1, 2 is both the index sequence and the capacity evidence.
    EXPECT_EQ(pool.Allocate(), 0);
    EXPECT_EQ(pool.Allocate(), 1);
    EXPECT_EQ(pool.Allocate(), 2);
    EXPECT_EQ(pool.GetAliveCount(), 3);
    EXPECT_EQ(pool.GetCapacity(), 3);
    EXPECT_TRUE(pool.IsValid(0));
    EXPECT_TRUE(pool.IsValid(2));
    // A freshly created slot starts at generation 1 (IDPool::Entry default).
    EXPECT_EQ(pool.GetGeneration(0), 1U);
    EXPECT_EQ(pool.GetGeneration(1), 1U);
    EXPECT_EQ(pool.GetGeneration(2), 1U);
}

TEST(IDPoolTest, ReleaseBumpsGenerationAndRejectsStaleHandle) {
    IDPool pool;
    const int32_t slot = pool.Allocate(); // slot 0, generation 1
    ASSERT_EQ(slot, 0);
    const BodyHandle firstGeneration{slot, pool.GetGeneration(slot)};
    ASSERT_EQ(firstGeneration.generation, 1U);
    ASSERT_TRUE(HandleResolves(pool, firstGeneration));

    pool.Release(slot);
    EXPECT_FALSE(pool.IsValid(slot));
    EXPECT_EQ(pool.GetAliveCount(), 0);
    // Bumped exactly once by the release: the handle above now fails IsValid.
    EXPECT_EQ(pool.GetGeneration(slot), 2U);
    EXPECT_FALSE(HandleResolves(pool, firstGeneration));

    // Reusing the slot yields a different generation, so the first handle cannot
    // resolve to the new occupant - this is the ABA fix in its smallest form.
    EXPECT_EQ(pool.Allocate(), slot);
    const BodyHandle secondGeneration{slot, pool.GetGeneration(slot)};
    EXPECT_EQ(secondGeneration.generation, 2U);
    EXPECT_EQ(pool.GetAliveCount(), 1);
    EXPECT_NE(firstGeneration, secondGeneration);
    EXPECT_TRUE(HandleResolves(pool, secondGeneration));
    EXPECT_FALSE(HandleResolves(pool, firstGeneration));
}

TEST(IDPoolTest, FreeListIsLifo) {
    IDPool pool;
    EXPECT_EQ(pool.Allocate(), 0);
    EXPECT_EQ(pool.Allocate(), 1);
    EXPECT_EQ(pool.Allocate(), 2);

    pool.Release(1);
    pool.Release(0);
    // LIFO: slot 0 was released last and is therefore handed back first, then
    // slot 1. After the free list is empty the next call appends slot 3.
    EXPECT_EQ(pool.Allocate(), 0);
    EXPECT_EQ(pool.Allocate(), 1);
    EXPECT_EQ(pool.Allocate(), 3);
    // Slots 0, 1, 2 and 3 are all alive now: 2 was never released, 0 and 1 were
    // handed back, and 3 is freshly appended (free list exhausted).
    EXPECT_EQ(pool.GetAliveCount(), 4);
    EXPECT_EQ(pool.GetCapacity(), 4);
}

TEST(IDPoolTest, AllocateReleaseReuse1000Times) {
    // Design doc 7.2 / Week 1 DoD: 1000 create/destroy/reuse cycles must keep
    // generations correct and must never revive a stale handle.
    constexpr int32_t kSlotCount = 1000;
    IDPool pool;

    // Round 1: 1000 fresh allocations get indices 0..999 at generation 1.
    std::vector<BodyHandle> firstRound;
    firstRound.reserve(static_cast<std::size_t>(kSlotCount));
    for (int32_t i = 0; i < kSlotCount; ++i) {
        const int32_t slot = pool.Allocate();
        EXPECT_EQ(slot, i); // no release yet, so indices are strictly increasing
        const BodyHandle handle{slot, pool.GetGeneration(slot)};
        EXPECT_EQ(handle.generation, 1U);
        firstRound.push_back(handle);
    }
    EXPECT_EQ(pool.GetAliveCount(), kSlotCount);
    EXPECT_EQ(pool.GetCapacity(), kSlotCount);

    // Release all 1000 slots: each generation is bumped exactly once (1 -> 2).
    for (int32_t i = 0; i < kSlotCount; ++i) {
        pool.Release(i);
    }
    EXPECT_EQ(pool.GetAliveCount(), 0);
    for (int32_t i = 0; i < kSlotCount; ++i) {
        EXPECT_FALSE(pool.IsValid(i));
        EXPECT_EQ(pool.GetGeneration(i), 2U);
    }

    // Round 2: every slot is reused once. LIFO means slot 999 comes back first,
    // 998 second, ..., slot 0 last. Each new handle is generation 2, so the
    // corresponding round-1 handle stays stale (same index, older generation).
    for (int32_t i = 0; i < kSlotCount; ++i) {
        const int32_t slot = pool.Allocate();
        EXPECT_EQ(slot, kSlotCount - 1 - i);
        const BodyHandle handle{slot, pool.GetGeneration(slot)};
        EXPECT_EQ(handle.generation, 2U);
        const BodyHandle& oldHandle = firstRound[static_cast<std::size_t>(slot)];
        EXPECT_EQ(oldHandle.generation, 1U);
        EXPECT_NE(handle, oldHandle);
        EXPECT_FALSE(HandleResolves(pool, oldHandle));
        EXPECT_TRUE(HandleResolves(pool, handle));
    }
    EXPECT_EQ(pool.GetAliveCount(), kSlotCount);
    EXPECT_EQ(pool.GetCapacity(), kSlotCount);

    // A third release bumps to generation 3, so even a round-2 handle goes dead.
    pool.Release(0);
    EXPECT_EQ(pool.GetGeneration(0), 3U);
    EXPECT_FALSE(pool.IsValid(0));
    EXPECT_FALSE(HandleResolves(pool, BodyHandle{0, 2U}));
}

TEST(IDPoolTest, ClearResetsPoolCompletely) {
    IDPool pool;
    (void)pool.Allocate();
    (void)pool.Allocate();
    pool.Release(0);

    pool.Clear();
    EXPECT_EQ(pool.GetAliveCount(), 0);
    EXPECT_EQ(pool.GetCapacity(), 0);
    EXPECT_FALSE(pool.IsValid(0));
    EXPECT_FALSE(pool.IsValidIndex(0));

    // Clear() is a full reset (World::Clear builds on it): the next slot starts
    // again at index 0 / generation 1.
    EXPECT_EQ(pool.Allocate(), 0);
    EXPECT_EQ(pool.GetGeneration(0), 1U);
}

// Debug-only misuse detection: both cases are caller bugs that must be loud in
// debug builds and harmless no-ops in release builds. The fixture turns the test
// into a SKIP on hosts that deny the temp-file write GoogleTest's death-test
// machinery needs (see TestSupport.h), so the suite stays green in sandboxes
// while CI still executes the assertions.
//
// The fixture name is unique across the unit-test tree on purpose: gtest
// requires every TEST_F in one suite to use the same fixture class, and
// test_debug_asserts.cpp defines its own (differently named) death fixtures.
class IDPoolAssertDeathTest : public p2test::DeathTestFixture {};

TEST_F(IDPoolAssertDeathTest, ReleaseOutOfRangeAsserts) {
    IDPool pool;
    EXPECT_DEATH(pool.Release(0), "IDPool::Release");
}

TEST_F(IDPoolAssertDeathTest, DoubleReleaseAsserts) {
    IDPool pool;
    (void)pool.Allocate();
    pool.Release(0);
    EXPECT_DEATH(pool.Release(0), "IDPool::Release");
}

} // namespace
