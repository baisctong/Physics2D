// Week 1 D6 (P05 / Section 4.3): fixed-timestep accumulation unit tests.
//
// The accumulator turns jittery wall-clock frame time into a whole number of
// fixed physics steps. The 0.25 s fixed step and the 0.125/0.0625 s frame times
// used below are exact binary fractions, so the integer results and the
// remainders are exactly representable: no tolerance is needed for the counts.

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Core/Settings.h"
#include "physics2d/Core/TimeStep.h"

#include <cmath>
#include <limits>

namespace {

// Exact binary values keep the hand-computed sequences unambiguous.
constexpr float kTestFixedDt = 0.25f;
constexpr int32_t kTestMaxSteps = 3;

using p2::StepAccumulator;

TEST(StepAccumulatorTest, StoresConfigurationAndStartsEmpty) {
    const StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_FLOAT_EQ(accumulator.GetFixedDeltaTime(), kTestFixedDt);
    EXPECT_EQ(accumulator.GetMaxStepsPerFrame(), kTestMaxSteps);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.0f);
}

TEST(StepAccumulatorTest, DefaultsMatchSettings) {
    // The defaults are the project-wide fixed step and spiral guard, so the
    // testbed does not have to pass them explicitly.
    const StepAccumulator accumulator;
    EXPECT_FLOAT_EQ(accumulator.GetFixedDeltaTime(), p2::kFixedTimeStep);
    EXPECT_EQ(accumulator.GetMaxStepsPerFrame(), p2::kMaxStepsPerFrame);
}

TEST(StepAccumulatorTest, ExactStepConsumesAccumulator) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_EQ(accumulator.Advance(kTestFixedDt), 1);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.0f);
}

TEST(StepAccumulatorTest, RemainderIsCarriedToNextFrame) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    // 0.125 = dt/2: not enough for a step; half a step is accumulated.
    EXPECT_EQ(accumulator.Advance(0.125f), 0);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.125f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.5f);
    // Another dt/2 brings the total to exactly dt -> one step, empty again.
    EXPECT_EQ(accumulator.Advance(0.125f), 1);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.0f);
    // 0.0625 = dt/4: a quarter step accumulated, alpha 0.25.
    EXPECT_EQ(accumulator.Advance(0.0625f), 0);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.25f);
}

TEST(StepAccumulatorTest, SeveralStepsInOneFrame) {
    // 0.75 = 3 * 0.25 -> three fixed steps and no remainder.
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_EQ(accumulator.Advance(0.75f), 3);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
}

TEST(StepAccumulatorTest, MaxStepsClampDiscardsBacklog) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    // 1.0 s asks for 4 fixed steps. The spiral guard allows 3 (0.75 s of
    // simulation) and drops the rest: queueing it would make the next frame run
    // more steps and never recover. Here the dropped part is exactly one step,
    // so fmod(0.25, 0.25) == 0 and the accumulator is empty.
    EXPECT_EQ(accumulator.Advance(1.0f), 3);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    // No debt is carried over: the next frame of one step runs exactly one step.
    EXPECT_EQ(accumulator.Advance(kTestFixedDt), 1);
}

TEST(StepAccumulatorTest, MaxStepsClampKeepsSubStepPhase) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    // 1.3125 s = 5.25 fixed steps. Three steps run; the backlog is 0.5625 s =
    // 2.25 steps. The whole steps are discarded but the sub-step phase survives:
    // fmod(0.5625, 0.25) = 0.0625 s = alpha 0.25, so render interpolation stays
    // continuous across the clamp.
    EXPECT_EQ(accumulator.Advance(1.3125f), 3);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0625f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.25f);
}

TEST(StepAccumulatorTest, NegativeFrameTimeIsIgnored) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_EQ(accumulator.Advance(0.125f), 0);
    // A negative frame time carries no simulation time and must not "give back"
    // time already accumulated.
    EXPECT_EQ(accumulator.Advance(-1.0f), 0);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.125f);
    // The 0.125 from the first frame is still there, so dt is reached now.
    EXPECT_EQ(accumulator.Advance(0.125f), 1);
}

TEST(StepAccumulatorTest, NonFiniteFrameTimeIsIgnored) {
    // Infinity would make the step count explode and NaN would poison every
    // later comparison, so both are treated like "no time passed".
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_EQ(accumulator.Advance(std::numeric_limits<float>::infinity()), 0);
    EXPECT_EQ(accumulator.Advance(std::numeric_limits<float>::quiet_NaN()), 0);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    EXPECT_EQ(accumulator.Advance(kTestFixedDt), 1);
}

TEST(StepAccumulatorTest, ResetClearsAccumulator) {
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    EXPECT_EQ(accumulator.Advance(0.125f), 0);
    accumulator.Reset();
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    EXPECT_FLOAT_EQ(accumulator.GetAlpha(), 0.0f);
    // The clock restarts from zero: half a step is again not enough.
    EXPECT_EQ(accumulator.Advance(0.125f), 0);
}

TEST(StepAccumulatorTest, DefaultTimestepJitterSequence) {
    // 30 frames of exactly one fixed step: every Advance adds and immediately
    // consumes the same float, so every frame runs exactly one step.
    StepAccumulator accumulator;
    for (int32_t frame = 0; frame < 30; ++frame) {
        EXPECT_EQ(accumulator.Advance(p2::kFixedTimeStep), 1);
    }
    // Two half-steps on consecutive frames: dt/2 + dt/2 == dt exactly (halving
    // and re-doubling a float is lossless), so frame 2 runs one step.
    EXPECT_EQ(accumulator.Advance(0.5f * p2::kFixedTimeStep), 0);
    EXPECT_EQ(accumulator.Advance(0.5f * p2::kFixedTimeStep), 1);
    EXPECT_FLOAT_EQ(accumulator.GetAccumulatedTime(), 0.0f);
    // A one-second stall is clamped to kMaxStepsPerFrame and must leave no
    // backlog that could trigger the death spiral on the following frame.
    EXPECT_EQ(accumulator.Advance(1.0f), p2::kMaxStepsPerFrame);
    EXPECT_EQ(accumulator.Advance(p2::kFixedTimeStep), 1);
}

TEST(StepAccumulatorTest, AlphaStaysInUnitInterval) {
    // Alpha is the render interpolation factor of Section 4.3 and must always be
    // in [0, 1), including after odd jitter patterns (multiples of 1/32 s).
    StepAccumulator accumulator(kTestFixedDt, kTestMaxSteps);
    for (int32_t frame = 0; frame < 100; ++frame) {
        const float frameTime = 0.03125f * static_cast<float>((frame % 7) + 1);
        const int32_t stepCount = accumulator.Advance(frameTime);
        EXPECT_GE(stepCount, 0);
        EXPECT_LE(stepCount, kTestMaxSteps);
        const float alpha = accumulator.GetAlpha();
        EXPECT_GE(alpha, 0.0f);
        EXPECT_LT(alpha, 1.0f);
    }
}

// Debug-only: an invalid configuration is a programmer error and must abort
// loudly (the release build falls back to the project defaults instead). The
// fixture skips this test where the sandbox forbids the temp-file write that a
// gtest death test needs (see TestSupport.h). The fixture name is unique across
// the unit-test tree because gtest requires one fixture class per test suite.
class StepAccumulatorAssertDeathTest : public p2test::DeathTestFixture {};

TEST_F(StepAccumulatorAssertDeathTest, ZeroFixedDeltaAsserts) {
    EXPECT_DEATH(
        {
            StepAccumulator invalid(0.0f, p2::kMaxStepsPerFrame);
            (void)invalid;
        },
        "StepAccumulator");
}

TEST_F(StepAccumulatorAssertDeathTest, ZeroMaxStepsAsserts) {
    EXPECT_DEATH(
        {
            StepAccumulator invalid(p2::kFixedTimeStep, 0);
            (void)invalid;
        },
        "StepAccumulator");
}

} // namespace
