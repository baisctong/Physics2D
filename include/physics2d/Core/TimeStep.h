#pragma once

// p2::TimeStep / p2::StepAccumulator — fixed-timestep time management
// (Section 4.3).
//
// The physics only ever sees the fixed dt. The renderer may run at any frame
// rate, so the host converts wall-clock frame time into a whole number of
// physics steps with an accumulator. This is the single place where variable
// frame time is turned into fixed steps; World::Step itself never sees jitter.
//
// StepAccumulator is a plain value type with no GLFW/ImGui dependency so that
// the logic is unit-testable headlessly (P05: "把 accumulator 做成可测的独立
// 函数"). The testbed owns one and calls Advance() once per rendered frame.
//
// Spiral-of-death protection: Advance() never returns more than
// maxStepsPerFrame; any leftover time is discarded (not queued), so a stalled
// frame cannot make the next frame run even more steps.

#include "physics2d/Core/Settings.h"

#include <cstdint>

namespace p2 {

struct TimeStep {
    float dt = kFixedTimeStep;
    float invDt = 1.0f / kFixedTimeStep;
    int32_t velocityIterations = kVelocityIterations;
    int32_t positionIterations = kPositionIterations;
};

class StepAccumulator {
  public:
    explicit StepAccumulator(float fixedDt = kFixedTimeStep, int32_t maxStepsPerFrame = kMaxStepsPerFrame);

    // Feed one rendered frame's wall-clock time; returns how many fixed steps
    // the caller must run. Negative frame times are treated as zero.
    [[nodiscard]] int32_t Advance(float frameTime);

    // Fraction of a fixed step currently accumulated, in [0, 1). Renderers use
    // it as the interpolation alpha between the previous and current physics
    // states (Section 4.3, optional Week 7 feature).
    [[nodiscard]] float GetAlpha() const;
    [[nodiscard]] float GetAccumulatedTime() const noexcept { return m_accumulator; }
    [[nodiscard]] float GetFixedDeltaTime() const noexcept { return m_fixedDt; }
    [[nodiscard]] int32_t GetMaxStepsPerFrame() const noexcept { return m_maxStepsPerFrame; }

    void Reset() noexcept;

  private:
    float m_fixedDt = kFixedTimeStep;
    float m_accumulator = 0.0f;
    int32_t m_maxStepsPerFrame = kMaxStepsPerFrame;
};

} // namespace p2
