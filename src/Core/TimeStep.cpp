// p2::StepAccumulator — fixed-timestep conversion (Week 1 D6, Section 4.3).
//
// The accumulator is the only place where variable wall-clock frame time meets
// the fixed physics dt. It is deliberately free of any windowing/rendering
// dependency so the spiral-of-death logic can be unit-tested headlessly (P05).

#include "physics2d/Core/TimeStep.h"

#include <cassert>
#include <cmath>
#include <cstdint>

namespace p2 {

StepAccumulator::StepAccumulator(float fixedDt, int32_t maxStepsPerFrame)
    : m_fixedDt(fixedDt), m_maxStepsPerFrame(maxStepsPerFrame) {
    assert(std::isfinite(fixedDt) && fixedDt > 0.0f &&
           "StepAccumulator: fixed delta time must be finite and > 0");
    assert(maxStepsPerFrame >= 1 && "StepAccumulator: maxStepsPerFrame must be at least 1");

    // Release-build guard for the same programmer errors. The accumulator feeds
    // a simulation loop, so a zero dt would divide by zero and an unbounded step
    // count would hang a stalled frame; falling back to the project defaults is
    // strictly better than poisoning the loop forever.
    if (!std::isfinite(fixedDt) || fixedDt <= 0.0f) {
        m_fixedDt = kFixedTimeStep;
    }
    if (m_maxStepsPerFrame < 1) {
        m_maxStepsPerFrame = 1;
    }
}

int32_t StepAccumulator::Advance(float frameTime) {
    // Negative frame times carry no simulation time. Non-finite samples are
    // treated the same way: one NaN would otherwise make every comparison false
    // and the accumulator would stay poisoned for the rest of the run.
    if (!std::isfinite(frameTime) || frameTime < 0.0f) {
        frameTime = 0.0f;
    }
    m_accumulator += frameTime;

    int32_t stepCount = 0;
    while (m_accumulator >= m_fixedDt && stepCount < m_maxStepsPerFrame) {
        m_accumulator -= m_fixedDt;
        ++stepCount;
    }

    if (stepCount == m_maxStepsPerFrame && m_accumulator >= m_fixedDt) {
        // Spiral-of-death guard (Section 4.3): the frame asked for more steps
        // than allowed. Queueing the backlog would make the next frame run even
        // more steps and never recover, so the excess whole steps are dropped.
        // The sub-step remainder is kept, which keeps the render interpolation
        // alpha continuous across the clamp.
        m_accumulator = std::fmod(m_accumulator, m_fixedDt);
        assert(m_accumulator >= 0.0f && m_accumulator < m_fixedDt);
    }
    return stepCount;
}

float StepAccumulator::GetAlpha() const {
    // The Advance() invariant keeps m_accumulator in [0, m_fixedDt), so this is
    // already the [0, 1) interpolation factor of Section 4.3. Clamp for the
    // pathological rounding case where acc / dt rounds up to exactly 1.
    float alpha = m_accumulator / m_fixedDt;
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha >= 1.0f) {
        alpha = std::nextafter(1.0f, 0.0f);
    }
    return alpha;
}

void StepAccumulator::Reset() noexcept {
    m_accumulator = 0.0f;
}

} // namespace p2
