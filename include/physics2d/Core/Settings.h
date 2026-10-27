#pragma once

// p2::Settings — every tunable number in the engine lives here (Section 4.4).
//
// Rule: implementation files must not contain magic numbers. If a constant is
// physically meaningful (slop, beta, iteration counts) it belongs in this
// header so that the ImGui panel can expose it and the devlog can cite it.
//
// Values are deliberately `inline constexpr` so each translation unit can fold
// them without ODR concerns. `SolverSettings` carries the subset that is
// mutable at runtime; World copies it and the testbed edits the copy.

#include "physics2d/Math/MathUtils.h"

namespace p2 {

// ---- contact tolerances ------------------------------------------------------

// Allowed small penetration (m). Contacts inside this band are treated as
// touching: pushing them apart only injects jitter, never real stability.
inline constexpr float kLinearSlop = 0.005f;

// Allowed small angular penetration (rad).
inline constexpr float kAngularSlop = 2.0f * kDegToRad;

// Per-step position-correction caps. They stop a single deep overlap from
// teleporting a body across the world (a classic source of "explosions").
inline constexpr float kMaxLinearCorrection = 0.2f;
inline constexpr float kMaxAngularCorrection = 8.0f * kDegToRad;

// ---- stability parameters ----------------------------------------------------

// Baumgarte stabilisation factor, only used when split impulse is disabled.
inline constexpr float kBaumgarte = 0.2f;

// Split impulse (pseudo-velocity) position correction (Section 6.4).
inline constexpr float kSplitImpulseBeta = 0.2f;
inline constexpr float kSplitImpulseSlop = 0.01f;

// Warm starting applies a fraction of the previous frame's accumulated
// impulse. Values in [0.8, 0.95] work; 1.0 over-commits and oscillates.
inline constexpr float kWarmStartFactor = 0.85f;

// Restitution is ignored below this approach speed (m/s). Without the
// threshold, resting stacks re-bounce forever on sub-millimetre approaches.
inline constexpr float kRestitutionVelocityThreshold = 1.0f;

// ---- tunnelling defences (Section 6.6) --------------------------------------

// Maximum translation / rotation per physics step. Box2D-style clamp: cheap,
// deterministic and covers most fast-object cases without a full CCD system.
inline constexpr float kMaxTranslation = 2.0f;
inline constexpr float kMaxRotation = 0.5f * kPi;

// ---- solver iteration defaults ---------------------------------------------

inline constexpr int32_t kVelocityIterations = 8;
inline constexpr int32_t kPositionIterations = 3;

// ---- sleeping (Week 7) -------------------------------------------------------

inline constexpr float kSleepLinearTolerance = 0.01f; // m/s
inline constexpr float kSleepAngularTolerance = 2.0f * kDegToRad;
inline constexpr float kSleepTimeThreshold = 0.5f; // seconds below tolerance

// ---- time stepping (Section 4.3) --------------------------------------------

// Fixed timestep. The physics never sees a variable dt: warm-start impulses
// and the split-impulse bias both assume a constant dt, and determinism
// regression tests depend on it.
inline constexpr float kFixedTimeStep = 1.0f / 60.0f;

// Safety valve against the "death spiral": a stalled frame runs at most this
// many physics steps, the rest of the accumulated time is discarded.
inline constexpr int32_t kMaxStepsPerFrame = 5;

// ---- default environment -----------------------------------------------------

// Integer gravity for easy mental arithmetic in demos (Section 4.1).
inline constexpr float kDefaultGravityY = -10.0f;

// ---- runtime-editable solver parameters -------------------------------------

// The subset of the constants above that the testbed exposes as sliders
// (Week 5 D6) and the regression tool can override. Defaults mirror the
// constexpr values so that "config == defaults" is the shipped behaviour.
struct SolverSettings {
    int32_t velocityIterations = kVelocityIterations;
    int32_t positionIterations = kPositionIterations;

    float warmStartFactor = kWarmStartFactor;
    bool useWarmStarting = true;

    bool useSplitImpulse = true;
    float splitImpulseBeta = kSplitImpulseBeta;
    float splitImpulseSlop = kSplitImpulseSlop;
    float baumgarte = kBaumgarte;

    float linearSlop = kLinearSlop;
    float maxLinearCorrection = kMaxLinearCorrection;

    // Reserved (Week 5): exposed for the friction/restitution experiments.
    float restitutionThreshold = kRestitutionVelocityThreshold;
};

} // namespace p2
