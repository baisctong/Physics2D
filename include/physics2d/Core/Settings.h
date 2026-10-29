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
// Allowed residual penetration of the position layer (m). The pseudo-velocity
// bias only removes overlap deeper than this band.
//
// Week 5 change (registered in docs/dev-interfaces.md section 5): the Week 1
// placeholder value was 0.01 m, exactly the Week 5 milestone limit ("max
// penetration < 0.01 m"). A correction dead-band equal to the acceptance limit
// can never satisfy it, so the band is now the universal contact tolerance.
// 0.01 m stays the *acceptance* threshold, not the solver's target.
inline constexpr float kSplitImpulseSlop = kLinearSlop;

// Warm starting applies a fraction of the previous frame's accumulated impulse
// when seeding the solver. Default 1.0 = trust last frame's solution completely.
//
// Week 5 measurement (registered in docs/dev-interfaces.md section 5): the Week 1
// plan suggested 0.85 ("1.0 over-commits and oscillates") without having run a
// stack. The measured 10-box tower says the opposite: at 8 velocity iterations
// a partial seed throws away (1 - factor) of the converged impulse every frame,
// so PGS has to re-resolve it and its constraint-ordering bias is re-injected
// every frame - which is exactly what warm starting exists to suppress.
// Measured collapse step for a 10x1 m tower (default 8/3 iterations, friction
// 0.3, split impulse on; run 600 steps = 10 s):
//   factor 0.00 -> collapses at step 258    0.50 -> 314
//   factor 0.85 -> collapses at step 259    0.90 -> 283   0.95 -> 332
//   factor 1.00 -> does not collapse (max penetration 5.2 mm, top y 9.478 m)
// Every factor below 1 loses the tower within 10 s; the exact collapse step is
// transient-dependent, so only "1.0 holds / <1.0 does not" is a contract.
// 1.0 is also what Box2D does (full warm starting); the factor stays exposed in
// SolverSettings as the P09 comparison knob, not as a tuning crutch.
inline constexpr float kWarmStartFactor = 1.0f;

// Restitution is ignored below this approach speed (m/s). Without the
// threshold, resting stacks re-bounce forever on sub-millimetre approaches.
inline constexpr float kRestitutionVelocityThreshold = 1.0f;

// ---- tunnelling defences (Section 6.6) --------------------------------------

// Maximum translation / rotation per physics step. Box2D-style clamp: cheap,
// deterministic and covers most fast-object cases without a full CCD system.
inline constexpr float kMaxTranslation = 2.0f;
inline constexpr float kMaxRotation = 0.5f * kPi;

// ---- broad phase (Week 6) ----------------------------------------------------

// Fat AABB margin (m). World::Synchronize grows every body's tight AABB by this
// before the broad phase sees it. Why: the broad phase is fed with bounds that
// are one step old (phase 2 runs before the next Synchronize), and a proxy whose
// bounds are exact would drop a pair in the frame a body moves a fraction of a
// millimetre. 0.1 m is Box2D's b2_aabbExtension; at our 0.1-10 m scale it is
// large enough to absorb one step of ordinary motion (<= kMaxTranslation is
// handled per-frame because the proxy is updated every step) and small enough
// that it does not flood the narrow phase with false positives.
inline constexpr float kAabbMargin = 0.1f;

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

// ---- Week 5: split impulse replaced the Week 2 temporary response -----------
// `kPositionCorrectionFactor` (the Week 2 push-out scale) was removed in Week 5:
// the split-impulse position layer is the only position-level response now.
// See docs/dev-interfaces.md section 5 for the deviation entry.

// Infinite-plane AABB half extent. Planes are handled analytically by the narrow
// phase; until the broad phase special-cases them (Week 6) their bounds are a
// deliberately huge box so the AABB candidate filter never drops the ground.
inline constexpr float kPlaneAabbExtent = 1.0e6f;

// ---- runtime-editable solver parameters -------------------------------------

// The subset of the constants above that the testbed exposes as sliders
// (Week 5 D6) and the regression tool can override. Defaults mirror the
// constexpr values so that "config == defaults" is the shipped behaviour.
struct SolverSettings {
    int32_t velocityIterations = kVelocityIterations;
    int32_t positionIterations = kPositionIterations;

    // Week 4 D6: SAT axis hysteresis (see Collision/SATCache.h). Keeping the
    // previous reference face among near-tied axes stabilises ContactID keys on
    // resting stacks; disabling it must not change the physical result, only the
    // amount of manifold/contact-key churn (used for the D6 comparison experiment).
    bool useSatCache = true;

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

    // ---- Week 7: islands & sleeping ------------------------------------
    // Island-grouped solving. The grouped order is BIT-IDENTICAL to the flat
    // Week 6 order because constraints from different islands touch disjoint body
    // sets (Gauss-Seidel updates commute across islands); the flag exists for the
    // P13 A/B verification and for debugging, not because the physics differ.
    bool useIslandSolving = true;

    // Whole-island sleeping. Disable to keep every body awake (A/B measurement of
    // the sleeping win, and regression scenes that must keep exercising
    // continuous dynamics; per-body `allowSleep = false` is the finer control).
    bool enableSleeping = true;

    // Week 7: copy each island's hot velocity state into a contiguous scratch
    // array before solving (design document 3.3/6.5 "cache friendly packing").
    // Implemented and verified bit-identical, but DEFAULT OFF because the Week 7
    // measurement says it does not pay at this scale: Release, 400 bodies / 760
    // contacts in one island, 8 iterations -> flat 1.720 ms vs packed 2.121 ms
    // (packed ~23% slower); at 400 bodies the whole Body array is ~80 KB and fits
    // L2, so the O(bodies) copy cost per frame exceeds the locality gain. The
    // packed path stays available (and is exercised by the bit-identity tests) for
    // much larger scenes where bodies no longer fit in cache.
    bool packIslandBodies = false;
};

} // namespace p2
