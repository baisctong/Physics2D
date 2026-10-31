#pragma once

// p2::ContactSolver — Sequential Impulse (PGS) constraint solver.
//
// ============================================================================
// Week 3: normal impulses (P08)            -> done
// Week 5: friction, warm starting, split impulse position layer (P09/P10)
// ============================================================================
// The velocity layer solves two one-dimensional constraints per manifold point
// in the SAME iteration - normal first, tangent second:
//
//   Normal (non-penetration, C_dot = vRel . n >= 0)
//     K_n      = invMassA + invMassB + invIA*Cross(rA,n)^2 + invIB*Cross(rB,n)^2
//     lambda   = -(vn + bias) / K_n
//     accN_new = max(accN + lambda, 0)          // contacts push, never pull
//     apply (accN_new - accN) * n               // A gets -J, B gets +J
//
//   Tangent (Coulomb friction, |accT| <= mu * accN)
//     t        = Skew(normal) = Perp(normal)
//     K_t      = invMassA + invMassB + invIA*Cross(rA,t)^2 + invIB*Cross(rB,t)^2
//     lambdaT  = -(vRel . t) / K_t
//     maxF     = mu * accN_new                  // after the normal update
//     accT_new = Clamp(accT + lambdaT, -maxF, maxF)
//     apply (accT_new - accT) * t
//
// Ordering and clamping rules (they are the Week 5 stability contract):
// - The normal constraint is solved before the tangent one within an iteration,
//   because the friction cone radius depends on the *updated* normal impulse;
//   using the pre-update value would let the cone shrink under an already
//   applied tangent impulse.
// - The clamp is applied to the ACCUMULATED impulse, never to the increment:
//   clamping increments can drift outside the cone and makes stacking unstable.
// - `mu` is the pre-combined contact friction (World computes sqrt(muA*muB));
//   the solver must not combine it again.
// - `bias` is captured in Prepare, i.e. BEFORE warm starting, from the initial
//   relative velocity (restitution) and - only when split impulse is disabled -
//   from the Baumgarte penetration term. Capturing restitution after warm start
//   would "bounce" resting contacts forever.
//
// Warm starting (P09 D4): the persistent accumulated impulses live in
// ContactPoint::normalImpulse / tangentImpulse and are inherited across frames
// by World (matched by ContactID::Key). Prepare seeds the per-frame accumulator
// with `inherited * warmStartFactor`; WarmStart applies that seed once before
// the iterations, so the cached accumulator and the actually applied impulse
// never diverge (warmStartFactor is the trust knob: 0 == Week 4 behaviour,
// 1 == trust last frame's solution completely).
//
// Split impulse position layer (P10, Section 6.4): penetration is corrected by a
// separate pseudo-velocity solve that never touches the real velocities:
//
//   for each position iteration:
//     separation_now  = Dot(Mul(xfB,localPointB) - Mul(xfA,localPointA), n)
//     penetration_now = manifold.penetration - separation_now
//     excess          = max(0, penetration_now - splitImpulseSlop)
//     target          = (beta / dt) * excess              // separating velocity
//     pseudoVn        = pseudo relative normal velocity
//     lambdaP         = -(pseudoVn - target) / K_n
//     accP_new        = max(accP + lambdaP, 0)            // push only
//   after the iterations:
//     xf.p += dt * pseudoLinearVelocity       (clamped to maxLinearCorrection)
//     xf.q  = xf.q * Rot(dt * pseudoAngularVelocity)  (clamped to kMaxAngularCorrection)
//     then the pseudo-velocities are cleared.
//
// When `useSplitImpulse` is false the position layer is skipped and Prepare adds
// the Baumgarte bias (beta/dt)*max(0, penetration - linearSlop) to the velocity
// constraint instead. That is the documented comparison baseline: it stabilises
// penetration too, but it injects energy back into the real velocities (resting
// stacks jitter / re-bounce), which is exactly what split impulse avoids.
//
// Sign conventions (Section 4.2, normative):
//   normal points A -> B; vRel = (vB + wB x rB) - (vA + wA x rA);
//   a positive normal impulse is applied as -J to A and +J to B; for the pseudo
//   velocities the same application rule holds.
//
// Data ownership: the *persistent* accumulated impulses live in ContactPoint, so
// World can inherit them by ContactID across frames. ContactSolver only caches
// derived per-frame quantities and writes the accumulated values back after
// solving. The position layer rebuilds its own lever arms from the
// post-integration transforms because the velocity cache is one integration step
// stale by the time phase 8 runs.

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/Vec2.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace p2 {

// One prepared contact point. `contactIndex`/`pointIndex` identify the
// persistent storage (Contact::manifold.points) the accumulated impulse belongs to.
struct ContactConstraintPoint {
    int32_t contactIndex = -1;
    int32_t pointIndex = -1;

    Vec2 rA{0.0f, 0.0f};             // lever arm: body A origin -> anchor (world)
    Vec2 rB{0.0f, 0.0f};             // lever arm: body B origin -> anchor (world)
    Vec2 normal{0.0f, 1.0f};         // contact normal, A -> B
    float normalMass = 0.0f;         // 1 / K_n
    float bias = 0.0f;               // restitution + (Baumgarte when split impulse is off)
    float accumulatedImpulse = 0.0f; // per-frame normal accumulation (written back to ContactPoint)

    // ---- Week 5 additions (additive; the Week 3 fields above are unchanged) ----

    Vec2 tangent{1.0f, 0.0f};               // unit tangent t = Skew(normal)
    float tangentMass = 0.0f;               // 1 / K_t
    float accumulatedTangentImpulse = 0.0f; // friction cone accumulator, |accT| <= mu*accN
};

class ContactSolver {
  public:
    ContactSolver() = default;

    // Builds the per-frame constraint cache from the contact list. Points whose
    // effective normal mass is not positive (both bodies static/infinite, NaN)
    // are skipped with a debug assert: they can never push anything apart.
    //
    // Week 5: also caches the tangent direction/mass, seeds the accumulated
    // impulses from ContactPoint (times warmStartFactor when warm starting is on)
    // and adds the Baumgarte bias when split impulse is disabled. `dt` is only
    // consumed by that bias term; the default is the fixed step, and World passes
    // the step it is actually integrating so the bias stays consistent.
    void Prepare(const std::vector<Body>& bodies, std::vector<Contact>& contacts,
                 const SolverSettings& settings, float dt = kFixedTimeStep);

    // Applies the seeded accumulated impulses (n * accN + t * accT) once, before
    // the velocity iterations. No-op when warm starting is disabled (the seeds
    // are zero then, so this is an early-out, not a behaviour branch).
    void WarmStart(std::vector<Body>& bodies, std::vector<Contact>& contacts, const SolverSettings& settings);

    // Sequential impulses, `iterations` passes: normal then tangent per point.
    // Writes the final accumulated impulses back into every ContactPoint.
    void SolveVelocity(std::vector<Body>& bodies, std::vector<Contact>& contacts, int32_t iterations,
                       const SolverSettings& settings);

    // Split impulse position layer (phase 8). Solves `iterations` pseudo-velocity
    // passes, integrates the pseudo-velocities into the body transforms once,
    // and clears them. Never reads or writes linearVelocity/angularVelocity of
    // any body. With `useSplitImpulse == false` it only clears the pseudo state
    // (penetration is then handled by the Baumgarte bias).
    void SolvePosition(std::vector<Body>& bodies, std::vector<Contact>& contacts, float dt,
                       int32_t iterations, const SolverSettings& settings);

    // Zeroes the pseudo-velocities of every body. SolvePosition does this itself;
    // the method is public so World::Clear (and tests) can guarantee no pseudo
    // state leaks across scene rebuilds.
    void ClearPseudoVelocities(std::vector<Body>& bodies) const;

    // ---- Week 7: island partition (P13) ----------------------------------
    // Installs the canonical island partition built by World::BuildIslands. When
    // installed, Prepare/SolveVelocity/SolvePosition process contacts island by
    // island: islands in canonical order (smallest body index first), contact
    // indices ascending (global order) inside each island, and sleeping islands
    // (all member bodies asleep) skipped entirely. Because constraints from
    // different islands touch disjoint body sets, this grouped order is
    // BIT-IDENTICAL to the flat Week 6 order - the P13 "与不按岛求解的结果 hash
    // 一致" requirement.
    //
    // The velocity solve has two interchangeable variants selected by
    // SolverSettings::packIslandBodies: direct (default; reads/writes the Body
    // array) and packed (copies the island's hot state into a contiguous scratch
    // array - the design-document layout for very large scenes). The packed
    // variant measured slower on the Week 7 dense-island benchmark, so it is
    // opt-in; both are bit-identical and covered by the three-mode test.
    //
    // The pointers must stay valid while installed. World passes its own member
    // vectors, which live as long as the World; tests may install their own.
    // Passing nullptr (or never calling this) means flat mode, i.e. exactly the
    // Week 6 behaviour. An INSTALLED but EMPTY partition (zero islands, e.g. a
    // scene with no dynamic bodies or the Week 7 DFS placeholder) is also treated
    // as flat mode, so the tree stays functional while the DFS lands.
    void SetIslandPartition(const std::vector<std::vector<int32_t>>* islandBodies,
                            const std::vector<std::vector<int32_t>>* islandContacts);
    void ClearIslandPartition() noexcept {
        m_islandBodyIndices = nullptr;
        m_islandContactIndices = nullptr;
    }
    [[nodiscard]] bool HasIslandPartition() const noexcept { return m_islandBodyIndices != nullptr; }

    // ---- math helpers, public so unit tests can pin the formulas (P08 D1/D2) ----

    // K_n = invMassA + invMassB + invIA*Cross(rA,n)^2 + invIB*Cross(rB,n)^2
    [[nodiscard]] static float ComputeEffectiveNormalMass(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                          Vec2 rB, Vec2 normal) noexcept;

    // K_t: identical form with the tangent axis. Kept as a named helper so the
    // tests (and the header comments) can distinguish the two constraints.
    [[nodiscard]] static float ComputeEffectiveTangentMass(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                           Vec2 rB, Vec2 tangent) noexcept;

    // vRel = (vB + wB x rB) - (vA + wA x rA)
    [[nodiscard]] static Vec2 ComputeRelativeVelocity(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                      Vec2 rB) noexcept;

    // Pseudo version of the same formula: uses pseudoLinearVelocity /
    // pseudoAngularVelocity only (split impulse position layer).
    [[nodiscard]] static Vec2 ComputePseudoRelativeVelocity(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                            Vec2 rB) noexcept;

    [[nodiscard]] int32_t GetConstraintCount() const noexcept {
        return static_cast<int32_t>(m_constraints.size());
    }
    [[nodiscard]] const std::vector<ContactConstraintPoint>& GetConstraints() const noexcept {
        return m_constraints;
    }

    void Clear() noexcept { m_constraints.clear(); }

  private:
    // ---- Week 7: island-grouped solving (P13) ----------------------------
    // Hot per-body state the velocity iterations actually touch. The island path
    // copies an island's bodies into a contiguous array of these, solves there and
    // writes back; the flat path works on Body directly. Both paths run the same
    // arithmetic on the same bit patterns, which is what keeps the grouped result
    // bit-identical to the flat one.
    struct PackedSolverBody {
        Vec2 linearVelocity{0.0f, 0.0f};
        float angularVelocity = 0.0f;
        float invMass = 0.0f;
        float invInertia = 0.0f;
    };

    // [begin, end) range of m_constraints belonging to one island, in canonical
    // island order. A sleeping island (or one without contacts) has begin == end.
    struct IslandConstraintRange {
        int32_t begin = 0;
        int32_t end = 0;
        bool asleep = false;
    };

    // True when the installed partition is usable: both pointers set, non-empty
    // and same size. An installed-but-empty partition is the documented flat-mode
    // fallback (placeholder / scene without dynamic bodies).
    [[nodiscard]] bool UsesIslandPartition() const noexcept;

    // Builds the constraints of one contact. Shared by the flat and the grouped
    // Prepare so both orders run identical per-contact code.
    void BuildConstraintsForContact(const std::vector<Body>& bodies, std::vector<Contact>& contacts,
                                    size_t contactIndex, const SolverSettings& settings, float dt);

    // Copies the bodies referenced by island `islandIndex` (dynamic members and
    // the static/kinematic boundary bodies of its contacts) into m_packedBodies.
    void PackIslandBodies(const std::vector<Body>& bodies, const std::vector<Contact>& contacts,
                          int32_t islandIndex);

    // Writes the packed island velocities back into the Body slots.
    void WriteBackPackedBodies(std::vector<Body>& bodies);

    std::vector<ContactConstraintPoint> m_constraints;

    // Week 7: non-owning view of the World's canonical island partition. Both
    // pointers are null in flat mode. The contact lists are ascending global
    // contact indices; see Solver/Island.h for the determinism contract.
    const std::vector<std::vector<int32_t>>* m_islandBodyIndices = nullptr;
    const std::vector<std::vector<int32_t>>* m_islandContactIndices = nullptr;

    // Grouped-iteration cache, rebuilt by Prepare every frame.
    std::vector<IslandConstraintRange> m_islandConstraintRanges;
    std::vector<PackedSolverBody> m_packedBodies;
    std::vector<int32_t> m_packedBodyIndices; // packed slot -> body slot index
    std::vector<int32_t> m_packedSlotOfBody;  // body slot -> packed slot (stamped)
    std::vector<int32_t> m_packedStamp;       // body slot -> pack generation
    int32_t m_packGeneration = 0;
};

} // namespace p2
