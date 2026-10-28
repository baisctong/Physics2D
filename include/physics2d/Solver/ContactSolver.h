#pragma once

// p2::ContactSolver — Sequential Impulse (PGS) velocity solver, Week 3 stage.
//
// ============================================================================
// Week 3 scope: normal impulses only
// ============================================================================
// This is the first *real* constraint solver in the project. It replaces the
// Week 2 temporary normal-velocity projection (Section 6.4, P08):
//
//   Prepare()       Precompute, per contact point: lever arms rA/rB, effective
//                   normal mass 1/K_n, and the restitution bias. Positions do not
//                   change during the velocity phase, so the cached levers stay
//                   valid for the whole frame.
//   WarmStart()     Week 5 (apply inherited accumulated impulses once before the
//                   iterations). No-op in Week 3 - P08 explicitly starts without
//                   warm starting.
//   SolveVelocity() `velocityIterations` passes over every prepared point:
//                       vn     = Dot(vRel, normal)
//                       lambda = -(vn + bias) * normalMass
//                       newAcc = max(accumulated + lambda, 0)   // push, never pull
//                       apply (newAcc - accumulated) to A(-n) and B(+n) at rA/rB
//                                                        (including the angular terms)
//
// Restitution bias (D5/P08): with `vn0` the approach speed captured once in
// Prepare and `e` the combined restitution, the target relative normal velocity is
// `-e * vn0`. The solver therefore uses
//     bias = e * vn0            when vn0 < -restitutionThreshold
//     bias = 0                  otherwise (resting contacts must not bounce)
// and lambda = -(vn + bias) / K_n. Note vn0 < 0 for an approaching pair, so the
// bias is negative and the target ends up positive (separating).
//
// Sign conventions (Section 4.2, normative):
//   normal points A -> B; vRel = (vB + wB x rB) - (vA + wA x rA);
//   a positive normal impulse is applied as -J to A and +J to B.
//
// Data ownership: the *persistent* accumulated normal impulse lives in
// ContactPoint::normalImpulse, so Week 5 can inherit it across frames by
// ContactID and warm start from it. ContactSolver only caches derived per-frame
// quantities and writes the accumulated value back after solving.
//
// ============================================================================
// Reserved extensions (Weeks 5-6): tangent/friction constraints (2 tangent
// masses, friction cone clamp), warm starting, split impulse pseudo-velocities,
// and the per-island solve ordering. ConstraintPoint has room for those fields;
// adding them is additive and does not change this file's Week 3 contract.
// ============================================================================

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/Vec2.h"

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
    float bias = 0.0f;               // restitution target term (e * vn0)
    float accumulatedImpulse = 0.0f; // per-frame accumulation (written back to ContactPoint)
};

class ContactSolver {
  public:
    ContactSolver() = default;

    // Builds the per-frame constraint cache from the contact list. Points whose
    // effective normal mass is not positive (both bodies static/infinite, NaN)
    // are skipped with a debug assert: they can never push anything apart.
    void Prepare(const std::vector<Body>& bodies, std::vector<Contact>& contacts,
                 const SolverSettings& settings);

    // RESERVED (Week 5): apply inherited accumulated impulse * warmStartFactor
    // once, before the velocity iterations. No-op in Week 3.
    void WarmStart(std::vector<Body>& bodies, std::vector<Contact>& contacts, const SolverSettings& settings);

    // Sequential normal impulses, `iterations` passes (P08 D4). Writes the final
    // accumulated normal impulse back into every ContactPoint.
    void SolveVelocity(std::vector<Body>& bodies, std::vector<Contact>& contacts, int32_t iterations,
                       const SolverSettings& settings);

    // ---- math helpers, public so unit tests can pin the formulas (P08 D1/D2) ----

    // K_n = invMassA + invMassB + invIA*Cross(rA,n)^2 + invIB*Cross(rB,n)^2
    [[nodiscard]] static float ComputeEffectiveNormalMass(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                          Vec2 rB, Vec2 normal) noexcept;

    // vRel = (vB + wB x rB) - (vA + wA x rA)
    [[nodiscard]] static Vec2 ComputeRelativeVelocity(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                      Vec2 rB) noexcept;

    [[nodiscard]] int32_t GetConstraintCount() const noexcept {
        return static_cast<int32_t>(m_constraints.size());
    }
    [[nodiscard]] const std::vector<ContactConstraintPoint>& GetConstraints() const noexcept {
        return m_constraints;
    }

    void Clear() noexcept { m_constraints.clear(); }

  private:
    std::vector<ContactConstraintPoint> m_constraints;
};

} // namespace p2
