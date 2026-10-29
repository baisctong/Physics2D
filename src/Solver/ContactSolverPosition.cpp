// p2::ContactSolver — split-impulse position layer (Week 5, P10 / Section 6.4).
//
// WHY THIS PHASE EXISTS
// ---------------------
// The velocity layer leaves a small residual penetration every frame: PGS with a
// finite iteration budget cannot drive every contact's relative normal velocity
// to exactly zero, and the discrete integration moves the bodies by dt * v. Over
// hundreds of frames a tall box tower would sink into the ground and into itself.
//
// The Week 2 answer was a direct position push-out (move the bodies apart by a
// mass-weighted fraction of the overlap). It removes the overlap, but it does so
// by teleporting bodies, which is not a velocity change and therefore not
// momentum conserving - and it cannot express the angular part of the contact.
//
// The Week 5 answer is a SEPARATE pseudo-velocity layer:
//   1. Solve pseudo-velocity constraints (one per contact point) whose target is
//      a fraction beta of the overlap excess above the slop band.
//   2. Integrate those pseudo-velocities into the transforms ONCE
//      (xf += dt * pseudoVelocity), with a per-body clamp so a deep overlap can
//      never teleport a body across the world.
//   3. Clear the pseudo-velocities. The real velocities were never read from nor
//      written to, so the correction cannot inject (or remove) kinetic energy.
//
// Why this is better than Baumgarte: the Baumgarte bias (beta/dt)*penetration is
// a target velocity added to the SAME velocity constraint, so the resulting
// impulse changes the real velocity of the bodies. The energy shows up as
// jitter/re-bounce in resting stacks. Split impulse moves the bodies without
// touching their velocity state. `useSplitImpulse == false` keeps the Baumgarte
// path in Prepare() as the comparison baseline (see ContactSolver.h).
//
// The constraint cache (levers, effective mass) is rebuilt here from the
// post-integration transforms: the velocity-phase cache is one IntegratePositions
// step stale by the time phase 8 runs, and the position correction must use the
// levers of the configuration it is actually correcting.

#include "physics2d/Solver/ContactSolver.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

namespace p2 {

namespace {

// One per-contact-point cache entry for the position layer. Kept local instead of
// reusing ContactConstraintPoint because the levers and the target velocity are
// recomputed from the post-integration transforms (see file comment).
struct PositionConstraintPoint {
    int32_t contactIndex = -1;
    int32_t pointIndex = -1;
    Vec2 rA{0.0f, 0.0f};
    Vec2 rB{0.0f, 0.0f};
    Vec2 normal{0.0f, 1.0f};
    float normalMass = 0.0f;               // 1 / K_n, same formula as the velocity layer
    float targetVelocity = 0.0f;           // (beta/dt) * max(0, penetration - slop), >= 0
    float accumulatedPseudoImpulse = 0.0f; // clamped >= 0, discarded after integration
};

} // namespace

void ContactSolver::ClearPseudoVelocities(std::vector<Body>& bodies) const {
    // Pseudo state is per-body and per-step; leaving it set would make the next
    // step's position layer start from a non-zero pseudo velocity (a silent
    // "free push" that would look like energy injection in the tests).
    for (Body& body : bodies) {
        body.pseudoLinearVelocity = Vec2{0.0f, 0.0f};
        body.pseudoAngularVelocity = 0.0f;
    }
}

void ContactSolver::SolvePosition(std::vector<Body>& bodies, std::vector<Contact>& contacts, float dt,
                                  int32_t iterations, const SolverSettings& settings) {
    // dt is the velocity target's time base (beta/dt) and the integration scale;
    // a non-positive value would flip the correction's sign or divide by zero.
    assert(std::isfinite(dt) && dt > 0.0f && "ContactSolver::SolvePosition: dt must be finite and > 0");
    assert(iterations >= 0 && "ContactSolver::SolvePosition: iterations must be non-negative");

    // Start from a clean pseudo state on EVERY path (including the no-op paths
    // below): a caller that reuses the solver must never inherit a stale push.
    ClearPseudoVelocities(bodies);

    // With split impulse disabled the penetration is handled by the Baumgarte
    // bias added in Prepare(); there is nothing left to do at position level.
    // Zero iterations is the explicit "position correction off" experiment.
    if (!settings.useSplitImpulse || iterations <= 0) {
        return;
    }

    // ---------------------------------------------------------------------
    // 1. Rebuild the constraint cache from the post-integration transforms.
    // ---------------------------------------------------------------------
    // The manifold anchors were baked in the bodies' local frames at contact
    // generation time, and Mul(xfA, localPointA) == Mul(xfB, localPointB) held
    // at that moment. After IntegratePositions the two world images have moved
    // apart; their separation along the normal measures how the overlap changed
    // since generation:
    //   separation_now = Dot(worldB - worldA, n)   (+ when the pair separated)
    //   penetration_now = manifold.penetration - separation_now
    // (the stored penetration was measured at generation, so the delta is
    // relative to it).
    std::vector<PositionConstraintPoint> constraints;
    size_t totalPoints = 0;
    for (const Contact& contact : contacts) {
        totalPoints += static_cast<size_t>(contact.manifold.pointCount);
    }
    constraints.reserve(totalPoints);

    const float betaOverDt = settings.splitImpulseBeta / dt;

    // Builds the position constraints of one contact. Shared by the flat and the
    // grouped pass so the per-contact arithmetic cannot drift between modes.
    const auto buildContact = [&](size_t contactIndex) {
        const Contact& contact = contacts[contactIndex];
        const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
        const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            const ContactPoint& point = contact.manifold.points[pointIndex];

            const Vec2 worldA = Mul(bodyA.xf, point.localPointA);
            const Vec2 worldB = Mul(bodyB.xf, point.localPointB);

            PositionConstraintPoint constraint;
            constraint.contactIndex = static_cast<int32_t>(contactIndex);
            constraint.pointIndex = pointIndex;
            constraint.normal = contact.manifold.normal;
            constraint.rA = worldA - bodyA.xf.p;
            constraint.rB = worldB - bodyB.xf.p;

            // The pseudo-velocity impulse changes the configuration exactly like
            // the normal velocity impulse, so the same effective mass applies.
            const float effectiveMass =
                ComputeEffectiveNormalMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.normal);
            if (!(effectiveMass > kEpsilon)) {
                // Static-static/degenerate pair: mirrors the velocity-layer guard
                // (World never creates these, this keeps the layer a total
                // function on hand-built contact lists).
                assert(false && "ContactSolver::SolvePosition: non-positive effective normal mass");
                continue;
            }

            const float separationNow = Dot(worldB - worldA, constraint.normal);
            const float penetrationNow = contact.manifold.penetration - separationNow;
            // Only the overlap beyond the slop band is corrected: treating the
            // band as contact tolerance is what makes resting stacks settle
            // instead of oscillating around zero separation.
            const float penetrationExcess = std::max(0.0f, penetrationNow - settings.splitImpulseSlop);
            if (penetrationExcess <= 0.0f) {
                continue; // already within tolerance: no pseudo impulse needed
            }
            constraint.normalMass = 1.0f / effectiveMass;
            // Velocity that removes beta of the excess in one step; the pseudo
            // velocity is integrated with the same dt below, so the net position
            // change is beta * excess per step.
            constraint.targetVelocity = betaOverDt * penetrationExcess;

            constraints.push_back(constraint);
        }
    };

    if (UsesIslandPartition()) {
        // Grouped order: island by island, island contacts ascending. Sleeping
        // islands are skipped entirely - their pseudo velocities stay zero and the
        // integration pass below leaves their transforms bit-identical. Islands are
        // independent components, so this permutation of the constraints cannot
        // change any float result versus the flat order.
        const std::vector<std::vector<int32_t>>& islands = *m_islandBodyIndices;
        const std::vector<std::vector<int32_t>>& islandContacts = *m_islandContactIndices;
        for (size_t islandIndex = 0; islandIndex < islands.size(); ++islandIndex) {
            bool islandAsleep = true;
            for (const int32_t bodyIndex : islands[islandIndex]) {
                if (bodyIndex >= 0 && static_cast<size_t>(bodyIndex) < bodies.size() &&
                    bodies[static_cast<size_t>(bodyIndex)].isAwake) {
                    islandAsleep = false;
                    break;
                }
            }
            if (islandAsleep) {
                continue;
            }
            for (const int32_t contactIndex : islandContacts[islandIndex]) {
                buildContact(static_cast<size_t>(contactIndex));
            }
        }
    } else {
        // Flat order, skipping contacts whose two bodies are both asleep so the
        // position layer cannot push a sleeping body. Static bodies are born asleep,
        // so the predicate must exclude static-static pairs: those are illegal input
        // that has to reach the effective-mass assert (Week 5 contract), exactly like
        // the velocity layer's skip in Prepare.
        for (size_t contactIndex = 0; contactIndex < contacts.size(); ++contactIndex) {
            const Contact& contact = contacts[contactIndex];
            const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
            const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
            const bool bothAsleep = !bodyA.isAwake && !bodyB.isAwake;
            const bool staticStatic = bodyA.type == BodyType::Static && bodyB.type == BodyType::Static;
            if (bothAsleep && !staticStatic) {
                continue;
            }
            buildContact(contactIndex);
        }
    }

    // ---------------------------------------------------------------------
    // 2. Pseudo-velocity iterations.
    // ---------------------------------------------------------------------
    // Positions are frozen for the whole loop (the pseudo velocities are only
    // integrated once, afterwards), so the target velocity of each constraint is
    // constant here; the iterations still matter because one contact's pseudo
    // impulse changes the pseudo relative velocity of its neighbours (Gauss-
    // Seidel through a stack).
    for (int32_t iteration = 0; iteration < iterations; ++iteration) {
        for (PositionConstraintPoint& constraint : constraints) {
            const Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
            Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
            Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

            const Vec2 pseudoRelativeVelocity =
                ComputePseudoRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float pseudoVn = Dot(pseudoRelativeVelocity, constraint.normal);

            // Drive the pseudo relative velocity to the (positive) target:
            //   lambdaP = -(pseudoVn - target) * normalMass
            // and clamp the ACCUMULATED pseudo impulse to >= 0 so correction can
            // only push apart, never pull together.
            const float lambdaP = -(pseudoVn - constraint.targetVelocity) * constraint.normalMass;
            const float newAccumulated = std::max(constraint.accumulatedPseudoImpulse + lambdaP, 0.0f);
            const float delta = newAccumulated - constraint.accumulatedPseudoImpulse;
            constraint.accumulatedPseudoImpulse = newAccumulated;
            if (delta == 0.0f) {
                continue;
            }

            const Vec2 impulse = constraint.normal * delta;
            // Same application rule as the velocity layer, but on the pseudo
            // state: A gets -J, B gets +J. The real velocities are untouched.
            bodyA.pseudoLinearVelocity -= impulse * bodyA.invMass;
            bodyA.pseudoAngularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
            bodyB.pseudoLinearVelocity += impulse * bodyB.invMass;
            bodyB.pseudoAngularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
        }
    }

    // ---------------------------------------------------------------------
    // 3. Integrate the pseudo velocities into the transforms once, then clear.
    // ---------------------------------------------------------------------
    for (Body& body : bodies) {
        // Zero pseudo velocity == untouched body: skip so unrelated transforms
        // (static/kinematic bodies, bodies without contacts) stay bit-identical.
        if (body.pseudoLinearVelocity == Vec2{0.0f, 0.0f} && body.pseudoAngularVelocity == 0.0f) {
            continue;
        }

        // Per-step translation cap: a single deep overlap must not teleport the
        // body across the world (same spirit as kMaxTranslation in the velocity
        // integrator). Direction is preserved.
        Vec2 translation = body.pseudoLinearVelocity * dt;
        const float translationLength = Length(translation);
        if (translationLength > settings.maxLinearCorrection) {
            translation = translation * (settings.maxLinearCorrection / translationLength);
        }
        body.xf.p += translation;

        // Rotation cap is a fixed angular correction constant (not scaled by dt:
        // it bounds the visible per-step orientation change).
        const float deltaAngle =
            Clamp(body.pseudoAngularVelocity * dt, -kMaxAngularCorrection, kMaxAngularCorrection);
        body.xf.q = Mul(body.xf.q, Rot(deltaAngle));
        // Keep the Rot invariant after composing the correction, exactly like the
        // position integrator does.
        body.xf.q.Normalize();
    }

    // Split impulse relies on the pseudo velocities being transient: if they
    // leaked into the next step they would act as a free velocity that the
    // velocity solver never sees and cannot remove.
    ClearPseudoVelocities(bodies);
}

Vec2 ContactSolver::ComputePseudoRelativeVelocity(const Body& bodyA, const Body& bodyB, Vec2 rA,
                                                  Vec2 rB) noexcept {
    // Same vRel formula as ComputeRelativeVelocity, on the pseudo-velocity state.
    // The two are separate functions on purpose: mixing them up (using the real
    // velocities in the position layer) is exactly the energy-injection bug that
    // split impulse exists to prevent.
    const Vec2 anchorVelocityA = bodyA.pseudoLinearVelocity + Cross(bodyA.pseudoAngularVelocity, rA);
    const Vec2 anchorVelocityB = bodyB.pseudoLinearVelocity + Cross(bodyB.pseudoAngularVelocity, rB);
    return anchorVelocityB - anchorVelocityA;
}

} // namespace p2
