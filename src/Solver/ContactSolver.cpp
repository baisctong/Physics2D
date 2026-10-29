// p2::ContactSolver — velocity layer: normal + Coulomb friction Sequential
// Impulse with warm starting (Week 3 P08, Week 5 P09).
//
// The equations implemented here are the ones documented in
// Solver/ContactSolver.h and Section 6.4 of the design document; the comments
// below spell out the derivation once instead of restating the code.
//
// Why a separate solver object owns the constraint cache: positions are frozen
// during the velocity phase, so rA/rB, K_n/K_t and the restitution bias only
// need to be computed once per frame (Prepare). The iterations then only
// re-evaluate velocities, which keeps the inner loop to one dot product and one
// clamp per constraint axis - the shape of a real PGS solver.
//
// Week 5 order inside one iteration: normal first, tangent second. The friction
// cone radius is mu * accN, so the tangent clamp must use the normal impulse that
// was just updated - using last iteration's value could let the cone shrink after
// a tangent impulse has already been applied (see ContactSolver.h).

#include "physics2d/Solver/ContactSolver.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace p2 {

namespace {

// Warm-start seed for one accumulated impulse.
//
// The persistent value lives on ContactPoint and was inherited by World from the
// previous frame (matched by ContactID::Key). Conditions, all of which are part
// of the Week 5 contract:
//  - settings.useWarmStarting == false -> start from zero (exactly the Week 4
//    behaviour, used as the comparison baseline);
//  - contact.isPersistent == false -> the cached impulse does not belong to this
//    configuration, so it must not be used even if the field happens to be
//    non-zero (World only inherits matched points, but the solver is a public
//    unit and must be safe on hand-built contacts too);
//  - warmStartFactor scales the trust in last frame's solution (0 == Week 4
//    behaviour, 1 == trust it completely).
[[nodiscard]] float WarmStartSeed(float inheritedImpulse, const Contact& contact,
                                  const SolverSettings& settings) noexcept {
    if (!settings.useWarmStarting || !contact.isPersistent) {
        return 0.0f;
    }
    return inheritedImpulse * settings.warmStartFactor;
}

} // namespace

float ContactSolver::ComputeEffectiveNormalMass(const Body& bodyA, const Body& bodyB, Vec2 rA, Vec2 rB,
                                                Vec2 normal) noexcept {
    // Derivation (Section 6.4): the normal constraint is C_dot = vRel . n >= 0
    // with vRel = (vB + wB x rB) - (vA + wA x rA). An impulse J = lambda * n
    // applied at the anchors changes vRel by
    //   dvRel = lambda * (invMassA + invMassB) * n
    //          + lambda * invIA * Cross(rA, n) * Cross(rA, n) * n
    //          + lambda * invIB * Cross(rB, n) * Cross(rB, n) * n,
    // so the inverse effective mass is
    //   K_n = invMassA + invMassB + invIA*Cross(rA,n)^2 + invIB*Cross(rB,n)^2.
    // Cross(r, n) = r.x*n.y - r.y*n.x is the signed lever arm of the normal: it
    // vanishes when the lever arm is parallel to the normal (a contact through
    // the line of action), which is exactly when the impulse produces no torque
    // and the angular terms drop out.
    const float crossA = Cross(rA, normal);
    const float crossB = Cross(rB, normal);
    const float k = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crossA * crossA +
                    bodyB.invInertia * crossB * crossB;
    // A finite K_n is a precondition for the reciprocal in Prepare; a NaN here
    // means the caller passed a corrupt body/transform and must not be masked.
    assert(std::isfinite(k) && "ComputeEffectiveNormalMass: non-finite effective mass");
    return k;
}

float ContactSolver::ComputeEffectiveTangentMass(const Body& bodyA, const Body& bodyB, Vec2 rA, Vec2 rB,
                                                 Vec2 tangent) noexcept {
    // Identical derivation with the tangent axis: the friction constraint is
    // 1-D along t, so the angular coupling is invI * Cross(r, t)^2. Keeping this
    // as a separate named helper (instead of reusing the normal one) makes the
    // two constraints distinguishable in tests and in the profiler.
    const float crossA = Cross(rA, tangent);
    const float crossB = Cross(rB, tangent);
    const float k = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crossA * crossA +
                    bodyB.invInertia * crossB * crossB;
    assert(std::isfinite(k) && "ComputeEffectiveTangentMass: non-finite effective mass");
    return k;
}

Vec2 ContactSolver::ComputeRelativeVelocity(const Body& bodyA, const Body& bodyB, Vec2 rA, Vec2 rB) noexcept {
    // 2D angular velocity is a scalar w along +z, so w x r is the Vec2 overload
    // Cross(float, Vec2) = (-w*r.y, w*r.x): the anchor velocity of a body is
    // v + w x r (Section 4.2). The contact-relative velocity is B minus A so that
    // a positive value means the pair is separating along the A -> B normal.
    const Vec2 anchorVelocityA = bodyA.linearVelocity + Cross(bodyA.angularVelocity, rA);
    const Vec2 anchorVelocityB = bodyB.linearVelocity + Cross(bodyB.angularVelocity, rB);
    return anchorVelocityB - anchorVelocityA;
}

void ContactSolver::Prepare(const std::vector<Body>& bodies, std::vector<Contact>& contacts,
                            const SolverSettings& settings, float dt) {
    // dt is only needed by the Baumgarte bias (a velocity target); the position
    // layer takes it again in SolvePosition. Non-positive dt is a caller bug.
    assert(std::isfinite(dt) && dt > 0.0f && "ContactSolver::Prepare: dt must be finite and > 0");

    m_constraints.clear();

    // One constraint per manifold point; reserve up front so the steady state
    // does not allocate inside the per-frame rebuild.
    size_t totalPoints = 0;
    for (const Contact& contact : contacts) {
        assert(contact.manifold.pointCount >= 0 && contact.manifold.pointCount <= kMaxManifoldPoints &&
               "ContactSolver::Prepare: pointCount out of range");
        totalPoints += static_cast<size_t>(contact.manifold.pointCount);
    }
    m_constraints.reserve(totalPoints);

    for (size_t contactIndex = 0; contactIndex < contacts.size(); ++contactIndex) {
        Contact& contact = contacts[contactIndex];
        assert(contact.bodyA.index >= 0 && static_cast<size_t>(contact.bodyA.index) < bodies.size() &&
               "ContactSolver::Prepare: bodyA index out of range");
        assert(contact.bodyB.index >= 0 && static_cast<size_t>(contact.bodyB.index) < bodies.size() &&
               "ContactSolver::Prepare: bodyB index out of range");
        const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
        const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

        // mu is already the combined material value (World computes
        // sqrt(muA*muB)); combining it again here would square the friction.
        // The local exists only for the debug assert, hence the NDEBUG guard:
        // without it MSVC /W4 reports C4189 in release builds (same pattern as
        // ComputePlaneMass in Week 2).
        const float friction = contact.friction;
        static_cast<void>(friction);
        assert(friction >= 0.0f && "ContactSolver::Prepare: negative friction");

        // The tangent is fixed in Prepare because positions (and therefore the
        // normal) are frozen for the whole velocity phase.
        const Vec2 tangent = Skew(contact.manifold.normal);

        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            ContactPoint& point = contact.manifold.points[pointIndex];

            ContactConstraintPoint constraint;
            constraint.contactIndex = static_cast<int32_t>(contactIndex);
            constraint.pointIndex = pointIndex;
            constraint.normal = contact.manifold.normal;
            constraint.tangent = tangent;

            // Lever arms from each body origin to the shared world anchor.
            // Positions do not change during the velocity phase, so caching them
            // here is valid for every iteration.
            const Vec2 worldA = Mul(bodyA.xf, point.localPointA);
            const Vec2 worldB = Mul(bodyB.xf, point.localPointB);
            constraint.rA = worldA - bodyA.xf.p;
            constraint.rB = worldB - bodyB.xf.p;

            const float effectiveNormalMass =
                ComputeEffectiveNormalMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.normal);
            if (!(effectiveNormalMass > kEpsilon)) {
                // Both bodies are immovable (static-static pair) or the data is
                // degenerate: no impulse can change the relative velocity, so the
                // point is reported in debug builds and skipped rather than
                // dividing by ~0. World avoids creating such pairs; this is the
                // internal-misuse guard promised by the header.
                assert(false && "ContactSolver::Prepare: non-positive effective normal mass");
                continue;
            }
            // K_n > 0 implies K_t > 0 here: the only way K_n is positive is a
            // dynamic body (invMass > 0, and then invInertia > 0 only for dynamic
            // bodies too), so the translational mass term is shared by both axes.
            const float effectiveTangentMass =
                ComputeEffectiveTangentMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.tangent);
            assert(effectiveTangentMass > kEpsilon && "ContactSolver::Prepare: non-positive tangent mass");
            constraint.normalMass = 1.0f / effectiveNormalMass;
            constraint.tangentMass = 1.0f / effectiveTangentMass;

            // Restitution bias (D5/P08). vn0 is the approach speed captured once,
            // before any impulse: negative when the pair is closing along the
            // A -> B normal. The target relative normal velocity is -e*vn0
            // (positive, separating); writing the equation
            //     lambda = -(vn + bias) * normalMass
            // the bias that reaches that target in the limit is bias = e*vn0.
            // The threshold keeps resting contacts from bouncing on numerical
            // noise: below it the target is simply vn == 0.
            //
            // This is captured BEFORE warm starting (Prepare runs before
            // WarmStart): otherwise warm-start impulses would already have
            // changed vn and resting contacts would be re-kicked every frame.
            const Vec2 relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float vn0 = Dot(relativeVelocity, constraint.normal);
            constraint.bias = (vn0 < -settings.restitutionThreshold) ? contact.restitution * vn0 : 0.0f;

            if (!settings.useSplitImpulse) {
                // Comparison baseline: no pseudo-velocity layer, so penetration is
                // removed by a Baumgarte velocity target in the SAME constraint.
                // bias < 0 means "target vn = -bias > 0": the impulse pushes the
                // pair apart and the resulting velocity change is real (this is
                // the energy injection split impulse avoids). Only overlap beyond
                // the slop band is corrected: contacts inside the band are
                // considered touching.
                const float penetrationExcess =
                    std::max(0.0f, contact.manifold.penetration - settings.linearSlop);
                constraint.bias -= (settings.baumgarte / dt) * penetrationExcess;
            }

            // Week 5 warm-start seeds. The cached accumulator and the impulse
            // actually applied by WarmStart() must stay identical, so the
            // warmStartFactor is consumed here exactly once (WarmStart applies the
            // cached value as-is). SolveVelocity writes the final accumulator back
            // into ContactPoint for the next frame to inherit.
            constraint.accumulatedImpulse = WarmStartSeed(point.normalImpulse, contact, settings);
            constraint.accumulatedTangentImpulse = WarmStartSeed(point.tangentImpulse, contact, settings);

            m_constraints.push_back(constraint);
        }
    }
}

void ContactSolver::WarmStart(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                              const SolverSettings& settings) {
    // The seeds are already scaled by warmStartFactor in Prepare, so warm starting
    // disabled/zero-factor naturally leaves every accumulated impulse at exactly
    // zero and this loop is an early-out: there is no second behaviour branch
    // here that could drift from the Prepare semantics.
    static_cast<void>(settings);

    for (const ContactConstraintPoint& constraint : m_constraints) {
        if (constraint.accumulatedImpulse == 0.0f && constraint.accumulatedTangentImpulse == 0.0f) {
            continue;
        }
        Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
        Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
        Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

        // Apply n * accN + t * accT once. This is the only place the inherited
        // impulse enters the simulation: the iterations continue from this state,
        // which is what "warm" means. Applying a fraction again here would
        // double-count the factor.
        const Vec2 impulse = constraint.normal * constraint.accumulatedImpulse +
                             constraint.tangent * constraint.accumulatedTangentImpulse;

        // Positive impulse J (normal points A -> B) is applied as -J to A and
        // +J to B, which pushes the pair apart. Friction uses the same rule: the
        // tangent impulse opposes the relative tangential motion.
        bodyA.linearVelocity -= impulse * bodyA.invMass;
        bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
        bodyB.linearVelocity += impulse * bodyB.invMass;
        bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
    }
}

void ContactSolver::SolveVelocity(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                                  int32_t iterations, const SolverSettings& settings) {
    // The iteration count is the only solve-time parameter; the restitution and
    // Baumgarte biases were consumed by Prepare and do not change mid-solve.
    static_cast<void>(settings);

    for (int32_t iteration = 0; iteration < iterations; ++iteration) {
        for (ContactConstraintPoint& constraint : m_constraints) {
            Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
            Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
            Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

            // ---- normal constraint first ------------------------------------
            // Positions are frozen during the velocity phase, so the cached
            // levers stay exact and only the velocities need re-reading.
            const Vec2 normalVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float vn = Dot(normalVelocity, constraint.normal);

            // Sequential impulse with accumulated clamping (Section 6.4):
            // solve the increment that would drive (vn + bias) to zero, clamp the
            // ACCUMULATED impulse to >= 0 (contacts push, never pull) and apply
            // only the difference, so the clamp is a projection on the total
            // impulse rather than a per-iteration hack.
            const float lambda = -(vn + constraint.bias) * constraint.normalMass;
            const float newAccumulated = std::max(constraint.accumulatedImpulse + lambda, 0.0f);
            const float deltaNormal = newAccumulated - constraint.accumulatedImpulse;
            constraint.accumulatedImpulse = newAccumulated;

            if (deltaNormal != 0.0f) {
                // Positive impulse J = delta * n (n points A -> B) is applied as
                // -J to A and +J to B, which pushes the pair apart.
                const Vec2 impulse = constraint.normal * deltaNormal;

                // Linear and angular response (Section 4.2). Static/kinematic
                // bodies have invMass = invInertia = 0, so they are a natural
                // no-op here and the solver needs no body-type branches.
                bodyA.linearVelocity -= impulse * bodyA.invMass;
                bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
                bodyB.linearVelocity += impulse * bodyB.invMass;
                bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
            }

            // ---- tangent (Coulomb friction) constraint second ---------------
            // The cone radius uses the normal impulse updated just above: if the
            // cone shrank after a tangent impulse was applied, the accumulated
            // friction could sit outside the (new) cone for the rest of the solve.
            const Vec2 tangentVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float vt = Dot(tangentVelocity, constraint.tangent);

            const float lambdaTangent = -vt * constraint.tangentMass;
            // mu * accN is the cone radius. mu is contact.friction as combined by
            // World (geometric mean) - the solver must not re-combine it.
            const float maxFriction = contact.friction * constraint.accumulatedImpulse;
            const float newAccumulatedTangent =
                Clamp(constraint.accumulatedTangentImpulse + lambdaTangent, -maxFriction, maxFriction);
            const float deltaTangent = newAccumulatedTangent - constraint.accumulatedTangentImpulse;
            constraint.accumulatedTangentImpulse = newAccumulatedTangent;

            if (deltaTangent != 0.0f) {
                const Vec2 impulse = constraint.tangent * deltaTangent;
                bodyA.linearVelocity -= impulse * bodyA.invMass;
                bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
                bodyB.linearVelocity += impulse * bodyB.invMass;
                bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
            }
        }
    }

    // Persist the per-frame accumulated impulses into the contact points (also
    // for iterations <= 0, where they stay at the warm-start seeds). These fields
    // are the data source for the next frame's warm starting, so the solver owns
    // writing them back every frame - including the tangent accumulator, whose
    // friction cone is only defined relative to this frame's normal impulse.
    for (const ContactConstraintPoint& constraint : m_constraints) {
        Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
        ContactPoint& point = contact.manifold.points[constraint.pointIndex];
        point.normalImpulse = constraint.accumulatedImpulse;
        point.tangentImpulse = constraint.accumulatedTangentImpulse;
    }
}

} // namespace p2
