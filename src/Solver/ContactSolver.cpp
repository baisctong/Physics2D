// p2::ContactSolver — normal-impulse Sequential Impulse (Week 3, P08).
//
// The equations implemented here are the ones documented in
// Solver/ContactSolver.h and Section 6.4 of the design document; the comments
// below spell out the derivation once instead of restating the code.
//
// Why a separate solver object owns the constraint cache: positions are frozen
// during the velocity phase, so rA/rB, K_n and the restitution bias only need to
// be computed once per frame (Prepare). The iterations then only re-evaluate
// velocities, which keeps the inner loop to one dot product and one clamp per
// contact point - the shape of a real PGS solver.

#include "physics2d/Solver/ContactSolver.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace p2 {

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
                            const SolverSettings& settings) {
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

        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            ContactPoint& point = contact.manifold.points[pointIndex];

            // Week 3 solves each frame from scratch: the accumulated impulse is
            // recomputed by the iterations below. Week 5 changes this line to
            // inherit last frame's value (matched by ContactID::Key) and applies
            // it once in WarmStart().
            point.normalImpulse = 0.0f;

            ContactConstraintPoint constraint;
            constraint.contactIndex = static_cast<int32_t>(contactIndex);
            constraint.pointIndex = pointIndex;
            constraint.normal = contact.manifold.normal;

            // Lever arms from each body origin to the shared world anchor.
            // Positions do not change during the velocity phase, so caching them
            // here is valid for every iteration.
            const Vec2 worldA = Mul(bodyA.xf, point.localPointA);
            const Vec2 worldB = Mul(bodyB.xf, point.localPointB);
            constraint.rA = worldA - bodyA.xf.p;
            constraint.rB = worldB - bodyB.xf.p;

            const float effectiveMass =
                ComputeEffectiveNormalMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.normal);
            if (!(effectiveMass > kEpsilon)) {
                // Both bodies are immovable (static-static pair) or the data is
                // degenerate: no impulse can change the relative velocity, so the
                // point is reported in debug builds and skipped rather than
                // dividing by ~0. World avoids creating such pairs; this is the
                // internal-misuse guard promised by the header.
                assert(false && "ContactSolver::Prepare: non-positive effective normal mass");
                continue;
            }
            constraint.normalMass = 1.0f / effectiveMass;

            // Restitution bias (D5/P08). vn0 is the approach speed captured once,
            // before any impulse: negative when the pair is closing along the
            // A -> B normal. The target relative normal velocity is -e*vn0
            // (positive, separating); writing the equation
            //     lambda = -(vn + bias) * normalMass
            // the bias that reaches that target in the limit is bias = e*vn0.
            // The threshold keeps resting contacts from bouncing on numerical
            // noise: below it the target is simply vn == 0.
            const Vec2 relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float vn0 = Dot(relativeVelocity, constraint.normal);
            constraint.bias = (vn0 < -settings.restitutionThreshold) ? contact.restitution * vn0 : 0.0f;

            m_constraints.push_back(constraint);
        }
    }
}

void ContactSolver::WarmStart(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                              const SolverSettings& settings) {
    // RESERVED (Week 5): apply accumulatedImpulse * warmStartFactor once before
    // the iterations so the solver starts from last frame's solution. Week 3
    // deliberately solves from zero (P08: "无 warm starting"); applying anything
    // here would double-count impulses once the real implementation lands.
    static_cast<void>(bodies);
    static_cast<void>(contacts);
    static_cast<void>(settings);
}

void ContactSolver::SolveVelocity(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                                  int32_t iterations, const SolverSettings& settings) {
    // The iteration count is the only solve-time parameter; the restitution
    // threshold was consumed by Prepare and does not change mid-solve.
    static_cast<void>(settings);

    for (int32_t iteration = 0; iteration < iterations; ++iteration) {
        for (ContactConstraintPoint& constraint : m_constraints) {
            Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
            Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
            Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

            // Positions are frozen during the velocity phase, so the cached
            // levers stay exact and only the velocities need re-reading.
            const Vec2 relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
            const float vn = Dot(relativeVelocity, constraint.normal);

            // Sequential impulse with accumulated clamping (Section 6.4):
            // solve the increment that would drive (vn + bias) to zero, clamp the
            // ACCUMULATED impulse to >= 0 (contacts push, never pull) and apply
            // only the difference, so the clamp is a projection on the total
            // impulse rather than a per-iteration hack.
            const float lambda = -(vn + constraint.bias) * constraint.normalMass;
            const float newAccumulated = std::max(constraint.accumulatedImpulse + lambda, 0.0f);
            const float delta = newAccumulated - constraint.accumulatedImpulse;
            constraint.accumulatedImpulse = newAccumulated;
            if (delta == 0.0f) {
                continue; // clamped at zero: nothing to apply this pass
            }

            // Positive impulse J = delta * n (n points A -> B) is applied as -J
            // to A and +J to B, which pushes the pair apart.
            const Vec2 impulse = constraint.normal * delta;

            // Linear and angular response (Section 4.2). Static/kinematic bodies
            // have invMass = invInertia = 0, so they are a natural no-op here and
            // the solver needs no body-type branches.
            bodyA.linearVelocity -= impulse * bodyA.invMass;
            bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
            bodyB.linearVelocity += impulse * bodyB.invMass;
            bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
        }
    }

    // Persist the per-frame accumulated impulse into the contact point (also for
    // iterations <= 0, where it stays at the zero set by Prepare). Week 5's warm
    // starting reads this field, so the solver owns writing it back every frame.
    for (const ContactConstraintPoint& constraint : m_constraints) {
        Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
        contact.manifold.points[constraint.pointIndex].normalImpulse = constraint.accumulatedImpulse;
    }
}

} // namespace p2
