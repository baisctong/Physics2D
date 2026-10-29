// p2::ContactSolver — velocity layer: normal + Coulomb friction Sequential
// Impulse with warm starting and island-grouped solving (W3 P08, W5 P09, W7 P13).
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
//
// Week 7: when World installs its island partition, the same constraints are
// built island by island and the velocity iterations run per island. Islands are
// independent components of the contact graph, so reordering the Gauss-Seidel
// passes across islands cannot change any float result; the canonical order inside
// an island is preserved. That is why the grouped solve is bit-identical to the
// flat one (the Week 7 acceptance test asserts it with bit_cast comparisons).
//
// Grouped mode has two interchangeable variants, selected by
// SolverSettings::packIslandBodies:
//  - direct (DEFAULT): iterate the island's constraints and read/write the World's
//    Body array. Measuring a dense single island (400 bodies / 760 contacts,
//    Release, 8 iterations) showed the packed variant is ~20% SLOWER at this scale
//    because the O(bodies) copy per frame costs more than the locality win while
//    the Body array still fits in cache;
//  - packed (opt-in): copy the island's hot state into a contiguous scratch array,
//    which is the documented layout for much larger scenes (design document
//    3.3/6.5). Both run the same arithmetic on the same values, so the results are
//    bit-identical (covered by the three-mode test).

#include "physics2d/Solver/ContactSolver.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

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
    // `x * 0` is -0.0 for negative x, and -0.0 != +0.0 bitwise. With warm starting
    // disabled the contract says "seed exactly +0"; a factor-0 seed must be the
    // SAME bit pattern or the two configurations can diverge at the bit level once
    // the accumulators stop being rewritten every frame (Week 7: a sleeping island
    // freezes its ContactPoint impulses, so the W5 World-level
    // "useWarmStarting=false == warmStartFactor=0" test exposes it). Adding +0
    // normalises -0 to +0 and leaves every other float untouched.
    return inheritedImpulse * settings.warmStartFactor + 0.0f;
}

// Mutable view of the four velocity fields an impulse touches. Used so the flat
// path (a Body) and the packed island path (PackedSolverBody) call the SAME solve
// function: duplicating the arithmetic is the classic way to break "bit-identical
// when grouped" without noticing.
struct VelocityState {
    Vec2* linearVelocity;
    float* angularVelocity;
    float invMass;
    float invInertia;
};

[[nodiscard]] inline Vec2 RelativeVelocityFromState(Vec2 linearVelocityA, float angularVelocityA, Vec2 rA,
                                                    Vec2 linearVelocityB, float angularVelocityB,
                                                    Vec2 rB) noexcept {
    // Identical formula to ContactSolver::ComputeRelativeVelocity, just on scalar
    // state: vRel = (vB + wB x rB) - (vA + wA x rA).
    const Vec2 anchorVelocityA = linearVelocityA + Cross(angularVelocityA, rA);
    const Vec2 anchorVelocityB = linearVelocityB + Cross(angularVelocityB, rB);
    return anchorVelocityB - anchorVelocityA;
}

// Applies one point's accumulated warm-start impulse to the two body states.
// (The state may be a real Body or a packed scratch body; the formula is shared.)
void ApplyWarmStartImpulse(const ContactConstraintPoint& constraint, VelocityState bodyA,
                           VelocityState bodyB) {
    if (constraint.accumulatedImpulse == 0.0f && constraint.accumulatedTangentImpulse == 0.0f) {
        return; // no inherited impulse: keep the fast path allocation-free
    }

    // Apply n * accN + t * accT once. This is the only place the inherited impulse
    // enters the simulation: the iterations continue from this state, which is
    // what "warm" means. Applying a fraction again here would double-count.
    const Vec2 impulse = constraint.normal * constraint.accumulatedImpulse +
                         constraint.tangent * constraint.accumulatedTangentImpulse;

    // Positive impulse J (normal points A -> B) is applied as -J to A and +J to B,
    // which pushes the pair apart. Friction uses the same rule.
    *bodyA.linearVelocity -= impulse * bodyA.invMass;
    *bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
    *bodyB.linearVelocity += impulse * bodyB.invMass;
    *bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
}

// One Gauss-Seidel pass over a contact point: normal constraint first, tangent
// (Coulomb friction) second. Shared by the flat and the grouped velocity paths.
void SolveConstraintPoint(ContactConstraintPoint& constraint, const Contact& contact, VelocityState bodyA,
                          VelocityState bodyB) {
    // ---- normal constraint first ----------------------------------------
    // Positions are frozen during the velocity phase, so the cached levers stay
    // exact and only the velocities need re-reading.
    const Vec2 normalVelocity =
        RelativeVelocityFromState(*bodyA.linearVelocity, *bodyA.angularVelocity, constraint.rA,
                                  *bodyB.linearVelocity, *bodyB.angularVelocity, constraint.rB);
    const float vn = Dot(normalVelocity, constraint.normal);

    // Sequential impulse with accumulated clamping (Section 6.4):
    // solve the increment that would drive (vn + bias) to zero, clamp the
    // ACCUMULATED impulse to >= 0 (contacts push, never pull) and apply only the
    // difference, so the clamp is a projection on the total impulse.
    const float lambda = -(vn + constraint.bias) * constraint.normalMass;
    const float newAccumulated = std::max(constraint.accumulatedImpulse + lambda, 0.0f);
    const float deltaNormal = newAccumulated - constraint.accumulatedImpulse;
    constraint.accumulatedImpulse = newAccumulated;

    if (deltaNormal != 0.0f) {
        // Positive impulse J = delta * n (n points A -> B) is applied as -J to A
        // and +J to B, which pushes the pair apart. Static/kinematic bodies have
        // invMass = invInertia = 0, so they are a natural no-op here.
        const Vec2 impulse = constraint.normal * deltaNormal;
        *bodyA.linearVelocity -= impulse * bodyA.invMass;
        *bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
        *bodyB.linearVelocity += impulse * bodyB.invMass;
        *bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
    }

    // ---- tangent (Coulomb friction) constraint second -------------------
    // The cone radius uses the normal impulse updated just above: if the cone
    // shrank after a tangent impulse was applied, the accumulated friction could
    // sit outside the (new) cone for the rest of the solve.
    const Vec2 tangentVelocity =
        RelativeVelocityFromState(*bodyA.linearVelocity, *bodyA.angularVelocity, constraint.rA,
                                  *bodyB.linearVelocity, *bodyB.angularVelocity, constraint.rB);
    const float vt = Dot(tangentVelocity, constraint.tangent);

    const float lambdaTangent = -vt * constraint.tangentMass;
    // mu * accN is the cone radius. mu is contact.friction as combined by World
    // (geometric mean) - the solver must not re-combine it.
    const float maxFriction = contact.friction * constraint.accumulatedImpulse;
    const float newAccumulatedTangent =
        Clamp(constraint.accumulatedTangentImpulse + lambdaTangent, -maxFriction, maxFriction);
    const float deltaTangent = newAccumulatedTangent - constraint.accumulatedTangentImpulse;
    constraint.accumulatedTangentImpulse = newAccumulatedTangent;

    if (deltaTangent != 0.0f) {
        const Vec2 impulse = constraint.tangent * deltaTangent;
        *bodyA.linearVelocity -= impulse * bodyA.invMass;
        *bodyA.angularVelocity -= bodyA.invInertia * Cross(constraint.rA, impulse);
        *bodyB.linearVelocity += impulse * bodyB.invMass;
        *bodyB.angularVelocity += bodyB.invInertia * Cross(constraint.rB, impulse);
    }
}

[[nodiscard]] inline VelocityState MakeVelocityState(Body& body) noexcept {
    return VelocityState{&body.linearVelocity, &body.angularVelocity, body.invMass, body.invInertia};
}

// Week 7 sleep-skip predicate. Static bodies are born asleep (Week 1 invariant), so
// "both bodies are not awake" alone also matches an ILLEGAL static-static pair and
// would silently skip it before the effective-mass check. The Week 5 solver contract
// says such input must trip the debug assert (pinned by
// SolverDeathTest.StaticStaticContactAssertsOnNonPositiveMass), so the skip requires
// at least one non-static body: sleeping dynamic contacts are skipped, static-static
// still reaches the assert (and the release-build skip branch).
[[nodiscard]] bool IsSleepingContactPair(const Body& bodyA, const Body& bodyB) noexcept {
    if (bodyA.isAwake || bodyB.isAwake) {
        return false;
    }
    return !(bodyA.type == BodyType::Static && bodyB.type == BodyType::Static);
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

bool ContactSolver::UsesIslandPartition() const noexcept {
    // An installed-but-empty partition is flat mode (documented fallback: the
    // placeholder DFS, a scene without dynamic bodies, or island solving disabled).
    return m_islandBodyIndices != nullptr && m_islandContactIndices != nullptr &&
           !m_islandBodyIndices->empty() && m_islandBodyIndices->size() == m_islandContactIndices->size();
}

void ContactSolver::BuildConstraintsForContact(const std::vector<Body>& bodies,
                                               std::vector<Contact>& contacts, size_t contactIndex,
                                               const SolverSettings& settings, float dt) {
    Contact& contact = contacts[contactIndex];
    assert(contact.bodyA.index >= 0 && static_cast<size_t>(contact.bodyA.index) < bodies.size() &&
           "ContactSolver::Prepare: bodyA index out of range");
    assert(contact.bodyB.index >= 0 && static_cast<size_t>(contact.bodyB.index) < bodies.size() &&
           "ContactSolver::Prepare: bodyB index out of range");
    const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
    const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];

    // mu is already the combined material value (World computes sqrt(muA*muB));
    // combining it again here would square the friction.
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
        // Positions do not change during the velocity phase, so caching them here
        // is valid for every iteration.
        const Vec2 worldA = Mul(bodyA.xf, point.localPointA);
        const Vec2 worldB = Mul(bodyB.xf, point.localPointB);
        constraint.rA = worldA - bodyA.xf.p;
        constraint.rB = worldB - bodyB.xf.p;

        const float effectiveNormalMass =
            ComputeEffectiveNormalMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.normal);
        if (!(effectiveNormalMass > kEpsilon)) {
            // Both bodies are immovable (static-static pair) or the data is
            // degenerate: no impulse can change the relative velocity, so the
            // point is reported in debug builds and skipped rather than dividing
            // by ~0. World avoids creating such pairs.
            assert(false && "ContactSolver::Prepare: non-positive effective normal mass");
            continue;
        }
        // K_n > 0 implies K_t > 0 here: the only way K_n is positive is a dynamic
        // body (invMass > 0, and then invInertia > 0 only for dynamic bodies too),
        // so the translational mass term is shared by both axes.
        const float effectiveTangentMass =
            ComputeEffectiveTangentMass(bodyA, bodyB, constraint.rA, constraint.rB, constraint.tangent);
        assert(effectiveTangentMass > kEpsilon && "ContactSolver::Prepare: non-positive tangent mass");
        constraint.normalMass = 1.0f / effectiveNormalMass;
        constraint.tangentMass = 1.0f / effectiveTangentMass;

        // Restitution bias (D5/P08). vn0 is the approach speed captured once,
        // before any impulse: negative when the pair is closing along the A -> B
        // normal. This is captured BEFORE warm starting: otherwise warm-start
        // impulses would already have changed vn and resting contacts would be
        // re-kicked every frame.
        const Vec2 relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, constraint.rA, constraint.rB);
        const float vn0 = Dot(relativeVelocity, constraint.normal);
        constraint.bias = (vn0 < -settings.restitutionThreshold) ? contact.restitution * vn0 : 0.0f;

        if (!settings.useSplitImpulse) {
            // Comparison baseline: no pseudo-velocity layer, so penetration is
            // removed by a Baumgarte velocity target in the SAME constraint.
            // bias < 0 means "target vn = -bias > 0"; only overlap beyond the slop
            // band is corrected.
            const float penetrationExcess =
                std::max(0.0f, contact.manifold.penetration - settings.linearSlop);
            constraint.bias -= (settings.baumgarte / dt) * penetrationExcess;
        }

        // Week 5 warm-start seeds. The cached accumulator and the impulse actually
        // applied by WarmStart() must stay identical, so warmStartFactor is
        // consumed here exactly once (WarmStart applies the cached value as-is).
        constraint.accumulatedImpulse = WarmStartSeed(point.normalImpulse, contact, settings);
        constraint.accumulatedTangentImpulse = WarmStartSeed(point.tangentImpulse, contact, settings);

        m_constraints.push_back(constraint);
    }
}

void ContactSolver::Prepare(const std::vector<Body>& bodies, std::vector<Contact>& contacts,
                            const SolverSettings& settings, float dt) {
    // dt is only needed by the Baumgarte bias (a velocity target); the position
    // layer takes it again in SolvePosition. Non-positive dt is a caller bug.
    assert(std::isfinite(dt) && dt > 0.0f && "ContactSolver::Prepare: dt must be finite and > 0");

    m_constraints.clear();
    m_islandConstraintRanges.clear();

    // One constraint per manifold point; reserve up front so the steady state does
    // not allocate inside the per-frame rebuild.
    size_t totalPoints = 0;
    for (const Contact& contact : contacts) {
        assert(contact.manifold.pointCount >= 0 && contact.manifold.pointCount <= kMaxManifoldPoints &&
               "ContactSolver::Prepare: pointCount out of range");
        totalPoints += static_cast<size_t>(contact.manifold.pointCount);
    }
    m_constraints.reserve(totalPoints);

    if (UsesIslandPartition()) {
        const std::vector<std::vector<int32_t>>& islands = *m_islandBodyIndices;
        const std::vector<std::vector<int32_t>>& islandContacts = *m_islandContactIndices;
        m_islandConstraintRanges.resize(islands.size());

        for (size_t islandIndex = 0; islandIndex < islands.size(); ++islandIndex) {
            IslandConstraintRange& range = m_islandConstraintRanges[islandIndex];
            range.begin = static_cast<int32_t>(m_constraints.size());

            // Whole-island sleep: membership is atomic, so the island is skipped
            // (no constraints at all) iff every member is asleep. Skipping means
            // no warm start, no solve and no write-back, which leaves the
            // ContactPoint accumulators untouched for the wake frame.
            range.asleep = true;
            for (const int32_t bodyIndex : islands[islandIndex]) {
                assert(bodyIndex >= 0 && static_cast<size_t>(bodyIndex) < bodies.size() &&
                       "ContactSolver::Prepare: island member out of range");
                if (bodyIndex >= 0 && static_cast<size_t>(bodyIndex) < bodies.size() &&
                    bodies[static_cast<size_t>(bodyIndex)].isAwake) {
                    range.asleep = false;
                    break;
                }
            }
            if (range.asleep) {
                range.end = range.begin;
                continue;
            }

            // Canonical order: islandContacts is ascending in global contact
            // order, which is the same relative order the flat solve uses inside
            // this island - required for bit-identical results.
            for (const int32_t contactIndex : islandContacts[islandIndex]) {
                assert(contactIndex >= 0 && static_cast<size_t>(contactIndex) < contacts.size() &&
                       "ContactSolver::Prepare: island contact index out of range");
                BuildConstraintsForContact(bodies, contacts, static_cast<size_t>(contactIndex), settings, dt);
            }
            range.end = static_cast<int32_t>(m_constraints.size());
        }
        return;
    }

    // Flat mode (Week 6 order). Contacts whose two bodies are both asleep are
    // skipped: a sleeping pair must not be solved, and in particular the position
    // layer must not push a sleeping body. Static bodies are born asleep, so a
    // sleeping dynamic body vs the ground is covered by the same test. The predicate
    // deliberately excludes static-static pairs: they are illegal input that must
    // reach BuildConstraintsForContact's effective-mass assert, not be skipped.
    for (size_t contactIndex = 0; contactIndex < contacts.size(); ++contactIndex) {
        const Contact& contact = contacts[contactIndex];
        assert(contact.bodyA.index >= 0 && static_cast<size_t>(contact.bodyA.index) < bodies.size());
        assert(contact.bodyB.index >= 0 && static_cast<size_t>(contact.bodyB.index) < bodies.size());
        const Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
        const Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
        if (IsSleepingContactPair(bodyA, bodyB)) {
            continue;
        }
        BuildConstraintsForContact(bodies, contacts, contactIndex, settings, dt);
    }
}

void ContactSolver::WarmStart(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                              const SolverSettings& settings) {
    // The seeds are already scaled by warmStartFactor in Prepare, so warm starting
    // disabled/zero-factor naturally leaves every accumulated impulse at exactly
    // zero and the per-constraint early-out below is a no-op: there is no second
    // behaviour branch here that could drift from the Prepare semantics.
    static_cast<void>(settings);

    // The only state WarmStart touches is the body velocities, so it does not need
    // the packed layout. Iterating the island ranges (instead of the flat
    // constraint vector) keeps the island boundary explicit and skips sleeping
    // islands even if they had constraints (they never do: Prepare omits them).
    const auto applyAll = [&](int32_t begin, int32_t end) {
        for (int32_t index = begin; index < end; ++index) {
            const ContactConstraintPoint& constraint = m_constraints[static_cast<size_t>(index)];
            Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
            Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
            Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
            ApplyWarmStartImpulse(constraint, MakeVelocityState(bodyA), MakeVelocityState(bodyB));
        }
    };

    if (UsesIslandPartition()) {
        for (const IslandConstraintRange& range : m_islandConstraintRanges) {
            if (range.asleep) {
                continue;
            }
            applyAll(range.begin, range.end);
        }
        return;
    }
    applyAll(0, static_cast<int32_t>(m_constraints.size()));
}

void ContactSolver::PackIslandBodies(const std::vector<Body>& bodies, const std::vector<Contact>& contacts,
                                     int32_t islandIndex) {
    m_packedBodies.clear();
    m_packedBodyIndices.clear();

    // Grow the stamp tables once: body slots are never compacted, so this is a
    // one-time (or scene-growth) cost. A generation stamp makes clearing O(1) per
    // island instead of O(N).
    if (m_packedSlotOfBody.size() < bodies.size()) {
        m_packedSlotOfBody.resize(bodies.size(), -1);
        m_packedStamp.resize(bodies.size(), -1);
    }
    ++m_packGeneration;
    if (m_packGeneration == std::numeric_limits<int32_t>::max()) {
        // Wraparound: retire every stamp so the next generation cannot alias.
        std::fill(m_packedStamp.begin(), m_packedStamp.end(), -1);
        m_packGeneration = 0;
    }

    const auto packBody = [this, &bodies](int32_t bodyIndex) {
        assert(bodyIndex >= 0 && static_cast<size_t>(bodyIndex) < bodies.size());
        if (m_packedStamp[static_cast<size_t>(bodyIndex)] == m_packGeneration) {
            return;
        }
        m_packedStamp[static_cast<size_t>(bodyIndex)] = m_packGeneration;
        m_packedSlotOfBody[static_cast<size_t>(bodyIndex)] = static_cast<int32_t>(m_packedBodies.size());

        const Body& body = bodies[static_cast<size_t>(bodyIndex)];
        PackedSolverBody packed;
        packed.linearVelocity = body.linearVelocity;
        packed.angularVelocity = body.angularVelocity;
        packed.invMass = body.invMass;
        packed.invInertia = body.invInertia;
        m_packedBodies.push_back(packed);
        m_packedBodyIndices.push_back(bodyIndex);
    };

    // Dynamic members first, then whatever static/kinematic boundary bodies the
    // islands' contacts reference (the ground). Boundaries carry invMass =
    // invInertia = 0, so the solve leaves their packed state bit-identical and the
    // write-back is an identity copy - which is why including them keeps the
    // grouped result equal to the flat one.
    const std::vector<std::vector<int32_t>>& islands = *m_islandBodyIndices;
    for (const int32_t bodyIndex : islands[static_cast<size_t>(islandIndex)]) {
        packBody(bodyIndex);
    }

    const std::vector<std::vector<int32_t>>& islandContacts = *m_islandContactIndices;
    for (const int32_t contactIndex : islandContacts[static_cast<size_t>(islandIndex)]) {
        const Contact& contact = contacts[static_cast<size_t>(contactIndex)];
        packBody(contact.bodyA.index);
        packBody(contact.bodyB.index);
    }
}

void ContactSolver::WriteBackPackedBodies(std::vector<Body>& bodies) {
    for (size_t packedIndex = 0; packedIndex < m_packedBodies.size(); ++packedIndex) {
        const int32_t bodyIndex = m_packedBodyIndices[packedIndex];
        Body& body = bodies[static_cast<size_t>(bodyIndex)];
        const PackedSolverBody& packed = m_packedBodies[packedIndex];
        body.linearVelocity = packed.linearVelocity;
        body.angularVelocity = packed.angularVelocity;
    }
}

void ContactSolver::SolveVelocity(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                                  int32_t iterations, const SolverSettings& settings) {
    // Solves one constraint against the World's body vector. Shared by flat mode and
    // by grouped-direct mode (the measured default); the packed path below runs the
    // exact same SolveConstraintPoint on SolverBody copies.
    const auto solveConstraintDirect = [&bodies, &contacts, this](ContactConstraintPoint& constraint) {
        Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
        Body& bodyA = bodies[static_cast<size_t>(contact.bodyA.index)];
        Body& bodyB = bodies[static_cast<size_t>(contact.bodyB.index)];
        SolveConstraintPoint(constraint, contact, MakeVelocityState(bodyA), MakeVelocityState(bodyB));
    };

    if (UsesIslandPartition()) {
        // Island-by-island Gauss-Seidel. Islands are independent, so this is the same
        // sequence of per-island updates as the flat loop below. Two variants:
        //  - grouped-direct (DEFAULT): read/write the Body array directly. Measured
        //    Release cost on a dense single island (400 bodies / 760 contacts,
        //    8 iterations): flat 1.720 ms vs packed 2.121 ms for 40 solves - the
        //    O(bodies) pack/unpack per frame costs more than the locality win once
        //    the Body array still fits in cache. See SolverSettings::packIslandBodies.
        //  - packed (opt-in): copy the island's hot state into a contiguous scratch
        //    array for much larger scenes. Both run identical arithmetic on the same
        //    float values, so the result is bit-identical either way.
        for (int32_t islandIndex = 0; islandIndex < static_cast<int32_t>(m_islandConstraintRanges.size());
             ++islandIndex) {
            const IslandConstraintRange& range = m_islandConstraintRanges[static_cast<size_t>(islandIndex)];
            if (range.asleep || range.begin == range.end) {
                continue; // sleeping island: no warm start, no solve, no write-back
            }

            if (settings.packIslandBodies) {
                PackIslandBodies(bodies, contacts, islandIndex);
                for (int32_t iteration = 0; iteration < iterations; ++iteration) {
                    for (int32_t index = range.begin; index < range.end; ++index) {
                        ContactConstraintPoint& constraint = m_constraints[static_cast<size_t>(index)];
                        const Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
                        PackedSolverBody& packedA = m_packedBodies[static_cast<size_t>(
                            m_packedSlotOfBody[static_cast<size_t>(contact.bodyA.index)])];
                        PackedSolverBody& packedB = m_packedBodies[static_cast<size_t>(
                            m_packedSlotOfBody[static_cast<size_t>(contact.bodyB.index)])];
                        const VelocityState stateA{&packedA.linearVelocity, &packedA.angularVelocity,
                                                   packedA.invMass, packedA.invInertia};
                        const VelocityState stateB{&packedB.linearVelocity, &packedB.angularVelocity,
                                                   packedB.invMass, packedB.invInertia};
                        SolveConstraintPoint(constraint, contact, stateA, stateB);
                    }
                }
                WriteBackPackedBodies(bodies);
            } else {
                for (int32_t iteration = 0; iteration < iterations; ++iteration) {
                    for (int32_t index = range.begin; index < range.end; ++index) {
                        solveConstraintDirect(m_constraints[static_cast<size_t>(index)]);
                    }
                }
            }
        }
    } else {
        // Flat Week 6 order. Constraints for both-asleep contacts were never built.
        for (int32_t iteration = 0; iteration < iterations; ++iteration) {
            for (ContactConstraintPoint& constraint : m_constraints) {
                solveConstraintDirect(constraint);
            }
        }
    }

    // Persist the per-frame accumulated impulses into the contact points (also for
    // iterations <= 0, where they stay at the warm-start seeds). Skipped contacts
    // (sleeping islands / both-asleep pairs) have no constraint, so their
    // accumulators stay untouched - exactly what the wake frame needs.
    for (const ContactConstraintPoint& constraint : m_constraints) {
        Contact& contact = contacts[static_cast<size_t>(constraint.contactIndex)];
        ContactPoint& point = contact.manifold.points[constraint.pointIndex];
        point.normalImpulse = constraint.accumulatedImpulse;
        point.tangentImpulse = constraint.accumulatedTangentImpulse;
    }
}

void ContactSolver::SetIslandPartition(const std::vector<std::vector<int32_t>>* islandBodies,
                                       const std::vector<std::vector<int32_t>>* islandContacts) {
    // Week 7: store the non-owning view. Prepare rebuilds the per-island
    // constraint ranges from it every frame; passing nullptr twice restores the
    // flat Week 6 order. An installed-but-empty partition is flat mode too (see
    // UsesIslandPartition).
    m_islandBodyIndices = islandBodies;
    m_islandContactIndices = islandContacts;
    m_islandConstraintRanges.clear();
}

} // namespace p2
