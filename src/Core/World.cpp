// p2::World — body ownership and the fixed simulation pipeline (Section 4.6,
// Week 1 D6). See World.h for the frozen 11-phase pipeline contract.
//
// Week 1 really runs phases 1 (IntegrateForces), 7 (IntegratePositions) and 11
// (Validate); phases 2-6 and 8-10 are empty hooks whose only purpose is to pin
// the documented ordering so later weeks cannot insert a phase in the wrong
// place. Body lifespan is generation-checked (IDPool), so a destroyed body's
// handle goes stale instead of aliasing the slot's next occupant.

#include "physics2d/Core/World.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>

namespace p2 {

namespace {

// Boundary validation for user input (Section 3.4: never silently repair).
// A rejected BodyDef makes CreateBody return an invalid handle and leaves both
// the body pool and the slot array untouched, so a rejected call is a no-op.
[[nodiscard]] bool IsValidBodyDef(const BodyDef& def) {
    if (!IsValid(def.position) || !std::isfinite(def.angle) || !IsValid(def.linearVelocity) ||
        !std::isfinite(def.angularVelocity)) {
        return false;
    }
    if (!std::isfinite(def.mass) || !std::isfinite(def.inertia) || !std::isfinite(def.density) ||
        !std::isfinite(def.friction) || !std::isfinite(def.restitution)) {
        return false;
    }
    if (def.friction < 0.0f || def.restitution < 0.0f || def.density < 0.0f) {
        return false;
    }
    // Mass properties only matter for dynamic bodies. Static/kinematic bodies
    // ignore them by design, so their values are not a rejection criterion.
    if (def.type == BodyType::Dynamic && (def.mass <= 0.0f || def.inertia <= 0.0f)) {
        return false;
    }
    return true;
}

} // namespace

World::World(Vec2 gravity) : m_gravity(gravity) {
    assert(p2::IsValid(gravity) && "World: non-finite gravity");
}

World::~World() = default;

// ---------------------------------------------------------------------------
// Body lifecycle
// ---------------------------------------------------------------------------

BodyHandle World::CreateBody(const BodyDef& def) {
    if (!IsValidBodyDef(def)) {
        return BodyHandle{};
    }

    const int32_t index = m_bodyIds.Allocate();
    // Allocate() reuses a released slot when one is available (index < size) and
    // appends the next fresh index otherwise, so the slot array grows only in
    // the latter case. Slots are never compacted: handles stay stable.
    if (index >= static_cast<int32_t>(m_bodies.size())) {
        m_bodies.emplace_back();
    }

    Body& body = m_bodies[static_cast<std::size_t>(index)];
    body.handle = BodyHandle{index, m_bodyIds.GetGeneration(index)};
    body.Initialize(def);
    return body.handle;
}

void World::DestroyBody(BodyHandle handle) {
    // Stale/unknown handles are ignored on purpose: destruction is idempotent,
    // so cleanup paths may run twice. The generation check also means a stale
    // handle can never destroy the newer body that reused its slot.
    if (!IsValid(handle)) {
        return;
    }

    Body& body = m_bodies[static_cast<std::size_t>(handle.index)];
    body.handle = BodyHandle{}; // a dead slot no longer advertises a handle
    body.isAwake = false;
    m_bodyIds.Release(handle.index);
}

bool World::IsValid(BodyHandle handle) const {
    if (!handle.IsValid()) {
        return false;
    }
    const int32_t index = handle.index;
    // Range check first: after Clear() a stale handle may point past the arrays.
    // Liveness + generation together reject both destroyed slots and the ABA
    // case where the slot has been reused by a newer body.
    return m_bodyIds.IsValidIndex(index) && m_bodyIds.IsValid(index) &&
           m_bodyIds.GetGeneration(index) == handle.generation;
}

const Body* World::GetBody(BodyHandle handle) const {
    // Stale handles yield nullptr (documented in the header) instead of tripping
    // the internal assertion: UI panels legitimately poll a handle for one frame
    // after destroying the body.
    if (!IsValid(handle)) {
        return nullptr;
    }
    return GetBodyChecked(handle);
}

// ---------------------------------------------------------------------------
// Stable state accessors
//
// All read accessors tolerate stale handles and return neutral defaults; the
// world has no dangling references to hand out. Mutating accessors are no-ops
// for stale handles (defensive) and debug-assert the internal fetch, so a bug
// in the engine itself is still caught.
// ---------------------------------------------------------------------------

Vec2 World::GetPosition(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->GetPosition() : Vec2{0.0f, 0.0f};
}

Transform World::GetTransform(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->xf : Transform{};
}

void World::SetTransform(BodyHandle handle, Vec2 position, float angle) {
    assert(p2::IsValid(position) && std::isfinite(angle) && "World::SetTransform: non-finite transform");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->xf.p = position;
    body->xf.q.Set(angle);
    // Week 7 sleep hook: teleporting a body invalidates its sleep history. A
    // static body is always asleep - it has no velocity to integrate anyway.
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetAngle(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->GetAngle() : 0.0f;
}

Vec2 World::GetLinearVelocity(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->linearVelocity : Vec2{0.0f, 0.0f};
}

void World::SetLinearVelocity(BodyHandle handle, Vec2 velocity) {
    assert(p2::IsValid(velocity) && "World::SetLinearVelocity: non-finite velocity");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->linearVelocity = velocity;
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetAngularVelocity(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->angularVelocity : 0.0f;
}

void World::SetAngularVelocity(BodyHandle handle, float angularVelocity) {
    assert(std::isfinite(angularVelocity) && "World::SetAngularVelocity: non-finite velocity");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->angularVelocity = angularVelocity;
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetMass(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->mass : 0.0f;
}

float World::GetInvMass(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->invMass : 0.0f;
}

BodyType World::GetBodyType(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    // A dead body is inert; Static is the neutral "never moves" answer and keeps
    // generic UI loops free of special cases.
    return body != nullptr ? body->type : BodyType::Static;
}

bool World::IsAwake(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr && body->isAwake;
}

void* World::GetUserData(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->userData : nullptr;
}

void World::SetUserData(BodyHandle handle, void* userData) {
    if (!IsValid(handle)) {
        return;
    }
    GetBodyMutable(handle)->userData = userData;
}

// ---------------------------------------------------------------------------
// External forces / impulses
// ---------------------------------------------------------------------------

void World::ApplyForce(BodyHandle handle, Vec2 force, Vec2 worldPoint) {
    if (!IsValid(handle)) {
        return;
    }
    // Body::ApplyForce ignores non-dynamic bodies (infinite mass).
    GetBodyMutable(handle)->ApplyForce(force, worldPoint);
}

void World::ApplyLinearImpulse(BodyHandle handle, Vec2 impulse, Vec2 worldPoint) {
    if (!IsValid(handle)) {
        return;
    }
    GetBodyMutable(handle)->ApplyLinearImpulse(impulse, worldPoint);
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

void World::Step(float dt) {
    // Single source of truth for the iteration defaults: the stored settings.
    Step(dt, m_solverSettings.velocityIterations, m_solverSettings.positionIterations);
}

void World::Step(float dt, int32_t velocityIterations, int32_t positionIterations) {
    // A non-positive or non-finite dt has no physical meaning and would turn the
    // whole body array into NaN within one step. The caller owns the fixed-step
    // contract (Section 4.3), so this is a programmer error: assert in debug and
    // refuse to corrupt the state in release.
    assert(std::isfinite(dt) && dt > 0.0f && "World::Step: dt must be finite and > 0");
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return;
    }

    // The iteration counts only reach the solver phases that land in Week 3+.
    // They are already part of the frozen public API (testbed sliders and the
    // reserved pipeline documentation), so accept and discard them explicitly
    // rather than leave an unused-parameter warning in the build.
    (void)velocityIterations;
    (void)positionIterations;

    // Section 4.6 pipeline: the ORDER is normative. When filling the reserved
    // phases in later weeks, do not move a call - only replace its body.
    IntegrateForces(dt);        //  1. force/gravity -> velocity            [done]
    UpdateBroadPhase(dt);       //  2. broad phase update                   [reserved Week 6]
    UpdateContacts();           //  3. narrow phase / contact generation    [reserved Weeks 2-5]
    BuildIslands();             //  4. island construction                  [reserved Week 7]
    WarmStart();                //  5. warm start                           [reserved Week 5]
    SolveVelocityConstraints(); //  6. velocity iterations                  [reserved Weeks 3-5]
    IntegratePositions(dt);     //  7. velocity -> position (semi-implicit) [done]
    SolvePositionConstraints(); //  8. position iterations (split impulse)  [reserved Week 5]
    Synchronize();              //  9. transform/AABB synchronisation       [reserved Week 6]
    UpdateSleep(dt);            // 10. sleep evaluation                     [reserved Week 7]
    Validate();                 // 11. debug validation (NaN/Inf scan)      [done]
}

void World::SetBroadPhase(std::unique_ptr<IBroadPhase> broadPhase) {
    // World owns the object; nullptr explicitly disables broad-phase work, which
    // is the Week 1 state (no shapes, no proxies).
    m_broadPhase = std::move(broadPhase);
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

int32_t World::GetAwakeBodyCount() const {
    // Static bodies are born asleep, so in practice this counts dynamic and
    // kinematic bodies. It is an O(n) scan; only the debug UI calls it.
    int32_t awakeCount = 0;
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (m_bodyIds.IsValid(index) && m_bodies[static_cast<std::size_t>(index)].isAwake) {
            ++awakeCount;
        }
    }
    return awakeCount;
}

int32_t World::GetIslandCount() const noexcept {
    // RESERVED (Week 7): islands only exist once contacts do.
    return 0;
}

int32_t World::GetContactCount() const noexcept {
    // RESERVED (Weeks 2+): no contacts exist in Week 1.
    return 0;
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

void World::Clear() {
    m_bodyIds.Clear();
    m_bodies.clear();
    // RESERVED (Week 6): the broad phase will need its proxies cleared here once
    // IBroadPhase grows a reset entry point; until then it holds no state.
}

void World::CheckSane() const {
#if defined(PHYSICS2D_DEBUG)
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        const Body& body = m_bodies[static_cast<std::size_t>(index)];
        const bool transformOk =
            p2::IsValid(body.xf.p) && std::isfinite(body.xf.q.c) && std::isfinite(body.xf.q.s);
        const bool velocityOk = p2::IsValid(body.linearVelocity) && std::isfinite(body.angularVelocity);
        const bool accumulatorOk = p2::IsValid(body.force) && std::isfinite(body.torque);
        if (transformOk && velocityOk && accumulatorOk) {
            continue;
        }
        // Report before asserting: the abort itself carries no context, and
        // "which body, which field" is the most useful fact when a solver
        // explodes. This is the debug-only safety net of Section 3.4.
        std::fprintf(stderr,
                     "World::CheckSane: body %d ('%s') has non-finite state: pos=(%g, %g) "
                     "vel=(%g, %g) w=%g force=(%g, %g) torque=%g\n",
                     index, body.debugName.c_str(), static_cast<double>(body.xf.p.x),
                     static_cast<double>(body.xf.p.y), static_cast<double>(body.linearVelocity.x),
                     static_cast<double>(body.linearVelocity.y), static_cast<double>(body.angularVelocity),
                     static_cast<double>(body.force.x), static_cast<double>(body.force.y),
                     static_cast<double>(body.torque));
        assert(false && "World::CheckSane: non-finite body state (NaN/Inf)");
    }
#endif
}

// ---------------------------------------------------------------------------
// Step phases
// ---------------------------------------------------------------------------

void World::IntegrateForces(float dt) {
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        Body& body = m_bodies[static_cast<std::size_t>(index)];
        if (body.type != BodyType::Dynamic || !body.isAwake) {
            // Static/kinematic bodies have zero inverse mass, sleeping bodies
            // must keep their velocity: neither group may be accelerated.
            continue;
        }

        // Semi-implicit Euler velocity update (Section 4.2):
        //   v += dt * (g + F/m)
        //   w += dt * (tau / I)
        // Precomputed inverse mass turns the division into a multiplication.
        body.linearVelocity += (m_gravity + body.force * body.invMass) * dt;
        body.angularVelocity += body.torque * body.invInertia * dt;
    }

    // Forces are per-step quantities and are consumed exactly once. Clear every
    // live body (not only the awake ones) so a force applied to a sleeping body
    // cannot bank up and kick it when it wakes up in Week 7.
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (m_bodyIds.IsValid(index)) {
            m_bodies[static_cast<std::size_t>(index)].ClearForces();
        }
    }
}

void World::UpdateBroadPhase(float dt) {
    // RESERVED (Week 6): with a broad phase installed, update/insert the fat
    // AABB of every live body and enumerate candidate pairs into UpdateContacts.
    // Week 1 has no shape geometry, so there is nothing to insert.
    (void)dt;
}

void World::UpdateContacts() {
    // RESERVED (Weeks 2-5): run the narrow phase on the candidate pairs from the
    // broad phase, regenerate manifolds, and (Week 5) inherit per-point impulses
    // by ContactID for warm starting.
}

void World::BuildIslands() {
    // RESERVED (Week 7): DFS/union-find over touching dynamic bodies. Static
    // bodies act as island boundaries and are never island members.
}

void World::WarmStart() {
    // RESERVED (Week 5): apply accumulatedImpulse * warmStartFactor to every
    // persistent contact exactly once, before the velocity iterations begin.
}

void World::SolveVelocityConstraints() {
    // RESERVED (Weeks 3-5): velocityIterations passes of sequential impulses
    // (normal, friction and restitution constraints).
}

void World::IntegratePositions(float dt) {
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        Body& body = m_bodies[static_cast<std::size_t>(index)];
        if (body.type == BodyType::Static || !body.isAwake) {
            // Static bodies never move; kinematic and awake dynamic bodies do.
            continue;
        }

        // Semi-implicit (symplectic) Euler, Section 4.2: use the velocity that
        // IntegrateForces just produced, x_{n+1} = x_n + dt * v_{n+1}. Using the
        // old velocity would be explicit Euler, which gains energy every step.
        Vec2 translation = body.linearVelocity * dt;
        const float translationLength = Length(translation);
        if (translationLength > kMaxTranslation) {
            // Tunnelling defence (Section 6.6): a body may not translate further
            // than kMaxTranslation in one step; direction is preserved so a fast
            // body still flies along its velocity.
            translation = translation * (kMaxTranslation / translationLength);
        }
        body.xf.p += translation;

        // Integrate the angle through Rot composition instead of storing an
        // unwrapped angle: the incremental rotation dq = Rot(w * dt) is clamped
        // to kMaxRotation for the same tunnelling reason.
        const float deltaAngle = Clamp(body.angularVelocity * dt, -kMaxRotation, kMaxRotation);
        // 2D rotations commute, so composing the increment body-locally (q * dq)
        // and world-locally (dq * q) gives the same orientation. q * dq matches
        // Transform's "local -> world" convention.
        body.xf.q = Mul(body.xf.q, Rot(deltaAngle));
        // Long chains of Mul() accumulate rounding error; renormalise so the Rot
        // invariant c^2 + s^2 == 1 holds for the future broad phase and the
        // renderer (which reads the axes straight from the transform).
        body.xf.q.Normalize();
    }
}

void World::SolvePositionConstraints() {
    // RESERVED (Week 5): positionIterations passes of split-impulse position
    // correction. Positions change, velocities do not.
}

void World::Synchronize() {
    // RESERVED (Week 6): copy the final simulated state into the body transform
    // and refresh the broad-phase AABBs (the Week 4 hot/cold split writes the
    // state arrays back here as well).
}

void World::UpdateSleep(float dt) {
    // RESERVED (Week 7): accumulate sleepTime for slow bodies and put whole
    // islands to sleep after kSleepTimeThreshold seconds.
    (void)dt;
}

void World::Validate() const {
    // Phase 11, debug only: CheckSane compiles to an empty function outside
    // PHYSICS2D_DEBUG builds, so this phase costs nothing in release.
    CheckSane();
}

// ---------------------------------------------------------------------------
// Internal accessors
// ---------------------------------------------------------------------------

Body* World::GetBodyMutable(BodyHandle handle) {
    if (!IsValid(handle)) {
        assert(false && "World::GetBodyMutable: stale or invalid body handle");
        return nullptr;
    }
    return &m_bodies[static_cast<std::size_t>(handle.index)];
}

const Body* World::GetBodyChecked(BodyHandle handle) const {
    if (!IsValid(handle)) {
        assert(false && "World::GetBodyChecked: stale or invalid body handle");
        return nullptr;
    }
    return &m_bodies[static_cast<std::size_t>(handle.index)];
}

} // namespace p2
