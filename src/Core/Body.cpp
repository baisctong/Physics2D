// p2::Body — body parameters, mass derivation and force/impulse accumulation
// (Week 1 D6, P05; formulas fixed by Section 4.2).
//
// User-input validation belongs to World::CreateBody (it rejects a bad BodyDef
// by returning an invalid handle, Section 3.4). The asserts here only catch
// internal misuse, e.g. a future caller that constructs a Body directly.

#include "physics2d/Core/Body.h"

#include <cassert>
#include <cmath>

namespace p2 {

void Body::Initialize(const BodyDef& def) {
    // Preconditions: World::CreateBody has already rejected non-finite state.
    assert(IsValid(def.position) && "Body::Initialize: non-finite position");
    assert(std::isfinite(def.angle) && "Body::Initialize: non-finite angle");
    assert(IsValid(def.linearVelocity) && "Body::Initialize: non-finite velocity");
    assert(std::isfinite(def.angularVelocity) && "Body::Initialize: non-finite angular velocity");
    assert(std::isfinite(def.density) && std::isfinite(def.friction) && std::isfinite(def.restitution) &&
           "Body::Initialize: non-finite material property");

    // ---- cold, user-facing parameters -----------------------------------
    type = def.type;
    shape = ShapeHandle{}; // reserved: no shape storage before Week 2
    density = def.density;
    friction = def.friction;
    restitution = def.restitution;
    allowSleep = def.allowSleep;
    isSensor = def.isSensor;
    userData = def.userData;
    debugName = def.debugName;

    // ---- simulated state -------------------------------------------------
    xf.p = def.position;
    xf.q.Set(def.angle);
    linearVelocity = def.linearVelocity;
    angularVelocity = def.angularVelocity;

    // Slots are recycled by IDPool, so every mutable field must be reset here:
    // a new body must never inherit per-step accumulators from its predecessor.
    force = Vec2{0.0f, 0.0f};
    torque = 0.0f;
    sleepTime = 0.0f;
    islandId = -1;
    aabb = AABB{};

    // ---- mass properties -------------------------------------------------
    switch (type) {
    case BodyType::Static:
    case BodyType::Kinematic:
        // Infinite mass is represented by zero inverse mass, never by inf:
        // multiplying by 0 stays finite and branch-free in the integrator
        // and (from Week 3) the solver, whereas inf * 0 produces NaN.
        mass = 0.0f;
        inertia = 0.0f;
        invMass = 0.0f;
        invInertia = 0.0f;
        // Static bodies are born asleep. Kinematic bodies move by their
        // velocity, so they must stay in the simulation (but ignore forces).
        isAwake = (type == BodyType::Kinematic);
        break;

    case BodyType::Dynamic:
        assert(std::isfinite(def.mass) && def.mass > 0.0f &&
               "Body::Initialize: dynamic mass must be finite and > 0");
        assert(std::isfinite(def.inertia) && def.inertia > 0.0f &&
               "Body::Initialize: dynamic inertia must be finite and > 0");
        mass = def.mass;
        inertia = def.inertia;
        invMass = 1.0f / mass;
        // fixedRotation is a dynamic-body option: it freezes the angular
        // degree of freedom without turning the body into a kinematic one.
        invInertia = def.fixedRotation ? 0.0f : (1.0f / inertia);
        isAwake = true;
        break;
    }
}

void Body::ApplyForce(Vec2 forceWorld, Vec2 worldPoint) {
    // Infinite-mass bodies cannot accelerate, so their force accumulator is
    // meaningless. Ignoring the call keeps stale forces out of the World.
    if (!IsDynamic()) {
        return;
    }
    assert(IsValid(forceWorld) && "Body::ApplyForce: non-finite force");
    assert(IsValid(worldPoint) && "Body::ApplyForce: non-finite application point");

    force += forceWorld;
    // tau = r x F, with r measured from the body origin (the centre of mass in
    // Week 1; Week 2 shape mass data adds a centre-of-mass offset). Section 4.2:
    // positive z is counter-clockwise, matching the Rot angle convention.
    torque += Cross(worldPoint - xf.p, forceWorld);
}

void Body::ApplyLinearImpulse(Vec2 impulse, Vec2 worldPoint) {
    if (!IsDynamic()) {
        return;
    }
    assert(IsValid(impulse) && "Body::ApplyLinearImpulse: non-finite impulse");
    assert(IsValid(worldPoint) && "Body::ApplyLinearImpulse: non-finite application point");

    // Section 4.2 sign convention, written once, used everywhere:
    //   dv = J / m
    //   dw = invI * Cross(r, J)
    // The Cross(r, J) order matters: Cross(J, r) would spin the body backwards.
    linearVelocity += impulse * invMass;
    angularVelocity += invInertia * Cross(worldPoint - xf.p, impulse);
}

} // namespace p2
