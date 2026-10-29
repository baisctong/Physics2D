#pragma once

// p2::Body — one rigid body (Section 3.3, P05).
//
// Layout note: the fields are grouped into "cold" (organisational, touched by
// the user/UI once per frame) and "hot" (touched by the solver every iteration)
// exactly as the design document describes. Weeks 1-3 keep a single struct for
// simplicity; the planned Week 4 refactor splits the hot block into
// `BodySim` / `BodyState` parallel arrays without changing the public API
// (World's accessor functions are the stable boundary - see docs/dev-interfaces.md).
//
// The body does not own geometry: it stores a ShapeHandle into World's shape
// pool (Week 2). World::AttachShape derives mass/inertia from the shape and the
// body's density; `fixedRotation` keeps the angular inverse mass at zero.

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cstdint>
#include <string>

namespace p2 {

struct Body {
    // ---------------------------------------------------------------------
    // Cold data: organisational state, rarely touched during simulation.
    // ---------------------------------------------------------------------
    BodyHandle handle;
    BodyType type = BodyType::Static;
    ShapeHandle shape; // Week 2: handle into World's shape storage

    float density = 1.0f;
    float friction = 0.3f;
    float restitution = 0.0f;

    bool allowSleep = true;
    bool isAwake = true; // static bodies are created asleep, kinematic/dynamic awake
    bool isSensor = false;
    bool fixedRotation = false; // forces invInertia = 0 (pinned rotation)

    void* userData = nullptr; // opaque to the engine
    std::string debugName;    // testbed / regression output only

    // ---------------------------------------------------------------------
    // Hot data: written by the integrator and (from Week 3) the solver.
    // This block becomes BodySim + BodyState in the Week 4 refactor.
    // ---------------------------------------------------------------------
    Transform xf; // body origin transform (position + rotation)
    Vec2 linearVelocity{0.0f, 0.0f};
    float angularVelocity = 0.0f;

    // WEEK 5 (split impulse): pseudo-velocities owned by the position-correction
    // layer only. The velocity solver never reads them and the position layer
    // never writes the real velocities above: after the pseudo-velocity
    // iterations these are integrated into `xf` once and then zeroed, so the
    // correction removes penetration without injecting energy into the real
    // velocity state (Section 6.4). Appended to the hot block - the Week 3
    // velocity-solver contract is unchanged.
    Vec2 pseudoLinearVelocity{0.0f, 0.0f};
    float pseudoAngularVelocity = 0.0f;

    Vec2 force{0.0f, 0.0f}; // accumulated external force, cleared every step
    float torque = 0.0f;

    float mass = 0.0f;
    float invMass = 0.0f;
    float inertia = 0.0f;
    float invInertia = 0.0f;

    float sleepTime = 0.0f;
    int32_t islandId = -1; // reserved (Week 7)

    AABB aabb; // fat AABB (tight bounds + kAabbMargin), refreshed by World::Synchronize

    // Week 6: broad-phase proxy id owned by the installed IBroadPhase, -1 when the
    // body has no proxy. World creates/updates/releases it around
    // UpdateBroadPhase/DestroyBody; the body itself never talks to the broad phase.
    // Appended to the hot block - the Week 5 velocity-solver contract is unchanged.
    int32_t proxyId = -1;

    // ---------------------------------------------------------------------
    // Operations
    // ---------------------------------------------------------------------

    // Copies user parameters and derives mass properties. Called only by
    // World::CreateBody. `def` has already been validated there.
    void Initialize(const BodyDef& def);

    [[nodiscard]] Vec2 GetPosition() const noexcept { return xf.p; }
    [[nodiscard]] float GetAngle() const noexcept { return xf.q.GetAngle(); }
    [[nodiscard]] bool IsDynamic() const noexcept { return type == BodyType::Dynamic; }

    // Accumulate a force applied at a world-space point. The torque is
    // computed here so that the integrator only has to read `force`/`torque`.
    void ApplyForce(Vec2 forceWorld, Vec2 worldPoint);

    // Instantaneously change linear/angular velocity. `worldPoint` is the
    // application point; the lever arm r = worldPoint - position creates the
    // angular impulse Cross(r, impulse) / I. (Week 3 warm-up API, used by the
    // testbed's "kick the stack" interactions.)
    void ApplyLinearImpulse(Vec2 impulse, Vec2 worldPoint);

    // Called by World after the integration phase.
    void ClearForces() noexcept {
        force = {0.0f, 0.0f};
        torque = 0.0f;
    }
};

} // namespace p2
