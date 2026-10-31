#pragma once

// p2::BodyType / p2::BodyDef — creation parameters for a rigid body
// (Section 3.3, P05).
//
// BodyDef is the user-facing struct: it only contains data, is safe to build
// on the stack and is copied into the body by World::CreateBody. Validation
// happens in CreateBody, never here, so a partially filled BodyDef is a legal
// value (that is why the fields have safe defaults).

#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cstdint>
#include <string>

namespace p2 {

enum class BodyType : uint8_t {
    Static = 0,    // never moves, invMass = invInertia = 0, not affected by gravity
    Kinematic = 1, // moves by its velocity only, invMass = invInertia = 0, ignores forces
    Dynamic = 2,   // fully simulated
};

struct BodyDef {
    BodyType type = BodyType::Static;

    // Initial state.
    Vec2 position{0.0f, 0.0f};
    float angle = 0.0f;
    Vec2 linearVelocity{0.0f, 0.0f};
    float angularVelocity = 0.0f;

    // Mass properties.
    // Week 1 has no shapes yet, so the mass is given explicitly here. From
    // Week 2 onwards, attaching a shape recomputes mass/inertia from `density`;
    // these fields then act as the fallback for bodies without a shape.
    float mass = 1.0f;    // kg, must be > 0 for dynamic bodies
    float inertia = 1.0f; // kg*m^2, must be > 0 for dynamic bodies
    float density = 1.0f; // kg/m^2, used once shapes exist (Week 2+)

    // Surface properties (combined pairwise in the solver, Week 5).
    float friction = 0.3f;
    float restitution = 0.0f;

    // Lifecycle / filtering.
    bool allowSleep = true;     // reserved: honoured by the island sleeper (Week 7)
    bool isSensor = false;      // reserved: sensor contacts generate no impulses (Week 5)
    bool fixedRotation = false; // reserved: zeroes invInertia even for dynamic bodies

    // User payload: the engine never dereferences this. `debugName` is for the
    // testbed panels and regression output only.
    void* userData = nullptr;
    std::string debugName;
};

} // namespace p2
