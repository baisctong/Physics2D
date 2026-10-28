#pragma once

// p2::ShapeType / p2::MassData — the small shared vocabulary of the collision
// module (Week 2, P06/P04).
//
// Why a dedicated header: both shape types (Circle in Week 2, Polygon in Week 4)
// and the World's shape storage need the same enum and the same mass result, and
// that result must not depend on Core (dependency direction: Core -> Collision).

#include "physics2d/Math/Vec2.h"

#include <cstdint>

namespace p2 {

enum class ShapeType : uint8_t {
    Circle = 0, // Week 2
    Plane = 1,  // Week 2: infinite static half-space (ground)
    Polygon = 2 // reserved (Week 4, P04/P07)
};

// Result of integrating a shape's area/density. The `center` is expressed in the
// body-local frame and `inertia` is taken about that center (parallel-axis
// transfer to another point is the caller's job - World::AttachShape does it
// once when the shape is attached).
struct MassData {
    float mass = 0.0f;       // kg (density already applied)
    Vec2 center{0.0f, 0.0f}; // local centre of mass
    float inertia = 0.0f;    // kg*m^2 about `center`
};

} // namespace p2
