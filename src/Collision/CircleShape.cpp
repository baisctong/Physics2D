// p2::CircleShape — analytic circle geometry helpers (Week 2 D2/D4, P06).
//
// Circle mass properties are closed-form, so this file is the shape half of the
// "circles never go through SAT" rule (Section 6.3): no triangulation, no
// numerical integration, no accumulated rounding. The collision half lives in
// Analytic.cpp.

#include "physics2d/Collision/CircleShape.h"

#include "physics2d/Math/MathUtils.h"

#include <cassert>
#include <cmath>

namespace p2 {

bool CircleShape::IsValid() const noexcept {
    // A circle must have a strictly positive, finite radius (a zero radius
    // would make every rotation about the body origin degenerate: the mass
    // properties below divide by the radius) and a finite local centre.
    if (!std::isfinite(radius) || radius <= 0.0f) {
        return false;
    }
    // Fully qualified: inside a member function the unqualified name IsValid
    // would resolve to this member (0-argument overload) and hide the
    // p2::IsValid(Vec2) free function from Vec2.h.
    return p2::IsValid(localCenter);
}

MassData ComputeCircleMass(const CircleShape& shape, float density) noexcept {
    // Preconditions come from CircleShape.h. Asserting here (and not repairing)
    // follows Section 3.4: bad shapes are a caller bug, and silently replacing
    // them with a default would hide it until the simulation misbehaves.
    assert(shape.IsValid() && "ComputeCircleMass: invalid CircleShape");
    assert(std::isfinite(density) && density > 0.0f && "ComputeCircleMass: density must be > 0");

    // Solid disc of radius r and area density rho (Section 6.1):
    //   area    A = pi * r^2
    //   mass    m = rho * A
    //   inertia I = (1/2) * m * r^2        about the disc's own centre
    // The multiplication order matches the hand-computed test values.
    const float area = kPi * shape.radius * shape.radius;
    const float mass = density * area;
    return MassData{mass, shape.localCenter, 0.5f * mass * shape.radius * shape.radius};
}

AABB ComputeCircleAABB(const CircleShape& shape, const Transform& xf) noexcept {
    assert(shape.IsValid() && "ComputeCircleAABB: invalid CircleShape");

    // Rotating a circle about its own centre does not change its shape, so only
    // the centre has to be transformed: the bounds are the world centre plus or
    // minus the radius on each axis.
    const Vec2 worldCenter = Mul(xf, shape.localCenter);
    const Vec2 extent{shape.radius, shape.radius};
    return AABB{worldCenter - extent, worldCenter + extent};
}

} // namespace p2
