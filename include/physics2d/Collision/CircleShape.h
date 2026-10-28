#pragma once

// p2::CircleShape — the first real collider (Week 2, P06).
//
// A circle is special among the shapes: its collision with other circles and
// with a plane is solved analytically (distance vs radius sum), never through
// SAT/clipping. That is why these paths appear a full design phase before the
// polygon machinery (Week 4) and why the design document says "Circle-Circle /
// Circle-Polygon / Circle-Plane 走专门解析路径" (Section 6.3).
//
// Geometry conventions:
// - `localCenter` is expressed in the owning body's local frame and defaults to
//   the body origin. A body may therefore rotate about a point that is not the
//   circle centre; the mass properties account for that via the parallel-axis
//   theorem (World::AttachShape).
// - `radius` is in metres (MKS, Section 4.1). Only a strictly positive, finite
//   radius is valid; the engine never silently repairs a bad shape (Section 3.4).
//
// Reserved extension (Week 4): the same analytic pattern will gain
// CollideCircleAndPolygon (nearest edge / vertex Voronoi regions, P06).

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

namespace p2 {

struct CircleShape {
    float radius = 0.5f;          // m, must be > 0
    Vec2 localCenter{0.0f, 0.0f}; // centre in body-local coordinates

    [[nodiscard]] bool IsValid() const noexcept;
};

// Mass properties with a uniform density (kg/m^2):
//   mass    = density * pi * r^2
//   center  = localCenter
//   inertia = 0.5 * mass * r^2        (solid disc, about its own centre)
// Preconditions: shape.IsValid(), density finite and > 0 (asserted in debug).
[[nodiscard]] MassData ComputeCircleMass(const CircleShape& shape, float density) noexcept;

// Axis-aligned bounds of the circle under `xf`.
[[nodiscard]] AABB ComputeCircleAABB(const CircleShape& shape, const Transform& xf) noexcept;

} // namespace p2
