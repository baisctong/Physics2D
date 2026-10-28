#pragma once

// p2::PlaneShape — infinite static half-space (Week 2, P06).
//
// The first scene of the project needs a ground. A dedicated plane type is
// cheaper and numerically much better than "a huge static polygon": the signed
// distance from a point to the plane is a dot product, no clipping, no
// degenerate long-thin geometry, and the result is exactly stable for any
// position (design doc Week 2 D2: "无限地面 + 解析式穿透检测").
//
// Definition (body-local frame):
//
//     plane surface : { x | Dot(normal, x) == offset }
//     solid region  : { x | Dot(normal, x) <  offset }
//     free space    : { x | Dot(normal, x) >  offset }
//
// `normal` is unit length and points OUT of the solid, i.e. toward free space
// (for a ground plane in a +y-up world: normal = (0,1), offset = 0). A plane is
// an infinite mass shape: it may only be attached to a Static body, and its
// mass data is zero with invMass = invInertia = 0 (Section 4.2 / P05).
//
// World-space evaluation only needs a rotation and a point on the plane:
//     worldNormal = Rot(xf.q) * normal
//     worldOffset = offset + Dot(worldNormal, xf.p)

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

namespace p2 {

struct PlaneShape {
    Vec2 normal{0.0f, 1.0f}; // unit vector, points toward free space
    float offset = 0.0f;     // signed distance from the body origin to the surface

    [[nodiscard]] bool IsValid() const noexcept;
};

// Infinite mass shape: mass = 0, invMass = invInertia = 0. The density argument
// exists only so that the shape dispatch in Shape can stay uniform.
[[nodiscard]] MassData ComputePlaneMass(const PlaneShape& shape, float density) noexcept;

// A deliberately enormous box that always overlaps everything. The real broad
// phase (Week 6) will special-case planes; until then this keeps the naive
// AABB filter honest instead of silently dropping the ground.
[[nodiscard]] AABB ComputePlaneAABB(const PlaneShape& shape, const Transform& xf) noexcept;

[[nodiscard]] Vec2 ComputePlaneWorldNormal(const PlaneShape& shape, const Transform& xf) noexcept;
[[nodiscard]] float ComputePlaneWorldOffset(const PlaneShape& shape, const Transform& xf) noexcept;

} // namespace p2
