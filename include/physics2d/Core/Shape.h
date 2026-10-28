#pragma once

// p2::Shape — tagged union of the colliders a body can carry (Week 2, P05/P06).
//
// Week 2 supports Circle and Plane. The Polygon member arrives with P04/P07 in
// Week 4 and will be appended here without changing anything else, which is why
// this struct is the storage element of World's shape pool rather than a class
// hierarchy: a value type keeps the shape pool a plain std::vector (cache
// friendly, deterministic iteration order, no virtual dispatch in the narrow
// phase).
//
// Mass/AABB helpers live in Core because they dispatch over the shape family;
// each individual shape's math is implemented in the Collision module.

#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Transform.h"

namespace p2 {

struct Shape {
    ShapeType type = ShapeType::Circle;

    // Only the member matching `type` is meaningful. A tagged union with three
    // small values is simpler and safer than std::variant here (no exception
    // path, trivially copyable, debuggable in a watch window).
    CircleShape circle;
    PlaneShape plane;
    PolygonShape polygon; // Week 4

    [[nodiscard]] bool IsValid() const noexcept;
};

// Mass properties of the shape with a uniform density.
// Precondition: shape.IsValid().
[[nodiscard]] MassData ComputeShapeMass(const Shape& shape, float density) noexcept;

// World-space AABB of the shape under `xf`.
[[nodiscard]] AABB ComputeShapeAABB(const Shape& shape, const Transform& xf) noexcept;

} // namespace p2
