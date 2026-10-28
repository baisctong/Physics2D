// p2::Shape — tagged-union dispatch for the shape family (Week 2/Week 4).
//
// The individual shape math lives in the Collision module (ComputeCircleMass,
// ComputeCircleAABB, ComputePolygonMass, ...); this translation unit only
// dispatches over the tag. Keeping the dispatch in Core is what lets World's
// shape pool stay a plain std::vector<Shape> while the Collision module stays
// Core-agnostic.
//
// Week 4 fills in the Polygon tag: geometry is validated by SetPolygon /
// SetPolygonAsBox when the shape is created, so the dispatch below can trust the
// stored vertices and simply forward to the Collision helpers.

#include "physics2d/Core/Shape.h"

#include <cassert>

namespace p2 {

bool Shape::IsValid() const noexcept {
    switch (type) {
    case ShapeType::Circle:
        return circle.IsValid();
    case ShapeType::Plane:
        return plane.IsValid();
    case ShapeType::Polygon:
        return polygon.IsValid();
    }
    return false;
}

MassData ComputeShapeMass(const Shape& shape, float density) noexcept {
    assert(shape.IsValid() && "ComputeShapeMass: shape must be valid");
    switch (shape.type) {
    case ShapeType::Circle:
        return ComputeCircleMass(shape.circle, density);
    case ShapeType::Plane:
        return ComputePlaneMass(shape.plane, density);
    case ShapeType::Polygon:
        return ComputePolygonMass(shape.polygon, density);
    }
    assert(false && "ComputeShapeMass: unknown ShapeType");
    return MassData{};
}

AABB ComputeShapeAABB(const Shape& shape, const Transform& xf) noexcept {
    assert(shape.IsValid() && "ComputeShapeAABB: shape must be valid");
    switch (shape.type) {
    case ShapeType::Circle:
        return ComputeCircleAABB(shape.circle, xf);
    case ShapeType::Plane:
        return ComputePlaneAABB(shape.plane, xf);
    case ShapeType::Polygon:
        return ComputePolygonAABB(shape.polygon, xf);
    }
    assert(false && "ComputeShapeAABB: unknown ShapeType");
    return AABB{};
}

} // namespace p2
