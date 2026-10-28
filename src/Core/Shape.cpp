// p2::Shape — tagged-union dispatch for the shape family (Week 2, P05/P06).
//
// The individual shape math lives in the Collision module (ComputeCircleMass,
// ComputeCircleAABB, ...); this translation unit only dispatches over the tag.
// Keeping the dispatch in Core is what lets World's shape pool stay a plain
// std::vector<Shape> while the Collision module stays Core-agnostic.
//
// Week 2 implements Circle and Plane. The Polygon tag is reserved for Week 4
// (P04/P07): until Shape gains polygon storage every polygon-tagged value is
// invalid, so the dispatch below asserts instead of silently returning zeros.

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
        // Reserved: no polygon storage exists yet, so there is nothing that
        // could make a polygon-tagged shape valid in Week 2.
        return false;
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
        break;
    }
    assert(false && "ComputeShapeMass: PolygonShape mass properties arrive in Week 4 (P04)");
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
        break;
    }
    assert(false && "ComputeShapeAABB: PolygonShape collision arrives in Week 4 (P07)");
    return AABB{};
}

} // namespace p2
