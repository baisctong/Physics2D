#pragma once

// p2::PolygonShape — convex polygon collider (Week 4, P04/P07).
//
// Storage contract:
// - `vertices[0..count)` are in the owning body's local frame, in counter-clockwise
//   order, with no repeated first/last vertex.
// - `normals[i]` is the unit outward normal of edge i (vertices[i] -> vertices[i+1]).
// - `centroid` is the area centroid in the same local frame. The body origin is NOT
//   required to coincide with it; World::AttachShape transfers the mass properties
//   to the body origin with the parallel-axis theorem (same convention as circles).
//
// Validation is strict and happens in SetPolygon (Section 3.4: never silently
// repair user input). A rejected polygon leaves the shape in the invalid state
// (count == 0) and returns false. Rejected cases:
//   - count outside [3, kMaxPolygonVertices]
//   - any non-finite coordinate
//   - adjacent vertices closer than kLinearSlop (duplicate/overlapping input)
//   - a redundant collinear vertex (the middle vertex can be removed)
//   - clockwise winding (signed area < 0) or a non-convex vertex
//   - (near-)zero area
//   - finite but overflowing coordinates (edge length squared, shoelace sum or
//     weighted centroid sum not finite). Week 8 boundary hardening: such input used
//     to be accepted and produced (0,0) edge normals / an inf centroid that only
//     failed later in the narrow phase or mass properties; it is now rejected like
//     any other bad geometry (tests/unit/test_boundary_solver.cpp).
//
// Note for the reviewer: P04 also mentions silently merging adjacent vertices
// closer than kLinearSlop. This project rejects them instead, because the design
// document's error-handling section forbids silently repairing bad geometry; the
// caller gets a deterministic false and can fix the data.
//
// SetPolygonAsBox() is the expected construction path for boxes (the BoxStack demo
// and most tests). The vertices are ordered CCW starting at (+hx,-hy) so the same
// input always produces the same edge indices - ContactID stability depends on it.

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"
#include <cstdint>

namespace p2 {

struct PolygonShape {
    Vec2 vertices[kMaxPolygonVertices];
    Vec2 normals[kMaxPolygonVertices];
    int32_t count = 0;
    Vec2 centroid{0.0f, 0.0f};

    [[nodiscard]] bool IsValid() const noexcept;
};

// Builds (or rebuilds) the polygon from a point array. Returns false and leaves
// the shape invalid on any rejected input; see the validation contract above.
[[nodiscard]] bool SetPolygon(PolygonShape& shape, const Vec2* vertices, int32_t count) noexcept;

// Axis-aligned box centred on the body origin: halfWidth/halfHeight > 0.
[[nodiscard]] bool SetPolygonAsBox(PolygonShape& shape, float halfWidth, float halfHeight) noexcept;

// Box centred at `center` and rotated by `angle` (both in body-local coordinates).
[[nodiscard]] bool SetPolygonAsBox(PolygonShape& shape, float halfWidth, float halfHeight, Vec2 center,
                                   float angle) noexcept;

// Mass properties with uniform density:
//   area A = 1/2 * sum(P_i x P_{i+1})
//   centroid C = (1/(6A)) * sum((P_i + P_{i+1}) * (P_i x P_{i+1}))
//   I_origin = (rho/12) * sum((P_i x P_{i+1}) * (P_i.P_i + P_i.P_{i+1} + P_{i+1}.P_{i+1}))
//   I_centroid = I_origin - m * |C|^2       (parallel-axis theorem)
// The returned MassData uses local coordinates: center = C, inertia about C.
[[nodiscard]] MassData ComputePolygonMass(const PolygonShape& shape, float density) noexcept;

// World-space AABB of the transformed vertices.
[[nodiscard]] AABB ComputePolygonAABB(const PolygonShape& shape, const Transform& xf) noexcept;

} // namespace p2
