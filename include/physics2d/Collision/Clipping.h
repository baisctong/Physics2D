#pragma once

// p2::Clipping — the pure 2D line-clipping primitive used by SAT manifold
// generation (Week 4 D4 / P07).
//
// Box2D-style clipping is Sutherland-Hodgman applied to the *incident edge*: a
// segment (two endpoints) is clipped against the side planes of the reference face
// twice, then against the reference face plane itself. Keeping the primitive pure
// (no manifolds, no ContactID, no transforms) is what makes it unit-testable and
// reusable by the circle-polygon path if that ever needs it.
//
// Half-plane convention: a point is kept when Dot(normal, point) <= offset.
// The reference face plane uses the outward normal, so "kept" means the point is
// behind the face (penetrating); the side planes keep the segment between the two
// reference-face endpoints.

#include "physics2d/Math/Vec2.h"

#include <cstdint>

namespace p2 {

// Clips the segment [input[0], input[1]] to the half-plane Dot(normal, x) <= offset.
// Returns the number of surviving vertices (0, 1 or 2) written to `output`.
// When exactly one endpoint is inside, the other is replaced by the intersection
// point. The normal is expected to be unit length (SAT passes unit edge normals).
[[nodiscard]] int32_t ClipSegmentToLine(const Vec2 input[2], Vec2 output[2], Vec2 normal,
                                        float offset) noexcept;

} // namespace p2
