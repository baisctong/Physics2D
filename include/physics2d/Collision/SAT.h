#pragma once

// p2::SAT + manifold generation for convex polygons (Week 4, P07).
//
// Two convex polygons in 2D are separated iff there is a separating axis among the
// edge normals of the two shapes (4 candidate axes for two boxes). SAT finds the
// axis with the smallest overlap; that axis becomes the contact normal, and the
// face that owns it becomes the *reference face*. The other polygon contributes the
// *incident face* (the face most anti-parallel to the normal), which is clipped
// against the reference face by Sutherland-Hodgman line clipping to obtain one or
// two contact points (Section 6.3).
//
// Conventions (normative, Section 4.2):
// - Manifold.normal points from polygon A toward polygon B.
// - A negative separation means overlap; a positive separation means the pair is
//   apart and CollidePolygons returns an empty manifold.
// - Colliding with swapped A/B must produce the flipped manifold: normal negated,
//   same penetration, and the SAME ContactID::Key() set (flip is metadata, not
//   identity) - see ContactID.h and the Week 4 tests.
//
// SAT cache (Week 4 D6): when several axes are within a relative epsilon of the
// best separation, the previous frame's reference face is preferred. This is
// hysteresis only: it keeps the manifold (and therefore the ContactIDs) stable on
// resting stacks without changing the physical result. Pass nullptr to disable it.
//
// Callers get best performance by preparing transforms/vertices once (World does
// this per step); the functions are pure and do not retain global state.

#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Collision/SATCache.h"
#include "physics2d/Math/Transform.h"

#include <cstdint>

namespace p2 {

// Result of projecting one polygon onto the edge normals of another.
struct SeparationResult {
    int32_t edgeIndex = 0;   // edge of `polygon` whose normal gives the separation
    float separation = 0.0f; // negative when the projections overlap
};

// For every edge of `polygon`, project `other` onto the edge normal and keep the
// axis of maximum separation (P07 D2). Returns the edge index on `polygon` and the
// signed separation along it: > 0 means the pair is separated.
[[nodiscard]] SeparationResult FindMaxSeparation(const PolygonShape& polygon, const Transform& xfPolygon,
                                                 const PolygonShape& other,
                                                 const Transform& xfOther) noexcept;

// Full SAT + clipping manifold for polygon A vs polygon B.
// `cache` is optional in/out hysteresis state (see SATCache.h); it is updated with
// the chosen reference face for the next frame.
[[nodiscard]] Manifold CollidePolygons(const PolygonShape& polygonA, const Transform& xfA,
                                       const PolygonShape& polygonB, const Transform& xfB,
                                       SATCache* cache = nullptr) noexcept;

} // namespace p2
