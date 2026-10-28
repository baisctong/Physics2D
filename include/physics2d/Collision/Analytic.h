#pragma once

// p2::Analytic — closed-form collision routines for the circle family
// (Week 2 D2/D4, P06).
//
// Normal convention (Section 4.2, normative): `Manifold.normal` points from
// shape A toward shape B. The function name says which shape is A; swapping the
// arguments through the *And* overload produces the flipped manifold, and
// FlipManifold() implements that flip exactly (normal reversed, anchor points
// swapped, flip bit toggled) so callers never re-derive it.
//
// Separation/touching contract (Section 6.3):
//   - separated:      pointCount == 0, manifold otherwise unspecified but zeroed
//   - exactly tangent: pointCount == 1, penetration == 0
//   - overlapping:    pointCount == 1, penetration > 0
// Circle contacts never produce two points; two-point manifolds arrive with
// face-face clipping in Week 4.
//
// Determinism note: when two circle centres coincide there is no geometric
// direction for the normal. Instead of returning NaN we pick the fixed axis
// (+x) so that the same scene always produces the same manifold (Section 7.4).

#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Collision/PolygonShape.h"
#include "physics2d/Math/Transform.h"

namespace p2 {

// Circle A vs circle B.
[[nodiscard]] Manifold CollideCircles(const CircleShape& circleA, const Transform& xfA,
                                      const CircleShape& circleB, const Transform& xfB) noexcept;

// Circle A vs infinite plane B. The normal points from the circle toward the
// plane surface (i.e. -planeWorldNormal for a circle above a ground plane).
[[nodiscard]] Manifold CollideCircleAndPlane(const CircleShape& circle, const Transform& xfCircle,
                                             const PlaneShape& plane, const Transform& xfPlane) noexcept;

// Plane A vs circle B: identical geometry with the manifold flipped.
[[nodiscard]] Manifold CollidePlaneAndCircle(const PlaneShape& plane, const Transform& xfPlane,
                                             const CircleShape& circle, const Transform& xfCircle) noexcept;

// ---------------------------------------------------------------------------
// Week 4 analytic paths: circle vs convex polygon and polygon vs infinite plane.
// These stay analytic (nearest feature / vertex-depth + face clipping) instead of
// going through SAT, per Section 6.3 ("圆相关走专门解析路径").
// ---------------------------------------------------------------------------

// Circle A vs polygon B. The polygon's closest feature (edge Voronoi region or
// vertex Voronoi region) determines the normal; ContactID identifies that feature
// (see ContactID.h) so the key is stable while the feature does not change.
[[nodiscard]] Manifold CollideCircleAndPolygon(const CircleShape& circle, const Transform& xfCircle,
                                               const PolygonShape& polygon,
                                               const Transform& xfPolygon) noexcept;

// Polygon A vs circle B: identical geometry with the manifold flipped.
[[nodiscard]] Manifold CollidePolygonAndCircle(const PolygonShape& polygon, const Transform& xfPolygon,
                                               const CircleShape& circle, const Transform& xfCircle) noexcept;

// Polygon A vs infinite plane B. Produces up to two contact points (all vertices
// behind the plane; the BoxStack demo needs a stable two-point support for a box
// resting on the ground), each with its polygon vertex index in the ContactID.
[[nodiscard]] Manifold CollidePolygonAndPlane(const PolygonShape& polygon, const Transform& xfPolygon,
                                              const PlaneShape& plane, const Transform& xfPlane) noexcept;

// Plane A vs polygon B: identical geometry with the manifold flipped.
[[nodiscard]] Manifold CollidePlaneAndPolygon(const PlaneShape& plane, const Transform& xfPlane,
                                              const PolygonShape& polygon,
                                              const Transform& xfPolygon) noexcept;

// Reverse a manifold: negate the normal, swap localPointA/localPointB on every
// contact point and toggle the ContactID flip bit. Penetration and pointCount
// are unchanged. Week 4's SAT path reuses this when normalising A/B order.
[[nodiscard]] Manifold FlipManifold(const Manifold& manifold) noexcept;

} // namespace p2
