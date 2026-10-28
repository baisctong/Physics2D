// p2::Analytic — closed-form collision routines for the circle and polygon
// families (Week 2 D2/D4 + Week 4 D1, P06/P04).
//
// These paths exist because a circle's closest-feature query is trivial (no
// edges: the normal is the centre-to-centre or centre-to-nearest-feature
// direction), and because polygon-vs-plane only needs per-vertex signed
// distances. Neither case needs SAT/clipping, which is why they are written
// before (circle family) and alongside (polygon-plane) the SAT machinery
// (Section 6.3: "圆相关走专门解析路径").

#include "physics2d/Collision/Analytic.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace p2 {
namespace {

// Distance below which the centre-to-centre direction is treated as undefined.
// One micrometre is far below any contact scale the engine cares about
// (kLinearSlop is 5 mm), yet far above the float resolution of positions in a
// 1e3 m scene, so the fallback below only fires for genuinely coincident
// centres instead of jittering ordinary small overlaps.
constexpr float kCoincidentCenterDistance = kEpsilon;

// Empty manifold: pointCount = 0 and a zeroed penetration. The normal is left at
// its default; the header documents the rest of the manifold as unspecified when
// pointCount == 0, so callers must only look at pointCount.
[[nodiscard]] Manifold MakeEmptyManifold() noexcept {
    Manifold manifold;
    manifold.Clear();
    return manifold;
}

// Fill one contact point from a world-space anchor. Storing the anchor in both
// bodies' local frames guarantees the solver invariant
//     Mul(xfA, localPointA) == Mul(xfB, localPointB)
// up to float rounding, without either shape having to know about the other.
[[nodiscard]] ContactPoint MakeContactPoint(const Transform& xfA, const Transform& xfB, Vec2 worldPoint,
                                            ContactID id) noexcept {
    ContactPoint point;
    point.localPointA = MulT(xfA, worldPoint);
    point.localPointB = MulT(xfB, worldPoint);
    point.id = id;
    return point;
}

// One-point manifold used by every analytic path. `id` carries the feature
// encoding (type plus polygon feature indices once Week 4's shapes appear).
[[nodiscard]] Manifold MakePointManifold(Vec2 normal, float penetration, Vec2 worldPoint,
                                         const Transform& xfA, const Transform& xfB, ContactID id) noexcept {
    Manifold manifold;
    manifold.Clear();
    manifold.normal = normal;
    manifold.penetration = penetration;
    manifold.pointCount = 1;
    manifold.points[0] = MakeContactPoint(xfA, xfB, worldPoint, id);
    return manifold;
}

// Vertex i is shared by the incoming edge (i + count - 1) % count and the
// outgoing edge i. For a polygon-plane contact the feature edge is the one whose
// outward normal points most towards the plane (largest dot with the local
// polygon -> plane direction); ties go to the incoming edge so the choice is
// deterministic. This keeps referenceEdge stable while the same face touches.
[[nodiscard]] int32_t FindSupportingEdge(const PolygonShape& polygon, int32_t vertexIndex,
                                         Vec2 localPlaneDirection) noexcept {
    const int32_t count = polygon.count;
    const int32_t incomingEdge = (vertexIndex + count - 1) % count;
    const int32_t outgoingEdge = vertexIndex;
    const float incomingAlignment = Dot(polygon.normals[incomingEdge], localPlaneDirection);
    const float outgoingAlignment = Dot(polygon.normals[outgoingEdge], localPlaneDirection);
    return outgoingAlignment > incomingAlignment ? outgoingEdge : incomingEdge;
}

} // namespace

Manifold CollideCircles(const CircleShape& circleA, const Transform& xfA, const CircleShape& circleB,
                        const Transform& xfB) noexcept {
    assert(circleA.IsValid() && "CollideCircles: invalid circleA");
    assert(circleB.IsValid() && "CollideCircles: invalid circleB");

    const Vec2 centerA = Mul(xfA, circleA.localCenter);
    const Vec2 centerB = Mul(xfB, circleB.localCenter);
    const Vec2 delta = centerB - centerA;
    const float radiusSum = circleA.radius + circleB.radius;

    // Separation is decided on the penetration (radiusSum - distance) and only a
    // strictly negative value is rejected. That keeps the exactly-tangent case
    // (penetration == 0) as a real contact, as required by the Manifold contract;
    // a squared-distance early-out would also need an epsilon and could flip the
    // tangent case either way depending on rounding.
    const float distance = std::sqrt(LengthSquared(delta));
    const float penetration = radiusSum - distance;
    if (!(penetration >= 0.0f)) {
        // Positive test on purpose: a NaN distance (corrupt transform) also
        // lands here, producing an empty manifold instead of a NaN contact that
        // would poison the whole island in one frame.
        return MakeEmptyManifold();
    }

    // Coincident centres have no geometric direction for the normal. Returning
    // NaN from normalising the zero vector would make the manifold (and every
    // body it touches) non-deterministic; the fixed +x axis is arbitrary but
    // reproducible, so the same scene always produces the same contact list
    // (Section 7.4 determinism requirement).
    Vec2 normal{1.0f, 0.0f};
    if (distance > kCoincidentCenterDistance) {
        normal = delta * (1.0f / distance);
    }

    // Anchor at the midpoint of the two surface points on the normal axis:
    //   centerA + normal * (rA - penetration/2) == centerB - normal * (rB - penetration/2).
    // For tangent circles this is the shared touch point; for overlapping circles
    // it is the middle of the overlap band, the only choice that treats A and B
    // symmetrically.
    const Vec2 worldPoint = centerA + normal * (circleA.radius - 0.5f * penetration);
    // Circles have no polygon features to name: referenceEdge/incidentVertex stay 0.
    return MakePointManifold(normal, penetration, worldPoint, xfA, xfB,
                             ContactID{0, 0, kContactTypeCircleCircle, 0});
}

Manifold CollideCircleAndPlane(const CircleShape& circle, const Transform& xfCircle, const PlaneShape& plane,
                               const Transform& xfPlane) noexcept {
    assert(circle.IsValid() && "CollideCircleAndPlane: invalid circle");
    assert(plane.IsValid() && "CollideCircleAndPlane: invalid plane");

    // Work in world space for the geometry: the plane is the set of points with
    // Dot(worldNormal, x) == worldOffset, with free space on the positive side.
    const Vec2 worldNormal = ComputePlaneWorldNormal(plane, xfPlane);
    const float worldOffset = ComputePlaneWorldOffset(plane, xfPlane);
    const Vec2 center = Mul(xfCircle, circle.localCenter);

    // Signed distance from the circle centre to the surface, positive on the
    // free-space side. For a +y-up ground (worldNormal = (0,1), offset 0) a
    // centre at y = 0.3 gives signedDistance = 0.3 and penetration r - 0.3.
    const float signedDistance = Dot(worldNormal, center) - worldOffset;
    const float penetration = circle.radius - signedDistance;
    if (!(penetration >= 0.0f)) {
        // Same strict-positive test as CollideCircles: exactly tangent is a
        // contact, a real gap is not, and NaN becomes an empty manifold.
        return MakeEmptyManifold();
    }

    // Contact anchor: the projection of the circle centre onto the surface,
    // center - worldNormal * signedDistance. It lies inside the overlap band for
    // a penetrating circle and is the touch point for a tangent one.
    const Vec2 worldPoint = center - worldNormal * signedDistance;

    // Manifold normal points from A (circle) to B (plane), i.e. opposite to the
    // plane's outward normal: for a circle above the ground it points downward,
    // into the solid. The solver pushes A out of B along -normal == +worldNormal.
    const Vec2 normal = -worldNormal;
    // Week 2 circle-plane feature: type + no polygon indices.
    return MakePointManifold(normal, penetration, worldPoint, xfCircle, xfPlane,
                             ContactID{0, 0, kContactTypeCirclePlane, 0});
}

Manifold CollidePlaneAndCircle(const PlaneShape& plane, const Transform& xfPlane, const CircleShape& circle,
                               const Transform& xfCircle) noexcept {
    // Generate the manifold once with the circle as A and flip it, so the normal
    // convention (A -> B) becomes correct for the caller's argument order. The
    // flip bit in the ContactID records that the geometry was evaluated in the
    // opposite order, which is what lets World keep a stable Key per pair after
    // normalising the pair order by handle.
    return FlipManifold(CollideCircleAndPlane(circle, xfCircle, plane, xfPlane));
}

Manifold CollideCircleAndPolygon(const CircleShape& circle, const Transform& xfCircle,
                                 const PolygonShape& polygon, const Transform& xfPolygon) noexcept {
    assert(circle.IsValid() && "CollideCircleAndPolygon: invalid circle");
    assert(polygon.IsValid() && "CollideCircleAndPolygon: invalid polygon");

    // Work in the polygon's local frame: its edges and normals are stored there,
    // so only the circle centre needs the inverse transform.
    const Vec2 circleCenter = MulT(xfPolygon, Mul(xfCircle, circle.localCenter));
    const float radius = circle.radius;

    // Circle vs convex polygon is SAT with one axis per polygon edge: if the
    // centre is farther than the radius outside any edge, the shapes are apart.
    // The same pass picks the edge with the largest separation; that edge and its
    // two Voronoi regions contain the closest feature.
    int32_t bestEdge = 0;
    float bestSeparation = Dot(polygon.normals[0], circleCenter - polygon.vertices[0]);
    if (bestSeparation > radius) {
        return MakeEmptyManifold();
    }
    for (int32_t i = 1; i < polygon.count; ++i) {
        const float separation = Dot(polygon.normals[i], circleCenter - polygon.vertices[i]);
        if (separation > radius) {
            return MakeEmptyManifold();
        }
        if (separation > bestSeparation) {
            bestSeparation = separation;
            bestEdge = i;
        }
    }

    const Vec2 edgeStart = polygon.vertices[bestEdge];
    const Vec2 edgeEnd = polygon.vertices[(bestEdge + 1) % polygon.count];
    const Vec2 edge = edgeEnd - edgeStart;
    // |edge| >= kLinearSlop is enforced by PolygonShape validation, so the
    // division below cannot blow up.
    const float projection = Dot(circleCenter - edgeStart, edge) / LengthSquared(edge);

    ContactID feature;
    feature.type = kContactTypeVertexFace;
    feature.incidentVertex = 0;

    Vec2 localNormal; // circle -> polygon, polygon-local frame
    Vec2 localAnchor; // contact anchor, polygon-local frame
    float penetration = 0.0f;

    if (bestSeparation <= 0.0f) {
        // Degenerate case: the centre is on or inside the polygon, so "towards B"
        // has no geometric direction. Use the negated outward edge normal: the
        // solver applies -normal to A (the circle), which then pushes the circle
        // OUT of the polygon (separating) rather than deeper in. The anchor is the
        // clamped projection onto the most-separated edge.
        const float clampedProjection = Clamp(projection, 0.0f, 1.0f);
        localAnchor = edgeStart + edge * clampedProjection;
        localNormal = -polygon.normals[bestEdge];
        penetration = radius - bestSeparation; // separation <= 0 => penetration >= radius
        // Inside/degenerate keeps the edge index as its feature (nearest boundary edge).
        feature.referenceEdge = static_cast<uint8_t>(bestEdge);
    } else if (projection < 0.0f || projection > 1.0f) {
        // Vertex Voronoi region: the closest feature is one of the edge endpoints.
        const int32_t vertexIndex = projection < 0.0f ? bestEdge : (bestEdge + 1) % polygon.count;
        const Vec2 toVertex = polygon.vertices[vertexIndex] - circleCenter;
        const float distance = Length(toVertex);
        if (distance > radius) {
            // The edge lines are within the radius but the corner is not: the
            // circle sits diagonally outside the Voronoi regions (separated).
            return MakeEmptyManifold();
        }
        localAnchor = polygon.vertices[vertexIndex];
        // Direction is undefined only when the centre coincides with the vertex;
        // fall back to the edge normal (deterministic and still separating).
        localNormal = distance > kEpsilon ? toVertex * (1.0f / distance) : -polygon.normals[bestEdge];
        penetration = radius - distance;
        // ContactID.h documents the dual use of referenceEdge for circle-polygon:
        // it holds the nearest *vertex* index in the vertex Voronoi region.
        feature.referenceEdge = static_cast<uint8_t>(vertexIndex);
    } else {
        // Edge Voronoi region: the closest point is the perpendicular projection.
        localAnchor = edgeStart + edge * projection;
        // The polygon's outward edge normal points polygon -> circle, so the
        // circle -> polygon manifold normal is its negation.
        localNormal = -polygon.normals[bestEdge];
        penetration = radius - bestSeparation;
        feature.referenceEdge = static_cast<uint8_t>(bestEdge);
    }

    return MakePointManifold(Mul(xfPolygon.q, localNormal), penetration, Mul(xfPolygon, localAnchor),
                             xfCircle, xfPolygon, feature);
}

Manifold CollidePolygonAndCircle(const PolygonShape& polygon, const Transform& xfPolygon,
                                 const CircleShape& circle, const Transform& xfCircle) noexcept {
    // Same geometry with the A/B roles normalized the other way; FlipManifold
    // reverses the normal, swaps the anchors and sets the ContactID flip metadata.
    return FlipManifold(CollideCircleAndPolygon(circle, xfCircle, polygon, xfPolygon));
}

Manifold CollidePolygonAndPlane(const PolygonShape& polygon, const Transform& xfPolygon,
                                const PlaneShape& plane, const Transform& xfPlane) noexcept {
    assert(polygon.IsValid() && "CollidePolygonAndPlane: invalid polygon");
    assert(plane.IsValid() && "CollidePolygonAndPlane: invalid plane");

    const Vec2 worldNormal = ComputePlaneWorldNormal(plane, xfPlane);
    const float worldOffset = ComputePlaneWorldOffset(plane, xfPlane);

    // Keep the two deepest vertices behind the plane. A flat resting face supplies
    // two of them (stable support for the BoxStack demo), a tilted corner supplies
    // exactly one. Ties keep the lower vertex index via the strict comparisons.
    int32_t deepestVertex = -1;
    int32_t secondVertex = -1;
    float deepestDepth = -1.0f;
    float secondDepth = -1.0f;
    float maxDepth = 0.0f;
    for (int32_t i = 0; i < polygon.count; ++i) {
        const Vec2 worldVertex = Mul(xfPolygon, polygon.vertices[i]);
        const float signedDistance = Dot(worldNormal, worldVertex) - worldOffset;
        if (signedDistance > 0.0f) {
            continue; // vertex is in free space: not a contact feature
        }
        const float depth = -signedDistance; // >= 0; 0 means exactly tangent
        maxDepth = std::max(maxDepth, depth);
        if (depth > deepestDepth) {
            secondVertex = deepestVertex;
            secondDepth = deepestDepth;
            deepestVertex = i;
            deepestDepth = depth;
        } else if (depth > secondDepth) {
            secondVertex = i;
            secondDepth = depth;
        }
    }
    if (deepestVertex < 0) {
        // Every vertex is strictly in free space: no contact (tangent vertices have
        // signedDistance == 0 and ARE included above).
        return MakeEmptyManifold();
    }

    // Emit points in ascending vertex index so the manifold order the solver
    // sweeps is deterministic frame to frame.
    if (secondVertex >= 0 && secondVertex < deepestVertex) {
        const int32_t swap = deepestVertex;
        deepestVertex = secondVertex;
        secondVertex = swap;
    }

    Manifold manifold;
    manifold.Clear();
    // A (polygon) -> B (plane): opposite to the plane's outward normal, so the
    // solver pushes the polygon back into free space along -normal.
    manifold.normal = -worldNormal;
    manifold.penetration = maxDepth;

    // Direction towards the plane in the polygon's local frame, used to pick which
    // of a vertex's two adjacent edges is its feature edge.
    const Vec2 localPlaneDirection = MulT(xfPolygon.q, -worldNormal);
    const int32_t contactVertices[kMaxManifoldPoints] = {deepestVertex, secondVertex};
    for (const int32_t vertexIndex : contactVertices) {
        if (vertexIndex < 0) {
            continue;
        }
        const Vec2 worldVertex = Mul(xfPolygon, polygon.vertices[vertexIndex]);
        const float signedDistance = Dot(worldNormal, worldVertex) - worldOffset;
        // Anchor on the plane surface: project the penetrating vertex by its
        // signed distance (v - n*s).
        const Vec2 worldAnchor = worldVertex - worldNormal * signedDistance;

        ContactID feature;
        feature.type = kContactTypeCirclePlane; // shared plane-contact type (ContactID.h)
        feature.referenceEdge =
            static_cast<uint8_t>(FindSupportingEdge(polygon, vertexIndex, localPlaneDirection));
        feature.incidentVertex = static_cast<uint8_t>(vertexIndex);
        manifold.points[manifold.pointCount] = MakeContactPoint(xfPolygon, xfPlane, worldAnchor, feature);
        ++manifold.pointCount;
    }
    return manifold;
}

Manifold CollidePlaneAndPolygon(const PlaneShape& plane, const Transform& xfPlane,
                                const PolygonShape& polygon, const Transform& xfPolygon) noexcept {
    return FlipManifold(CollidePolygonAndPlane(polygon, xfPolygon, plane, xfPlane));
}

Manifold FlipManifold(const Manifold& manifold) noexcept {
    Manifold flipped = manifold;
    flipped.normal = -manifold.normal;

    // Only the live points are swapped; the unused slots keep their defaults,
    // exactly like a freshly generated manifold with this point count.
    for (int32_t i = 0; i < manifold.pointCount; ++i) {
        ContactPoint& point = flipped.points[i];
        point.localPointA = manifold.points[i].localPointB;
        point.localPointB = manifold.points[i].localPointA;
        // Toggle rather than set: flipping twice restores the original manifold
        // (normal, anchors and Key), which makes the operation an involution and
        // keeps Week 4's A/B normalisation order-independent.
        point.id.flip = static_cast<uint8_t>(manifold.points[i].id.flip ^ 1u);
    }
    return flipped;
}

} // namespace p2
