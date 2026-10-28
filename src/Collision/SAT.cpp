// p2::SAT — separating-axis narrow phase and clipping-based manifold generation
// for convex polygons (Week 4 D2/D3/D4/D5, P07).
//
// Pipeline (CollidePolygons):
//   1. candidate axes = the edge normals of both polygons (in 2D the cross-product
//      axes of 3D SAT do not exist, so this set is complete, Section 6.3);
//   2. FindMaxSeparation on each polygon; the larger of the two best separations is
//      the axis of least penetration and wins;
//   3. that face becomes the *reference face*; the face of the other polygon most
//      anti-parallel to it becomes the *incident face*;
//   4. the incident edge is clipped against the two side planes of the reference
//      face, then every surviving point is accepted only if it lies behind the
//      reference face plane (penetration >= 0);
//   5. the 0/1/2 points are converted to ContactPoints: face-face when two points
//      survive, vertex-face when a single point does, with the ContactID feature
//      bits described in ContactID.h.
//
// Two properties make the result usable by the solver:
//   - normal is A -> B, so swapped arguments must return the negated normal with
//     the same depth (P07 D3);
//   - the ContactID::Key() *set* is identical for (A, B) and (B, A) because `flip`
//     is excluded from Key() (P07 D5). That only holds if the reference polygon is
//     selected independently of the argument order, hence the symmetric tie-break
//     in SelectReference below.

#include "physics2d/Collision/SAT.h"

#include "physics2d/Collision/Clipping.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace p2 {
namespace {

// A clipped point plus the incident-polygon vertex index it derives from. The
// pure ClipSegmentToLine primitive only carries positions, while ContactID needs
// the feature (vertex) index; this wrapper keeps exactly one copy of the
// intersection arithmetic by delegating to the primitive and only tracking the
// identity mapping around it.
struct ClipVertex {
    Vec2 point;
    int32_t vertex = 0;
};

int32_t ClipSegmentToLineKeepingVertex(const ClipVertex input[2], ClipVertex output[2], Vec2 normal,
                                       float offset) noexcept {
    const Vec2 points[2] = {input[0].point, input[1].point};
    Vec2 clipped[2];
    const int32_t count = ClipSegmentToLine(points, clipped, normal, offset);
    if (count == 0) {
        return 0;
    }

    // ClipSegmentToLine keeps interior endpoints first (in input order) and appends
    // the intersection last, so the surviving point identities are reconstructible
    // from exact comparisons: the values are copied verbatim, no arithmetic is
    // re-done here. An intersection belongs to the endpoint that was clipped away
    // (the Box2D convention): that endpoint is the feature the contact point is
    // leaving, so its key stays stable while the intersection slides along the
    // incident edge.
    if (count == 1) {
        output[0] = {clipped[0], clipped[0] == input[0].point ? input[0].vertex : input[1].vertex};
        return 1;
    }

    if (clipped[0] == input[0].point) {
        // Both endpoints survived, or only input[1] was cut away.
        output[0] = {clipped[0], input[0].vertex};
        output[1] = {clipped[1], input[1].vertex};
    } else if (clipped[0] == input[1].point) {
        // Only input[0] was cut away, so it owns the intersection point.
        output[0] = {clipped[0], input[1].vertex};
        output[1] = {clipped[1], input[0].vertex};
    } else {
        // Degenerate input: both endpoints coincide, so the two features are
        // indistinguishable. Keeping the input order is deterministic.
        output[0] = {clipped[0], input[0].vertex};
        output[1] = {clipped[1], input[1].vertex};
    }
    return 2;
}

struct ReferenceChoice {
    bool isB = false; // true when polygon B owns the reference face
    int32_t edge = 0; // edge index on the reference polygon
};

// Picks the reference face: the larger separation is the axis of least penetration
// and therefore the physical reference face.
//
// Exact ties (a box resting flat on a box: both candidate separations are the same
// float) are broken by a rule that does not depend on the argument order - first
// the smaller edge index, then the lexicographically smaller world-space face
// anchor. Both are properties of the geometry: the polygons' construction order is
// deterministic (SetPolygonAsBox numbers the edges the same way for every box) and
// the anchor is a world point.
//
// Why this matters: with plain "prefer A on a tie", CollidePolygons(A, B) and
// CollidePolygons(B, A) would pick *different* reference polygons for the most
// common contact of all (two parallel faces), and referenceEdge/incidentVertex -
// hence every ContactID::Key() - would flip with the argument order. P07 D3/D5
// requires the key set to be invariant under an A/B swap.
ReferenceChoice SelectReference(const PolygonShape& polygonA, const Transform& xfA,
                                const SeparationResult& sepA, const PolygonShape& polygonB,
                                const Transform& xfB, const SeparationResult& sepB) noexcept {
    if (sepA.separation != sepB.separation) {
        return (sepB.separation > sepA.separation) ? ReferenceChoice{true, sepB.edgeIndex}
                                                   : ReferenceChoice{false, sepA.edgeIndex};
    }
    if (sepA.edgeIndex != sepB.edgeIndex) {
        return (sepB.edgeIndex < sepA.edgeIndex) ? ReferenceChoice{true, sepB.edgeIndex}
                                                 : ReferenceChoice{false, sepA.edgeIndex};
    }
    const Vec2 anchorA = Mul(xfA, polygonA.vertices[sepA.edgeIndex]);
    const Vec2 anchorB = Mul(xfB, polygonB.vertices[sepB.edgeIndex]);
    if (anchorA.x != anchorB.x) {
        return (anchorB.x < anchorA.x) ? ReferenceChoice{true, sepB.edgeIndex}
                                       : ReferenceChoice{false, sepA.edgeIndex};
    }
    return (anchorB.y < anchorA.y) ? ReferenceChoice{true, sepB.edgeIndex}
                                   : ReferenceChoice{false, sepA.edgeIndex};
}

// A polygon that is not a validated PolygonShape (count outside [3,
// kMaxPolygonVertices]) has no well-defined convex geometry. The narrow phase runs
// in the hot path, so it reports "no contact" instead of asserting; user-facing
// validation happens in SetPolygon (Section 3.4).
bool IsWellFormed(const PolygonShape& shape) noexcept {
    return shape.count >= 3 && shape.count <= kMaxPolygonVertices;
}

} // namespace

SeparationResult FindMaxSeparation(const PolygonShape& polygon, const Transform& xfPolygon,
                                   const PolygonShape& other, const Transform& xfOther) noexcept {
    SeparationResult result;
    // A degenerate input has no separating direction: reporting a positive
    // separation keeps the caller's "is this pair apart?" test true (it already
    // rejected malformed shapes, see CollidePolygons).
    if (!IsWellFormed(polygon) || !IsWellFormed(other)) {
        result.separation = std::numeric_limits<float>::max();
        return result;
    }

    // The projection target is transformed once. Projecting the same vertices onto
    // every edge normal is the whole cost of 2D SAT (O(edges1 * vertices2)); doing
    // the transform inside the inner loop would multiply it by the vertex count for
    // no reason.
    Vec2 otherVertices[kMaxPolygonVertices];
    for (int32_t j = 0; j < other.count; ++j) {
        otherVertices[j] = Mul(xfOther, other.vertices[j]);
    }

    result.separation = -std::numeric_limits<float>::max();
    for (int32_t i = 0; i < polygon.count; ++i) {
        const Vec2 worldNormal = Mul(xfPolygon.q, polygon.normals[i]);
        const Vec2 edgeStart = Mul(xfPolygon, polygon.vertices[i]);

        // Separation of the projections along this axis: the deepest vertex of the
        // other polygon measured against the plane of edge i. Negative means the
        // other polygon pokes through that edge (Section 6.3 step 1).
        float separation = std::numeric_limits<float>::max();
        for (int32_t j = 0; j < other.count; ++j) {
            separation = std::min(separation, Dot(worldNormal, otherVertices[j] - edgeStart));
        }
        if (separation > result.separation) {
            result.separation = separation;
            result.edgeIndex = i;
        }
    }
    return result;
}

Manifold CollidePolygons(const PolygonShape& polygonA, const Transform& xfA, const PolygonShape& polygonB,
                         const Transform& xfB, SATCache* cache) noexcept {
    Manifold manifold; // default-constructed == pointCount 0 == "no contact"

    if (!IsWellFormed(polygonA) || !IsWellFormed(polygonB)) {
        return manifold;
    }

    // Any axis with a positive separation means the pair is apart: SAT is complete
    // in 2D, so the first hit is a definitive "no collision" (Section 6.3 step 3).
    // Exactly 0 is touching and continues into manifold generation, which is what
    // makes the tangent contact report penetration == 0 with at least one point.
    const SeparationResult separationA = FindMaxSeparation(polygonA, xfA, polygonB, xfB);
    if (separationA.separation > 0.0f) {
        return manifold;
    }
    const SeparationResult separationB = FindMaxSeparation(polygonB, xfB, polygonA, xfA);
    if (separationB.separation > 0.0f) {
        return manifold;
    }

    ReferenceChoice reference = SelectReference(polygonA, xfA, separationA, polygonB, xfB, separationB);

    // SAT cache / hysteresis (D6): when both best axes are within a relative
    // epsilon of each other, keep the previous frame's reference face. This only
    // decides which of two near-equivalent faces is reported; the physical result
    // (normal direction, penetration depth) is unchanged - only the ContactID
    // churn across frames is. Passing nullptr disables it (the D6 experiment).
    if (cache != nullptr && cache->IsValid()) {
        const bool cachedIsB = cache->referenceBody == 1;
        const SeparationResult& cachedSeparation = cachedIsB ? separationB : separationA;
        const SeparationResult& bestSeparation = reference.isB ? separationB : separationA;
        const int32_t cachedEdgeCount = cachedIsB ? polygonB.count : polygonA.count;
        const bool cachedEdgeIsValid = cache->referenceEdge >= 0 && cache->referenceEdge < cachedEdgeCount;
        // Relative tolerance scaled by max(1, |separation|): absolute for contacts
        // near touching (metres, so 1e-5 m = 0.01 mm), relative for deep overlaps.
        const float tolerance = kRelativeEpsilon * std::max(1.0f, std::fabs(bestSeparation.separation));
        if (cachedEdgeIsValid && (bestSeparation.separation - cachedSeparation.separation) <= tolerance) {
            reference = ReferenceChoice{cachedIsB, cache->referenceEdge};
        }
    }
    if (cache != nullptr) {
        // Remember the face that was actually used, so the next frame can keep it
        // while the two axes stay near-tied. Written even for a zero-point manifold:
        // the pair is still overlapping and the next frame's choice is just as good
        // a hint.
        cache->referenceBody = reference.isB ? 1 : 0;
        cache->referenceEdge = reference.edge;
    }

    const PolygonShape& referencePolygon = reference.isB ? polygonB : polygonA;
    const Transform& referenceXf = reference.isB ? xfB : xfA;
    const PolygonShape& incidentPolygon = reference.isB ? polygonA : polygonB;
    const Transform& incidentXf = reference.isB ? xfA : xfB;

    // World-space reference face. `normals[]` is unit length by the PolygonShape
    // storage contract; normalising again is cheap insurance for hand-built shapes
    // and keeps the separation/penetration scale exact.
    const Vec2 referenceNormal = Normalized(Mul(referenceXf.q, referencePolygon.normals[reference.edge]));
    if (Dot(referenceNormal, referenceNormal) < 0.5f) {
        return manifold; // zero-length normal: malformed shape, no meaningful normal
    }
    const Vec2 referenceVertex1 = Mul(referenceXf, referencePolygon.vertices[reference.edge]);
    const Vec2 referenceVertex2 =
        Mul(referenceXf, referencePolygon.vertices[(reference.edge + 1) % referencePolygon.count]);
    const Vec2 referenceTangent = Normalized(referenceVertex2 - referenceVertex1);
    if (Dot(referenceTangent, referenceTangent) < 0.5f) {
        return manifold; // zero-length reference face (coincident vertices)
    }

    // Incident face: the edge of the other polygon whose outward normal is most
    // anti-parallel to the reference normal (Section 6.3). The polygon's deepest
    // vertex (the support vertex along -referenceNormal) is always an endpoint of
    // that edge, which is why clipping below yields at least one point for a
    // genuinely overlapping pair.
    int32_t incidentEdge = 0;
    float minNormalDot = std::numeric_limits<float>::max();
    for (int32_t i = 0; i < incidentPolygon.count; ++i) {
        const float normalDot = Dot(referenceNormal, Mul(incidentXf.q, incidentPolygon.normals[i]));
        if (normalDot < minNormalDot) {
            minNormalDot = normalDot;
            incidentEdge = i;
        }
    }
    const int32_t incidentVertex1 = incidentEdge;
    const int32_t incidentVertex2 = (incidentEdge + 1) % incidentPolygon.count;
    const ClipVertex incidentEdgeVertices[2] = {
        {Mul(incidentXf, incidentPolygon.vertices[incidentVertex1]), incidentVertex1},
        {Mul(incidentXf, incidentPolygon.vertices[incidentVertex2]), incidentVertex2}};

    // Clip the incident edge against the two side planes of the reference face:
    // the part of the edge whose projection on the reference tangent lies between
    // the face's endpoints survives. Written in the Clipping.h half-plane form
    // Dot(normal, x) <= offset:
    //   start plane: normal = -tangent, offset = -Dot(tangent, referenceVertex1)
    //   end plane:   normal = +tangent, offset = +Dot(tangent, referenceVertex2)
    ClipVertex clippedSide1[2];
    ClipVertex clippedSide2[2];
    int32_t clippedCount = ClipSegmentToLineKeepingVertex(
        incidentEdgeVertices, clippedSide1, -referenceTangent, -Dot(referenceTangent, referenceVertex1));
    if (clippedCount == 0) {
        return manifold;
    }
    clippedCount = ClipSegmentToLineKeepingVertex(clippedSide1, clippedSide2, referenceTangent,
                                                  Dot(referenceTangent, referenceVertex2));
    if (clippedCount == 0) {
        return manifold;
    }

    // Reference face plane test. The third Sutherland-Hodgman pass is folded into
    // the penetration computation below instead of being a separate clip: an
    // explicit clip would keep the segment/plane crossing at depth exactly 0, so
    // every vertex-face contact would come out as a two-point manifold, while D4
    // requires a single point there. Unclipped points that are not behind the face
    // are simply rejected, which is the same set of contact points minus that
    // zero-depth crossing artefact.
    const float referenceOffset = Dot(referenceNormal, referenceVertex1);
    Vec2 contactPoints[kMaxManifoldPoints];
    int32_t contactVertices[kMaxManifoldPoints];
    int32_t contactCount = 0;
    float penetration = 0.0f;
    for (int32_t i = 0; i < clippedCount; ++i) {
        // penetration = -(Dot(referenceNormal, point - referenceVertex1)): how far
        // behind the reference face the incident point lies. Negative means the
        // point is on the free (separating) side of the face.
        const float pointPenetration = referenceOffset - Dot(referenceNormal, clippedSide2[i].point);
        if (pointPenetration < 0.0f) {
            continue;
        }
        contactPoints[contactCount] = clippedSide2[i].point;
        contactVertices[contactCount] = clippedSide2[i].vertex;
        penetration = std::max(penetration, pointPenetration);
        ++contactCount;
    }
    if (contactCount == 0) {
        // No point behind the reference face: the incident edge lies entirely on
        // the free side. For validated convex polygons this cannot happen (the
        // incident edge owns the deepest vertex, whose separation is <= 0), so this
        // is a numerical/degenerate corner and "no contact" is the safe answer.
        return manifold;
    }

    // normal is A -> B: the reference face's outward normal points from its polygon
    // towards the other one, so it must be negated when B owned the reference.
    manifold.normal = reference.isB ? -referenceNormal : referenceNormal;
    manifold.penetration = penetration;
    manifold.pointCount = contactCount;
    // Two points = the incident face lay inside the reference face (face-face);
    // one point = only a corner of the incident face reached the face (vertex-face).
    const uint8_t contactType = (contactCount == 2) ? kContactTypeFaceFace : kContactTypeVertexFace;
    for (int32_t i = 0; i < contactCount; ++i) {
        ContactPoint& point = manifold.points[i];
        // The same world anchor in both local frames: the solver rebuilds rA/rB from
        // the current transforms every frame (Manifold.h), so both must describe the
        // identical world point.
        point.localPointA = MulT(xfA, contactPoints[i]);
        point.localPointB = MulT(xfB, contactPoints[i]);
        point.normalImpulse = 0.0f;
        point.tangentImpulse = 0.0f;
        point.id.referenceEdge = static_cast<uint8_t>(reference.edge);
        point.id.incidentVertex = static_cast<uint8_t>(contactVertices[i]);
        point.id.type = contactType;
        // `flip` records only which polygon owned the reference face; Key() ignores
        // it, so the A/B swap preserves the contact identity (ContactID.h).
        point.id.flip = reference.isB ? 1 : 0;
    }
    return manifold;
}

} // namespace p2
