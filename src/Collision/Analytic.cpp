// p2::Analytic — closed-form collision routines for the circle family
// (Week 2 D2/D4, P06).
//
// A circle is the one shape whose closest-feature query is trivial: it has no
// edges, so the contact normal is always the centre-to-centre (or centre-to-
// plane) direction and the penetration is a scalar subtraction. That is why this
// path never touches SAT/clipping and why it is written before the polygon
// machinery (Section 6.3: "Circle-Circle / Circle-Polygon / Circle-Plane 走专用
// 解析路径").

#include "physics2d/Collision/Analytic.h"

#include "physics2d/Math/MathUtils.h"

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

// Fill the one-point manifold both analytic paths produce. `worldPoint` is the
// contact anchor in world space; storing it in both bodies' local frames
// guarantees the solver invariant
//     Mul(xfA, localPointA) == Mul(xfB, localPointB)
// up to float rounding, without either shape having to know about the other.
[[nodiscard]] Manifold MakePointManifold(Vec2 normal, float penetration, Vec2 worldPoint,
                                         const Transform& xfA, const Transform& xfB,
                                         uint8_t contactType) noexcept {
    Manifold manifold;
    manifold.Clear();
    manifold.normal = normal;
    manifold.penetration = penetration;
    manifold.pointCount = 1;

    ContactPoint& point = manifold.points[0];
    point.localPointA = MulT(xfA, worldPoint);
    point.localPointB = MulT(xfB, worldPoint);

    // Week 2 only assigns the feature type; the polygon indices (referenceEdge,
    // incidentVertex) stay zero because a circle has no features to name.
    ContactID id;
    id.type = contactType;
    point.id = id;
    return manifold;
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
    return MakePointManifold(normal, penetration, worldPoint, xfA, xfB, kContactTypeCircleCircle);
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
    return MakePointManifold(normal, penetration, worldPoint, xfCircle, xfPlane, kContactTypeCirclePlane);
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
