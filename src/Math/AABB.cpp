// p2::AABB — out-of-line broad-phase queries (Section 4.5 / P03 of the design
// document).
//
// The trivial accessors stay inline in AABB.h; the functions here encode the
// inclusive overlap convention and the "fat AABB" growth that the future
// sweep-and-prune broad phase depends on.

#include "physics2d/Math/AABB.h"

#include "physics2d/Math/MathUtils.h"

namespace p2 {

void AABB::Extend(float margin) noexcept {
    // The contract requires a non-negative margin. Clamping instead of asserting
    // keeps Extend monotonically growing even when a caller passes a negative
    // "shrink" by mistake: the broad phase only ever uses Extend to fatten
    // proxies, and a silent shrink could make it drop real pairs.
    const float grow = margin > 0.0f ? margin : 0.0f;
    lowerBound.x -= grow;
    lowerBound.y -= grow;
    upperBound.x += grow;
    upperBound.y += grow;
}

bool AABB::IsValid() const noexcept {
    if (!IsFinite(lowerBound.x) || !IsFinite(lowerBound.y) || !IsFinite(upperBound.x) ||
        !IsFinite(upperBound.y)) {
        return false;
    }
    // An inverted box has no interior and would break every downstream overlap
    // and containment query, so it is rejected here once. A degenerate box
    // (lower == upper) is allowed: a point or a zero-thickness segment is a
    // legitimate proxy for future shape types.
    return lowerBound.x <= upperBound.x && lowerBound.y <= upperBound.y;
}

AABB Union(const AABB& a, const AABB& b) noexcept {
    return {Min(a.lowerBound, b.lowerBound), Max(a.upperBound, b.upperBound)};
}

bool Overlaps(const AABB& a, const AABB& b) noexcept {
    // Inclusive test: touching (lower == upper on an axis) counts as overlap
    // because zero separation is a real contact state and the broad phase must
    // not drop a settling stack on the exact frame it touches. The narrow phase
    // makes the final call anyway, so erring on "overlap" is the safe direction.
    return a.lowerBound.x <= b.upperBound.x && b.lowerBound.x <= a.upperBound.x &&
           a.lowerBound.y <= b.upperBound.y && b.lowerBound.y <= a.upperBound.y;
}

bool Contains(const AABB& outer, const AABB& inner) noexcept {
    // Inclusive on both sides for the same reason as Overlaps: an inner box that
    // shares a face with the outer box is still contained.
    return outer.lowerBound.x <= inner.lowerBound.x && inner.upperBound.x <= outer.upperBound.x &&
           outer.lowerBound.y <= inner.lowerBound.y && inner.upperBound.y <= outer.upperBound.y;
}

bool Contains(const AABB& box, Vec2 point) noexcept {
    // Bounds are inclusive so that points on the surface count as contained;
    // contact points are frequently generated exactly on a face.
    return box.lowerBound.x <= point.x && point.x <= box.upperBound.x && box.lowerBound.y <= point.y &&
           point.y <= box.upperBound.y;
}

} // namespace p2
