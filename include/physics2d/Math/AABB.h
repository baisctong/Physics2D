#pragma once

// p2::AABB — axis-aligned bounding box used by the broad phase (Section 4.5).
//
// Overlap convention: boxes that merely touch (lower == upper on an axis) DO
// overlap. Rationale (P03): a contact at exactly zero separation is a real
// contact state, and excluding it would make the broad phase lose pairs during
// the exact frame a touching stack settles. Narrow phase (SAT) makes the final
// call anyway, so being inclusive here is the safe direction.
//
// Extend() semantics: grow the box outward by `margin` on every side
// (margin must be >= 0). This is the "fat AABB" of Section 6.2, used to avoid
// re-inserting proxies every frame in the future sweep-and-prune broad phase.

#include "physics2d/Math/Vec2.h"

namespace p2 {

struct AABB {
    Vec2 lowerBound{0.0f, 0.0f};
    Vec2 upperBound{0.0f, 0.0f};

    constexpr AABB() = default;
    constexpr AABB(Vec2 lower, Vec2 upper) : lowerBound(lower), upperBound(upper) {}

    [[nodiscard]] constexpr Vec2 GetCenter() const noexcept { return (lowerBound + upperBound) * 0.5f; }

    // Half-extents (Box2D naming: "extents" = half width/height).
    [[nodiscard]] constexpr Vec2 GetExtents() const noexcept { return (upperBound - lowerBound) * 0.5f; }

    [[nodiscard]] constexpr float GetWidth() const noexcept { return upperBound.x - lowerBound.x; }
    [[nodiscard]] constexpr float GetHeight() const noexcept { return upperBound.y - lowerBound.y; }

    // Perimeter (not area) is what Box2D's dynamic tree uses as its cost
    // heuristic; kept here so the future tree can reuse it.
    [[nodiscard]] constexpr float GetPerimeter() const noexcept { return 2.0f * (GetWidth() + GetHeight()); }

    // Grow outward by margin on all sides.
    void Extend(float margin) noexcept;

    [[nodiscard]] bool IsValid() const noexcept;
};

[[nodiscard]] AABB Union(const AABB& a, const AABB& b) noexcept;

// Inclusive overlap test (touching counts as overlap, see file header).
[[nodiscard]] bool Overlaps(const AABB& a, const AABB& b) noexcept;

[[nodiscard]] bool Contains(const AABB& outer, const AABB& inner) noexcept;
[[nodiscard]] bool Contains(const AABB& box, Vec2 point) noexcept;

} // namespace p2
