#pragma once

// p2::Manifold / p2::ContactPoint — the narrow-phase output consumed by the
// solver (Section 6.3, P06).
//
// ============================================================================
// RESERVED INTERFACE (Week 2 = circle paths, Week 4 = SAT + clipping)
// ============================================================================
// Data layout only in Week 1. The generator functions live in the collision
// module from Week 2 onwards and fill these structures; the Sequential Impulse
// solver reads `points[]` and the per-point accumulated impulses.
//
// Sign conventions (normative, Section 4.2 - do not flip locally):
//   normal  : unit vector pointing from shape A toward shape B
//   penetration : overlap depth along `normal`, >= 0 when touching/overlapping
//   pointCount : 0 = separated, 1 = vertex/analytic contact, 2 = face-face
//
// The accumulated impulses live on the contact point (not on the manifold) so
// that clipping can add/remove individual points and warm starting can inherit
// per-point history by ContactID (Week 5, P09).
// ============================================================================

#include "physics2d/Collision/ContactID.h"
#include "physics2d/Math/Vec2.h"

#include <cstdint>

namespace p2 {

// Section 4.4 / P06: a 2D polygon contact has at most two points.
inline constexpr int32_t kMaxManifoldPoints = 2;

struct ContactPoint {
    // Contact anchors in each body's local frame. The solver recomputes the
    // world lever arms rA/rB every frame from these + the current transforms.
    Vec2 localPointA{0.0f, 0.0f};
    Vec2 localPointB{0.0f, 0.0f};

    // Accumulated (warm-started) impulses. Written by the solver only.
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;

    // Feature key used to inherit impulses across frames (warm starting).
    ContactID id;
};

struct Manifold {
    Vec2 normal{0.0f, 1.0f}; // from A to B
    float penetration = 0.0f;
    ContactPoint points[kMaxManifoldPoints];
    int32_t pointCount = 0;

    [[nodiscard]] bool IsTouching() const noexcept { return pointCount > 0; }

    void Clear() noexcept {
        pointCount = 0;
        penetration = 0.0f;
    }
};

} // namespace p2
