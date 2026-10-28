#pragma once

// p2::SATCache — previous-frame reference face, used as a hysteresis hint when
// two candidate separating axes have almost equal penetration (Section 6.3,
// Week 4 D6 / P07).
//
// SAT is axis-symmetric: for a box resting flat on another box, "the reference
// face is A's top edge" and "the reference face is B's bottom edge" describe the
// same physical contact and produce nearly identical separations. Without a
// tie-break, floating-point noise can flip the chosen reference from frame to
// frame, which flips every ContactID in the manifold and destroys the
// frame-to-frame identity warm starting depends on.
//
// The cache therefore only records a *preference*: when two axes are within a
// relative epsilon of the best separation, the cached reference is kept. It
// never changes the collision result - only which of the near-tied axes is
// reported - so enabling/disabling it must leave the physics equivalent.
//
// Ownership: World stores one cache per contact pair (Contact::satCache) and
// carries it across frames; the cache is passed into CollidePolygons as an
// in/out parameter.

#include <cstdint>

namespace p2 {

struct SATCache {
    // 0 = polygon A was the reference, 1 = polygon B was the reference.
    int32_t referenceBody = -1;
    int32_t referenceEdge = 0;

    [[nodiscard]] bool IsValid() const noexcept { return referenceBody == 0 || referenceBody == 1; }

    void Clear() noexcept {
        referenceBody = -1;
        referenceEdge = 0;
    }
};

} // namespace p2
