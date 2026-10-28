#pragma once

// p2::ContactID — feature-encoded contact key that survives frame-to-frame
// (Section 6.3, P06/P07).
//
// ============================================================================
// Feature encoding (Week 2 assigns the circle types, Week 4 the polygon types)
// ============================================================================
// The encoding is fixed because warm starting (Week 5) is worthless without it:
// if the key of a physical contact point changes between frames, the solver
// applies last frame's accumulated impulse to a different point and stacks buzz.
//
// Layout (uint16_t):
//   bits 0-3   referenceEdge   edge index on the reference polygon (0..15)
//   bits 4-7   incidentVertex  vertex index on the incident polygon (0..15)
//   bits 8-9   type            0 = vertex-face, 1 = face-face,
//                              2 = circle-circle, 3 = circle-plane (Week 2)
//   bit  10    flip            A/B were swapped while generating this manifold
//
// Swap symmetry: the collision routine is responsible for filling the fields so
// that the SAME physical contact produces the SAME Key regardless of whether the
// caller passed (A,B) or (B,A) - World normalises the pair order by handle
// index, and FlipManifold() toggles `flip` when geometry is evaluated in the
// other order. The SAT path refines this in Week 4 (P07).
// ============================================================================

#include <cstdint>

namespace p2 {

// Encoded in ContactID::type (bits 8-9).
inline constexpr uint8_t kContactTypeVertexFace = 0;   // Week 4 (SAT clipping)
inline constexpr uint8_t kContactTypeFaceFace = 1;     // Week 4 (SAT clipping)
inline constexpr uint8_t kContactTypeCircleCircle = 2; // Week 2
inline constexpr uint8_t kContactTypeCirclePlane = 3;  // Week 2

struct ContactID {
    uint8_t referenceEdge = 0;  // edge index on the reference polygon
    uint8_t incidentVertex = 0; // vertex index on the incident polygon
    uint8_t type = 0;           // one of the kContactType* constants
    uint8_t flip = 0;           // 1 when the generator swapped A and B

    [[nodiscard]] constexpr uint16_t Key() const noexcept {
        const uint16_t referencePart = static_cast<uint16_t>(referenceEdge & 0x0Fu);
        const uint16_t incidentPart = static_cast<uint16_t>((incidentVertex & 0x0Fu) << 4);
        const uint16_t typePart = static_cast<uint16_t>((type & 0x03u) << 8);
        const uint16_t flipPart = static_cast<uint16_t>((flip & 0x01u) << 10);
        return static_cast<uint16_t>(referencePart | incidentPart | typePart | flipPart);
    }

    friend constexpr bool operator==(const ContactID& a, const ContactID& b) noexcept {
        return a.Key() == b.Key();
    }
    friend constexpr bool operator!=(const ContactID& a, const ContactID& b) noexcept { return !(a == b); }
};

} // namespace p2
