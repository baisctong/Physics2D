#pragma once

// p2::ContactID — feature-encoded contact key that survives frame-to-frame
// (Section 6.3, P06/P07).
//
// ============================================================================
// RESERVED INTERFACE (Week 2 / Week 4)
// ============================================================================
// Week 1 defines and unit-tests the encoding only; the collision code that
// *produces* stable ids lands with the narrow phase. The encoding is fixed now
// because warm starting (Week 5) is worthless without it: if the key of a
// physical contact point changes between frames, the solver applies last
// frame's accumulated impulse to a different point and stacks buzz.
//
// Layout (uint16_t):
//   bits 0-3   referenceEdge   edge index on the reference polygon (0..15)
//   bits 4-7   incidentVertex  vertex index on the incident polygon (0..15)
//   bits 8-9   type            0 = vertex-face, 1 = face-face (reserved: 2,3)
//   bit  10    flip            A/B were swapped while generating this manifold
//
// Swap symmetry: the SAT routine is responsible for filling the fields so that
// the SAME physical contact produces the SAME Key regardless of whether the
// caller passed (A,B) or (B,A) - it normalises the reference polygon first
// (Week 4 D3/D5, P07). This struct only packs whatever it is given.
// ============================================================================

#include <cstdint>

namespace p2 {

struct ContactID {
    uint8_t referenceEdge = 0;  // edge index on the reference polygon
    uint8_t incidentVertex = 0; // vertex index on the incident polygon
    uint8_t type = 0;           // 0 = vertex-face, 1 = face-face
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
