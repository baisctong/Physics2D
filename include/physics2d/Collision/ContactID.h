#pragma once

// p2::ContactID — feature-encoded contact key that survives frame-to-frame
// (Section 6.3, P06/P07).
//
// ============================================================================
// Feature encoding (Week 2 assigned the circle/plane types, Week 4 the polygon
// types; the layout itself was frozen in Week 1)
// ============================================================================
// The encoding is fixed because warm starting (Week 5) is worthless without it:
// if the key of a physical contact point changes between frames, the solver
// applies last frame's accumulated impulse to a different point and stacks buzz.
//
// Identity layout (uint16_t, produced by Key()):
//   bits 0-3   referenceEdge   edge index on the reference polygon / plane contact
//   bits 4-7   incidentVertex  vertex index on the incident polygon
//   bits 8-9   type            see kContactType* below
//
// `flip` is stored on the struct but is deliberately NOT part of Key(): it records
// only that the collision routine evaluated the geometry with A and B swapped
// (World normalises the pair order by handle index, FlipManifold() toggles it).
// The same physical contact must keep the same identity when the caller passes
// (A,B) or (B,A) - P07 D3/D5 explicitly requires the Key *set* to match under a
// swap, which is only possible if orientation metadata is excluded from the key.
//
// Type values and per-shape feature meaning:
//   0 vertex-face      polygon-polygon single-point clipping; circle-polygon uses
//                      it too (referenceEdge = nearest polygon edge index when the
//                      circle is in the edge Voronoi region, or nearest vertex
//                      index when it is in the vertex region; incidentVertex = 0)
//   1 face-face        polygon-polygon two-point clipping
//   2 circle-circle    incidentVertex/referenceEdge stay 0
//   3 plane contact    circle-plane (Week 2) and polygon-plane (Week 4):
//                      referenceEdge = polygon edge index (0 for a circle),
//                      incidentVertex = polygon vertex index (0 for a circle)
// ============================================================================

#include <cstdint>

namespace p2 {

// Encoded in ContactID::type (bits 8-9).
inline constexpr uint8_t kContactTypeVertexFace = 0;   // W4: SAT single point / circle-polygon
inline constexpr uint8_t kContactTypeFaceFace = 1;     // W4: SAT two points
inline constexpr uint8_t kContactTypeCircleCircle = 2; // W2
inline constexpr uint8_t kContactTypeCirclePlane = 3;  // W2 circle-plane; W4 polygon-plane too

struct ContactID {
    uint8_t referenceEdge = 0;  // edge index on the reference polygon
    uint8_t incidentVertex = 0; // vertex index on the incident polygon
    uint8_t type = 0;           // one of the kContactType* constants
    uint8_t flip = 0;           // 1 when the generator swapped A and B (metadata)

    [[nodiscard]] constexpr uint16_t Key() const noexcept {
        const uint16_t referencePart = static_cast<uint16_t>(referenceEdge & 0x0Fu);
        const uint16_t incidentPart = static_cast<uint16_t>((incidentVertex & 0x0Fu) << 4);
        const uint16_t typePart = static_cast<uint16_t>((type & 0x03u) << 8);
        return static_cast<uint16_t>(referencePart | incidentPart | typePart);
    }

    // Full comparison includes the orientation metadata; Key() is the identity
    // used for cross-frame impulse inheritance.
    friend constexpr bool operator==(const ContactID& a, const ContactID& b) noexcept {
        return a.referenceEdge == b.referenceEdge && a.incidentVertex == b.incidentVertex &&
               a.type == b.type && a.flip == b.flip;
    }
    friend constexpr bool operator!=(const ContactID& a, const ContactID& b) noexcept { return !(a == b); }
};

} // namespace p2
