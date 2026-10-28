// p2::ClipSegmentToLine — the Sutherland-Hodgman style half-plane clip used by
// the SAT manifold generator (Week 4 D4 / P07).
//
// The routine is a pure primitive: SAT calls it for the two side planes of the
// reference face (and, conceptually, for the reference face plane itself), and
// the unit tests drive it with hand-computed segments. Keeping it free of
// manifolds, ContactIDs and transforms is what makes the clipping arithmetic
// reviewable in isolation - the geometry bugs in a narrow phase are almost always
// in the clipping, not in the SAT projection.
//
// Half-plane convention (normative, Clipping.h): a point survives when
//   Dot(normal, x) <= offset
// and `normal` is unit length, so the left-hand side is the signed distance of
// the point to the boundary line (positive = the discarded side).

#include "physics2d/Collision/Clipping.h"

#include <cstdint>

namespace p2 {

int32_t ClipSegmentToLine(const Vec2 input[2], Vec2 output[2], Vec2 normal, float offset) noexcept {
    // Signed distances of the endpoints to the clipping line. Using <= 0 for
    // "inside" makes a point exactly on the boundary survive: contact manifolds
    // must report the touching/tangent configuration (penetration == 0), not drop
    // it as separated.
    const float distance0 = Dot(normal, input[0]) - offset;
    const float distance1 = Dot(normal, input[1]) - offset;

    int32_t count = 0;
    if (distance0 <= 0.0f) {
        output[count] = input[0];
        ++count;
    }
    if (distance1 <= 0.0f) {
        output[count] = input[1];
        ++count;
    }

    // Append the segment/line intersection only for a genuine crossing. The
    // product test also documents the floating-point protection: the denominator
    // below is distance0 - distance1, which for distance0 > 0 > distance1 has
    // magnitude >= max(|distance0|, |distance1|) > 0, so the division cannot blow
    // up. A segment parallel to the boundary (denominator ~ 0) never enters this
    // branch because both distances then share a sign, and an endpoint exactly on
    // the boundary keeps the product at zero - neither case produces a NaN or a
    // bogus vertex.
    if (distance0 * distance1 < 0.0f) {
        const float interpolation = distance0 / (distance0 - distance1);
        output[count] = input[0] + interpolation * (input[1] - input[0]);
        ++count;
    }

    // 0 = segment entirely on the discarded side, 1 = degenerate touch (an
    // endpoint exactly on the boundary with the other one discarded), 2 = a real
    // clipped segment. The two-point case is the normal one for SAT side planes.
    return count;
}

} // namespace p2
