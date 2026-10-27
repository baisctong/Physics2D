// p2::Rot — out-of-line trig and renormalisation for the (c, s) rotation
// (Section 6.1 / P03 of the design document).
//
// Storing cos/sin instead of an angle means composition never calls trig and
// never needs wrapping; the cost is that the unit-length invariant degrades
// after long chains of Mul(), which Normalize() repairs. Trig is paid only at
// the user-facing boundary: constructing from an angle and GetAngle().

#include "physics2d/Math/Rot.h"

#include "physics2d/Math/MathUtils.h"

#include <cmath>

namespace p2 {

Rot::Rot(float angle) noexcept : c(std::cos(angle)), s(std::sin(angle)) {}

void Rot::Set(float angle) noexcept {
    c = std::cos(angle);
    s = std::sin(angle);
}

float Rot::GetAngle() const noexcept {
    // atan2 is well-defined for every (c, s) pair except (0, 0); the invariant
    // says that cannot happen, and Normalize() repairs a broken pair. The result
    // is in (-pi, pi], i.e. rotated angles come back wrapped, which is the
    // documented user-facing behaviour.
    return std::atan2(s, c);
}

void Rot::Normalize() noexcept {
    const float lengthSquared = c * c + s * s;
    // Written as a positive test so that a NaN pair (from a corrupt save file or
    // an uninitialised read) falls into the repair branch instead of staying NaN
    // forever. The guard reuses kEpsilon as a length scale, not as an absolute
    // error bound, which is why it is squared before comparing.
    if (!(lengthSquared > kEpsilon * kEpsilon)) {
        SetIdentity();
        return;
    }
    const float invLength = 1.0f / std::sqrt(lengthSquared);
    c *= invLength;
    s *= invLength;
}

} // namespace p2
