// p2::PlaneShape — infinite static half-space (Week 2 D2, P06).
//
// A plane is an infinite shape, so its "mass" and "AABB" are deliberately
// special cases: zero mass for the solver and a huge box for the temporary AABB
// filter. The real geometry is evaluated analytically in Analytic.cpp, which is
// why the world-space helpers (normal + offset) live here: both the narrow phase
// and the broad-phase stand-in need exactly the same two values.

#include "physics2d/Collision/PlaneShape.h"

#include "physics2d/Core/Settings.h"
#include "physics2d/Math/MathUtils.h"

#include <cassert>
#include <cmath>

namespace p2 {
namespace {

// Tolerance for "normal has unit length". 1e-4 is loose enough to accept a
// normal that accumulated a few ulps through Rot composition (1e-7 per
// operation) or through a serialized scene (float decimal round-trip), yet far
// tighter than any meaningful direction error: the dot-product distance formula
// would scale a real 1e-4 length error to a penetration error of 1e-4 * distance
// (millimetres at even 10 m), which the solver could not distinguish from slop.
constexpr float kPlaneNormalLengthTolerance = 1.0e-4f;

} // namespace

bool PlaneShape::IsValid() const noexcept {
    // Fully qualified: inside a member function the unqualified name IsValid
    // would resolve to this member and hide the p2::IsValid(Vec2) free function.
    if (!p2::IsValid(normal) || !p2::IsFinite(offset)) {
        return false;
    }
    // The solid/free-space split is only well defined for a unit normal: the
    // distance formula Dot(normal, p) - offset assumes |normal| == 1. A zero
    // normal (the most common corruption) also fails this check.
    return std::fabs(Length(normal) - 1.0f) <= kPlaneNormalLengthTolerance;
}

MassData ComputePlaneMass(const PlaneShape& shape, float density) noexcept {
    // `shape` is only read by the debug assert below; naming it in a release-only
    // no-op keeps MSVC /W4 quiet about the unused parameter (C4100) without
    // making the validity check debug-only.
    static_cast<void>(shape);
    assert(shape.IsValid() && "ComputePlaneMass: invalid PlaneShape");

    // An infinite static half-space has no finite mass or inertia; World turns
    // these into mass = inertia = invMass = invInertia = 0 (Section 4.2), which
    // is what makes the solver treat an attached body as immovable. The density
    // argument exists only so the shape dispatch in Core/Shape.cpp has a uniform
    // signature, hence the explicit discard instead of an unused-parameter
    // warning.
    static_cast<void>(density);
    return MassData{};
}

AABB ComputePlaneAABB(const PlaneShape& shape, const Transform& xf) noexcept {
    assert(shape.IsValid() && "ComputePlaneAABB: invalid PlaneShape");

    // p0 = worldNormal * worldOffset is the point of the plane closest to the
    // body origin (the signed-distance formula is zero there). The box is not
    // rotated: an axis-aligned box that contains an infinite half-space does not
    // exist, so the contract here is a conservative proxy that overlaps every
    // candidate; the narrow phase still solves the true geometry. Centering it
    // on p0 keeps the (huge) box inside float range for reasonable offsets.
    const Vec2 worldNormal = ComputePlaneWorldNormal(shape, xf);
    const float worldOffset = ComputePlaneWorldOffset(shape, xf);
    const Vec2 center = worldNormal * worldOffset;
    const Vec2 extent{kPlaneAabbExtent, kPlaneAabbExtent};
    return AABB{center - extent, center + extent};
}

Vec2 ComputePlaneWorldNormal(const PlaneShape& shape, const Transform& xf) noexcept {
    // Rotation only: the plane's orientation is carried by the body transform,
    // exactly like CircleShape::localCenter is carried by its transform.
    return Mul(xf.q, shape.normal);
}

float ComputePlaneWorldOffset(const PlaneShape& shape, const Transform& xf) noexcept {
    // Derivation: the local surface is Dot(normal, x) == offset. For the world
    // point y = Rot(xf.q) * x + xf.p with x = Rot^T(y - xf.p), substituting gives
    //   Dot(normal, Rot^T(y - xf.p)) == offset
    //   Dot(Rot(normal), y) - Dot(Rot(normal), xf.p) == offset
    // so worldOffset = offset + Dot(worldNormal, xf.p).
    const Vec2 worldNormal = ComputePlaneWorldNormal(shape, xf);
    return shape.offset + Dot(worldNormal, xf.p);
}

} // namespace p2
