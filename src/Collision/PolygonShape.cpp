// p2::PolygonShape — convex polygon collider: construction, validation and mass
// properties (Week 4 D1, P04).
//
// The shape is intentionally a plain value type with precomputed edge normals:
// SAT (Week 4) and the circle-polygon path both walk edges linearly, so caching
// the normals here removes every normalization from the narrow phase. Validation
// is strict (Section 3.4): bad geometry is rejected, never silently repaired -
// the header documents every rejection rule.

#include "physics2d/Collision/PolygonShape.h"

#include "physics2d/Core/Settings.h"
#include "physics2d/Math/MathUtils.h"

#include <cassert>
#include <cmath>
#include <cstdint>

namespace p2 {
namespace {

// Smallest allowed area. A polygon thinner than the contact slop squared cannot
// produce a stable contact normal (opposite edges nearly coincide), and the
// mass/centroid formulas divide by the shoelace sum, so this is also the guard
// that keeps those divisions away from zero.
constexpr float kMinPolygonArea = kLinearSlop * kLinearSlop;

// Build the four box corners in the canonical CCW order (starting at
// (+halfWidth, -halfHeight)). Every box input goes through this one order, which
// is what makes the edge indices - and therefore ContactID - deterministic.
void BuildBoxVertices(float halfWidth, float halfHeight, Vec2 center, Rot rotation, Vec2 (&out)[4]) noexcept {
    const Vec2 corners[4] = {
        Vec2{halfWidth, -halfHeight},  // bottom-right
        Vec2{halfWidth, halfHeight},   // top-right
        Vec2{-halfWidth, halfHeight},  // top-left
        Vec2{-halfWidth, -halfHeight}, // bottom-left
    };
    for (int32_t i = 0; i < 4; ++i) {
        out[i] = Mul(rotation, corners[i]) + center;
    }
}

} // namespace

bool PolygonShape::IsValid() const noexcept {
    if (count < 3 || count > kMaxPolygonVertices) {
        return false;
    }
    // Fully qualified math helpers: inside a member function the unqualified name
    // IsValid would resolve to this member and hide p2::IsValid(Vec2).
    for (int32_t i = 0; i < count; ++i) {
        if (!p2::IsValid(vertices[i]) || !p2::IsValid(normals[i])) {
            return false;
        }
    }
    return p2::IsValid(centroid);
}

bool SetPolygon(PolygonShape& shape, const Vec2* vertices, int32_t count) noexcept {
    // Contract: every rejected input leaves the shape invalid (count == 0), so a
    // caller can never keep using half-validated geometry.
    shape = PolygonShape{};

    if (vertices == nullptr || count < 3 || count > kMaxPolygonVertices) {
        return false;
    }
    for (int32_t i = 0; i < count; ++i) {
        if (!p2::IsValid(vertices[i])) {
            return false;
        }
    }

    // Adjacent vertices closer than the contact slop are duplicate/degenerate
    // input. P04 suggests merging them; this project rejects instead (header note:
    // silently repairing bad geometry hides caller bugs).
    const float minEdgeLengthSquared = p2::kLinearSlop * p2::kLinearSlop;
    for (int32_t i = 0; i < count; ++i) {
        const int32_t next = (i + 1) % count;
        const float edgeLengthSquared = LengthSquared(vertices[next] - vertices[i]);
        if (edgeLengthSquared < minEdgeLengthSquared) {
            return false;
        }
        // Week 8 boundary hardening: finite coordinates can still overflow this
        // squared length (half extents around 1e19 produce a length squared of
        // ~4e38 > FLT_MAX). Normalized() would then return (0,0), so the stored
        // edge normal would silently stop being unit length and every SAT axis
        // built from it would be meaningless. Such geometry is far outside the
        // supported 0.1-10 m scene scale, so it is rejected like any other bad
        // input instead of producing a shape that only fails later in the mass
        // properties / narrow phase.
        if (!std::isfinite(edgeLengthSquared)) {
            return false;
        }
    }

    // Shoelace area. CCW winding gives a positive sum; clockwise and (near-)zero
    // area are both rejected by the same guard. The area also guards the centroid
    // division below.
    float twiceArea = 0.0f;
    Vec2 centroidSum{0.0f, 0.0f};
    for (int32_t i = 0; i < count; ++i) {
        const int32_t next = (i + 1) % count;
        const float cross = Cross(vertices[i], vertices[next]);
        twiceArea += cross;
        centroidSum += (vertices[i] + vertices[next]) * cross;
    }
    // Week 8 boundary hardening: overflow guard on the shoelace sum. A finite
    // `twiceArea` is a precondition of the centroid division below; without this
    // check an infinite sum passes the area test `inf > kMinPolygonArea` and the
    // centroid becomes inf * 0 = NaN.
    if (!std::isfinite(twiceArea)) {
        return false;
    }
    const float area = 0.5f * twiceArea;
    if (!(area > kMinPolygonArea)) {
        return false;
    }

    // Strict convexity: the turn at every vertex must be positive for CCW input.
    // Zero is a redundant collinear vertex, negative is a concave vertex; both are
    // rejected. (For an already positive-area simple polygon, all turns positive
    // implies convex and CCW.)
    for (int32_t i = 0; i < count; ++i) {
        const Vec2 previous = vertices[(i + count - 1) % count];
        const Vec2 current = vertices[i];
        const Vec2 next = vertices[(i + 1) % count];
        if (!(Cross(current - previous, next - current) > 0.0f)) {
            return false;
        }
    }

    // C = (1 / (6A)) * sum((P_i + P_{i+1}) * (P_i x P_{i+1})), with 6A = 3*(2A).
    const Vec2 centroid = centroidSum * (1.0f / (3.0f * twiceArea));
    // Week 8 boundary hardening: the weighted centroid sum can overflow even when
    // the shoelace sum itself is finite (large coordinates, tiny area). A
    // non-finite centroid would silently poison ComputePolygonMass and the body
    // mass properties, so it is treated as invalid input here - the caller gets a
    // deterministic false instead of a shape that fails one module later.
    if (!p2::IsValid(centroid)) {
        return false;
    }

    // Copy first, then compute the normals: edge i needs vertices[i + 1], so a
    // single fused loop would read the still-default next vertex for i = count - 1.
    for (int32_t i = 0; i < count; ++i) {
        shape.vertices[i] = vertices[i];
    }
    for (int32_t i = 0; i < count; ++i) {
        const int32_t next = (i + 1) % count;
        // Outward normal of a CCW edge is the edge direction rotated -90 degrees:
        // (x, y) -> (y, -x). Adjacent vertices are at least kLinearSlop apart, so
        // Normalized() never hits its zero-length fallback.
        shape.normals[i] = Normalized(
            Vec2{shape.vertices[next].y - shape.vertices[i].y, shape.vertices[i].x - shape.vertices[next].x});
    }
    shape.centroid = centroid;
    shape.count = count;
    return true;
}

bool SetPolygonAsBox(PolygonShape& shape, float halfWidth, float halfHeight) noexcept {
    if (!std::isfinite(halfWidth) || !std::isfinite(halfHeight) || halfWidth <= 0.0f || halfHeight <= 0.0f) {
        shape = PolygonShape{};
        return false;
    }
    Vec2 vertices[4];
    BuildBoxVertices(halfWidth, halfHeight, Vec2{0.0f, 0.0f}, Rot{}, vertices);
    return SetPolygon(shape, vertices, 4);
}

bool SetPolygonAsBox(PolygonShape& shape, float halfWidth, float halfHeight, Vec2 center,
                     float angle) noexcept {
    if (!std::isfinite(halfWidth) || !std::isfinite(halfHeight) || halfWidth <= 0.0f || halfHeight <= 0.0f ||
        !p2::IsValid(center) || !std::isfinite(angle)) {
        shape = PolygonShape{};
        return false;
    }
    Vec2 vertices[4];
    BuildBoxVertices(halfWidth, halfHeight, center, Rot(angle), vertices);
    return SetPolygon(shape, vertices, 4);
}

MassData ComputePolygonMass(const PolygonShape& shape, float density) noexcept {
    // Preconditions from the header; a violating call is a caller bug, so it is
    // loud in debug instead of silently producing a zero-mass body.
    assert(shape.IsValid() && "ComputePolygonMass: invalid polygon");
    assert(std::isfinite(density) && density > 0.0f && "ComputePolygonMass: density must be > 0");

    // The polygonal mass integrals are the standard Green's-theorem forms
    // (Section 6.1 / P04):
    //   A         = 1/2 * sum(cross_i)
    //   C         = 1/(6A) * sum((P_i + P_{i+1}) * cross_i)
    //   I_origin  = rho/12 * sum(cross_i * (P_i.P_i + P_i.P_{i+1} + P_{i+1}.P_{i+1}))
    // with cross_i = Cross(P_i, P_{i+1}). The final inertia is transferred from the
    // body origin to the centroid with the parallel-axis theorem.
    float twiceArea = 0.0f;
    Vec2 centroidSum{0.0f, 0.0f};
    float inertiaIntegral = 0.0f;
    for (int32_t i = 0; i < shape.count; ++i) {
        const int32_t next = (i + 1) % shape.count;
        const Vec2& current = shape.vertices[i];
        const Vec2& following = shape.vertices[next];
        const float cross = Cross(current, following);
        twiceArea += cross;
        centroidSum += (current + following) * cross;
        inertiaIntegral +=
            cross * (Dot(current, current) + Dot(current, following) + Dot(following, following));
    }

    const float area = 0.5f * twiceArea;
    const float mass = density * area;
    const Vec2 center = centroidSum * (1.0f / (3.0f * twiceArea));
    const float inertiaOrigin = (density / 12.0f) * inertiaIntegral;
    const float inertia = inertiaOrigin - mass * LengthSquared(center);

    // A valid convex polygon has a non-negative centroidal inertia; a negative or
    // non-finite value means the input slipped past validation.
    assert(std::isfinite(mass) && mass > 0.0f && "ComputePolygonMass: non-finite mass");
    assert(std::isfinite(inertia) && inertia >= 0.0f && "ComputePolygonMass: non-finite/negative inertia");
    return MassData{mass, center, inertia};
}

AABB ComputePolygonAABB(const PolygonShape& shape, const Transform& xf) noexcept {
    assert(shape.IsValid() && "ComputePolygonAABB: invalid polygon");

    // Transform every vertex and take the component-wise min/max. A polygon
    // rotated about the body origin is not axis-aligned, so (unlike the circle)
    // there is no closed-form radius bound.
    Vec2 worldVertex = Mul(xf, shape.vertices[0]);
    Vec2 lower = worldVertex;
    Vec2 upper = worldVertex;
    for (int32_t i = 1; i < shape.count; ++i) {
        worldVertex = Mul(xf, shape.vertices[i]);
        lower = Min(lower, worldVertex);
        upper = Max(upper, worldVertex);
    }
    return AABB{lower, upper};
}

} // namespace p2
