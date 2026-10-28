// Unit tests for the Week 2 shape layer — CircleShape and PlaneShape geometry
// helpers (design document Week 2 D2, P06).
//
// All expected values are closed-form, and the arithmetic is written next to the
// assertion so the test can be checked without reading the implementation. The
// shapes are pure geometry: no World, no solver, no transforms beyond the ones
// passed in explicitly.

#include <gtest/gtest.h>

#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <limits>

namespace {

using p2::AABB;
using p2::CircleShape;
using p2::MassData;
using p2::PlaneShape;
using p2::Rot;
using p2::ShapeType;
using p2::Transform;
using p2::Vec2;

// Shapes are exact closed-form geometry; 1e-6 only covers float rounding of the
// trig and of the mass products.
constexpr float kTolerance = 1.0e-6f;

} // namespace

TEST(ShapeTypes, EnumValuesAndMassDataDefaults) {
    // The enum values are part of the public contract: ContactID and the future
    // shape dispatch table index into arrays with them, and they are persisted in
    // debug visualisation, so they are pinned here.
    EXPECT_EQ(static_cast<int>(ShapeType::Circle), 0);
    EXPECT_EQ(static_cast<int>(ShapeType::Plane), 1);
    EXPECT_EQ(static_cast<int>(ShapeType::Polygon), 2);

    // A default MassData is the "zero mass" result PlaneShape returns.
    const MassData mass;
    EXPECT_FLOAT_EQ(mass.mass, 0.0f);
    EXPECT_TRUE((mass.center == Vec2{0.0f, 0.0f}));
    EXPECT_FLOAT_EQ(mass.inertia, 0.0f);
}

TEST(CircleShape, IsValidAcceptsStrictlyPositiveFiniteRadius) {
    // Default-constructed circle: radius 0.5, centre at the body origin.
    EXPECT_TRUE((CircleShape{}.IsValid()));

    // A tiny but positive radius is still a valid shape (the engine does not
    // impose an arbitrary minimum; kLinearSlop handles contact resolution).
    CircleShape small;
    small.radius = 1.0e-3f;
    EXPECT_TRUE(small.IsValid());

    // localCenter may be anything finite: it just shifts where the circle sits.
    CircleShape offset;
    offset.localCenter = Vec2{3.0f, -4.0f};
    EXPECT_TRUE(offset.IsValid());
}

TEST(CircleShape, IsValidRejectsZeroNegativeAndNonFiniteRadius) {
    CircleShape shape;

    shape.radius = 0.0f; // degenerate: every mass property would be zero
    EXPECT_FALSE(shape.IsValid());

    shape.radius = -1.0f; // sign error is a bug, never "the same circle"
    EXPECT_FALSE(shape.IsValid());

    shape.radius = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(shape.IsValid());

    shape.radius = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(shape.IsValid());
}

TEST(CircleShape, IsValidRejectsNonFiniteLocalCenter) {
    CircleShape shape; // radius is the valid default 0.5
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    shape.localCenter = Vec2{nan, 0.0f};
    EXPECT_FALSE(shape.IsValid());
    shape.localCenter = Vec2{0.0f, inf};
    EXPECT_FALSE(shape.IsValid());
}

TEST(CircleShape, MassMatchesAnalyticDisc) {
    // r = 0.5, rho = 2:
    //   A = pi * r^2              = pi * 0.25 = 0.7853982 m^2
    //   m = rho * A              = 2 * pi/4  = pi/2 = 1.5707963 kg
    //   I = (1/2) * m * r^2      = 0.5 * (pi/2) * 0.25 = pi/16 = 0.19634954 kg*m^2
    CircleShape shape;
    shape.radius = 0.5f;
    const MassData mass = p2::ComputeCircleMass(shape, 2.0f);

    EXPECT_NEAR(mass.mass, p2::kPi / 2.0f, kTolerance);
    EXPECT_NEAR(mass.inertia, p2::kPi / 16.0f, kTolerance);
    EXPECT_TRUE((mass.center == Vec2{0.0f, 0.0f}));
}

TEST(CircleShape, MassUsesLocalCenterAndScalesWithDensity) {
    CircleShape shape;
    shape.radius = 2.0f;
    shape.localCenter = Vec2{1.5f, -0.5f};

    // Uniform density 3: A = pi*4 = 4*pi, m = 3*4*pi = 12*pi = 37.699112,
    // I = 0.5 * m * r^2 = 0.5 * 12*pi * 4 = 24*pi = 75.398224.
    const MassData mass = p2::ComputeCircleMass(shape, 3.0f);
    EXPECT_NEAR(mass.mass, 12.0f * p2::kPi, 1.0e-4f);
    EXPECT_NEAR(mass.inertia, 24.0f * p2::kPi, 1.0e-4f);
    // The centre of mass is reported in the body-local frame, unchanged.
    EXPECT_TRUE((mass.center == Vec2{1.5f, -0.5f}));

    // Mass is linear in density: doubling rho doubles m and I, the centre stays.
    const MassData doubled = p2::ComputeCircleMass(shape, 6.0f);
    EXPECT_NEAR(doubled.mass, 2.0f * mass.mass, 1.0e-4f);
    EXPECT_NEAR(doubled.inertia, 2.0f * mass.inertia, 1.0e-4f);
}

TEST(CircleShape, AabbIsWorldCenterPlusMinusRadius) {
    // r = 0.5 at the origin under the identity transform:
    // lower = (-0.5, -0.5), upper = (0.5, 0.5); width = height = 1.
    CircleShape shape;
    const AABB box = p2::ComputeCircleAABB(shape, Transform{});
    EXPECT_TRUE((box.lowerBound == Vec2{-0.5f, -0.5f}));
    EXPECT_TRUE((box.upperBound == Vec2{0.5f, 0.5f}));
    EXPECT_FLOAT_EQ(box.GetWidth(), 1.0f);
    EXPECT_FLOAT_EQ(box.GetHeight(), 1.0f);

    // Translation moves the box rigidly: centre (3, 4) -> [2.5, 3.5] x [3.5, 4.5].
    const AABB moved = p2::ComputeCircleAABB(shape, Transform{Vec2{3.0f, 4.0f}, Rot()});
    EXPECT_TRUE((moved.lowerBound == Vec2{2.5f, 3.5f}));
    EXPECT_TRUE((moved.upperBound == Vec2{3.5f, 4.5f}));
}

TEST(CircleShape, AabbRespectsRotatedLocalCenter) {
    // r = 1, localCenter = (2, 0), xf = rotate 90 deg CCW then translate (3, 4):
    //   Rot90 * (2, 0) = (0, 2)
    //   worldCenter    = (0, 2) + (3, 4) = (3, 6)
    //   box            = [2, 4] x [5, 7]
    CircleShape shape;
    shape.radius = 1.0f;
    shape.localCenter = Vec2{2.0f, 0.0f};
    const Transform xf{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const AABB box = p2::ComputeCircleAABB(shape, xf);

    EXPECT_NEAR(box.lowerBound.x, 2.0f, kTolerance);
    EXPECT_NEAR(box.lowerBound.y, 5.0f, kTolerance);
    EXPECT_NEAR(box.upperBound.x, 4.0f, kTolerance);
    EXPECT_NEAR(box.upperBound.y, 7.0f, kTolerance);
    // The box centre equals the transformed local centre.
    EXPECT_NEAR(box.GetCenter().x, 3.0f, kTolerance);
    EXPECT_NEAR(box.GetCenter().y, 6.0f, kTolerance);
    EXPECT_TRUE(box.IsValid());
}

TEST(PlaneShape, IsValidAcceptsUnitNormals) {
    // Default ground plane: normal (0, 1), offset 0.
    EXPECT_TRUE((PlaneShape{}.IsValid()));

    // Any rotated unit vector is valid: (0.6, 0.8) is the 3-4-5 direction.
    PlaneShape tilted;
    tilted.normal = Vec2{0.6f, 0.8f};
    tilted.offset = -2.5f;
    EXPECT_TRUE(tilted.IsValid());

    // The 1e-4 length tolerance accepts a normal that drifted by a few ulps
    // (e.g. through repeated Rot composition or a text round-trip); 1.00005 is
    // inside the band and 1.0002 is outside.
    PlaneShape drifted;
    drifted.normal = Vec2{1.00005f, 0.0f};
    EXPECT_TRUE(drifted.IsValid());
}

TEST(PlaneShape, IsValidRejectsNonUnitZeroAndNonFiniteNormals) {
    PlaneShape plane;

    plane.normal = Vec2{0.0f, 2.0f}; // length 2, not a direction
    EXPECT_FALSE(plane.IsValid());

    plane.normal = Vec2{0.0f, 0.0f}; // zero vector: solid/free space undefined
    EXPECT_FALSE(plane.IsValid());

    plane.normal = Vec2{1.0002f, 0.0f}; // length error 2e-4 > 1e-4 tolerance
    EXPECT_FALSE(plane.IsValid());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    plane.normal = Vec2{nan, 1.0f};
    EXPECT_FALSE(plane.IsValid());

    plane.normal = Vec2{0.0f, 1.0f};
    plane.offset = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(plane.IsValid());
    plane.offset = nan;
    EXPECT_FALSE(plane.IsValid());
}

TEST(PlaneShape, MassIsZeroRegardlessOfDensity) {
    // An infinite half-space has no finite mass/inertia; the density argument is
    // only there so the shape dispatch stays uniform.
    PlaneShape plane;
    const MassData mass = p2::ComputePlaneMass(plane, 7.0f);
    EXPECT_FLOAT_EQ(mass.mass, 0.0f);
    EXPECT_TRUE((mass.center == Vec2{0.0f, 0.0f}));
    EXPECT_FLOAT_EQ(mass.inertia, 0.0f);

    // Even a "density 0" call returns the same zero result (no division by it).
    const MassData zeroDensity = p2::ComputePlaneMass(plane, 0.0f);
    EXPECT_FLOAT_EQ(zeroDensity.mass, 0.0f);
}

TEST(PlaneShape, WorldNormalAndOffsetUnderTransform) {
    // Identity: the world plane equals the local one.
    const PlaneShape ground; // normal (0,1), offset 0
    EXPECT_TRUE((p2::ComputePlaneWorldNormal(ground, Transform{}) == Vec2{0.0f, 1.0f}));
    EXPECT_FLOAT_EQ(p2::ComputePlaneWorldOffset(ground, Transform{}), 0.0f);

    // Rotate 90 deg CCW and translate by (3, 4):
    //   worldNormal = Rot90 * (0, 1) = (-1, 0)
    //   worldOffset = 0 + Dot((-1, 0), (3, 4)) = -3
    // The surface is therefore the vertical line x = 3, with solid-space on its
    // positive-x side (Dot(worldNormal, x) < worldOffset  <=>  -x < -3).
    const Transform xf{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const Vec2 worldNormal = p2::ComputePlaneWorldNormal(ground, xf);
    EXPECT_NEAR(worldNormal.x, -1.0f, kTolerance);
    EXPECT_NEAR(worldNormal.y, 0.0f, kTolerance);
    EXPECT_NEAR(p2::ComputePlaneWorldOffset(ground, xf), -3.0f, kTolerance);

    // A point on the transformed surface satisfies Dot(worldNormal, p) ==
    // worldOffset, which is the invariant the narrow phase depends on.
    const Vec2 surfacePoint = worldNormal * p2::ComputePlaneWorldOffset(ground, xf);
    EXPECT_NEAR(p2::Dot(worldNormal, surfacePoint), p2::ComputePlaneWorldOffset(ground, xf), kTolerance);

    // A local offset translates the surface along the world normal:
    // offset 2 with the same transform gives worldOffset = 2 - 3 = -1.
    PlaneShape lifted = ground;
    lifted.offset = 2.0f;
    EXPECT_NEAR(p2::ComputePlaneWorldOffset(lifted, xf), -1.0f, kTolerance);
}

TEST(PlaneShape, AabbIsHugeAndContainsArbitraryPoints) {
    // The plane proxy is a box centred on the point of the surface nearest the
    // body origin, with half extent kPlaneAabbExtent = 1e6 m.
    const PlaneShape ground;
    const AABB box = p2::ComputePlaneAABB(ground, Transform{});
    EXPECT_TRUE(box.IsValid());
    EXPECT_NEAR(box.GetCenter().x, 0.0f, kTolerance);
    EXPECT_NEAR(box.GetCenter().y, 0.0f, kTolerance);
    EXPECT_FLOAT_EQ(box.GetWidth(), 2.0f * p2::kPlaneAabbExtent);
    EXPECT_FLOAT_EQ(box.GetHeight(), 2.0f * p2::kPlaneAabbExtent);

    // Typical world coordinates are far inside the proxy; only points beyond
    // ~1e6 m from the centre are rejected (the real broad phase special-cases
    // planes in Week 6).
    EXPECT_TRUE((p2::Contains(box, Vec2{0.0f, 0.0f})));
    EXPECT_TRUE((p2::Contains(box, Vec2{123.0f, -456.0f})));
    EXPECT_TRUE((p2::Contains(box, Vec2{-1.0e5f, 1.0e5f})));
    EXPECT_FALSE((p2::Contains(box, Vec2{1.5e6f, 0.0f})));

    // Rotated + translated plane: p0 = worldNormal * worldOffset = (3, 0),
    // so the huge box stays centred on that point.
    const Transform xf{Vec2{3.0f, 4.0f}, Rot(p2::kHalfPi)};
    const AABB movedBox = p2::ComputePlaneAABB(ground, xf);
    EXPECT_NEAR(movedBox.GetCenter().x, 3.0f, kTolerance);
    EXPECT_NEAR(movedBox.GetCenter().y, 0.0f, kTolerance);
    EXPECT_TRUE((p2::Contains(movedBox, Vec2{3.0f, 4.0f})));
    EXPECT_TRUE((p2::Contains(movedBox, Vec2{-1000.0f, 250.0f})));
    EXPECT_FALSE((p2::Contains(movedBox, Vec2{3.0f + 1.5e6f, 0.0f})));
}
