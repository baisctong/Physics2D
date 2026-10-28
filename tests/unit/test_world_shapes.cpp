// Week 2 D2: World shape pool, mass derivation and material accessor tests.
//
// Shape ownership contract (World.h): shapes live in a generation-checked pool;
// attaching one to a body derives mass/inertia from the shape's geometry and the
// body's density; replacing a shape releases the old slot; a plane is an
// infinite half-space and is only legal on a static body.
//
// Every expected value below is hand-computed from the formulas in
// CircleShape.h / PlaneShape.h, not captured from a run.

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <limits>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::ShapeHandle;
using p2::ShapeType;
using p2::Vec2;
using p2::World;

BodyDef MakeBodyDef(BodyType type, Vec2 position, float density = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.density = density;
    return def;
}

TEST(WorldShapeTest, AttachCircleDerivesMassAndInertia) {
    World world(Vec2(0.0f, 0.0f));
    // density = 2 kg/m^2, r = 0.5 m:
    //   area    = pi * r^2      = 0.7853982 m^2
    //   mass    = rho * area    = 1.5707964 kg   (pi / 2)
    //   inertia = 0.5 * m * r^2 = 0.19634954 kg m^2  (pi / 16)
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.IsValidShape(circle));
    ASSERT_TRUE(world.AttachShape(body, circle));

    EXPECT_EQ(world.GetShapeCount(), 1);
    const p2::Shape* shape = world.GetShape(circle);
    ASSERT_NE(shape, nullptr);
    EXPECT_EQ(shape->type, ShapeType::Circle);
    EXPECT_FLOAT_EQ(shape->circle.radius, 0.5f);

    EXPECT_NEAR(world.GetMass(body), p2::kPi * 0.25f * 2.0f, 1.0e-6f);
    EXPECT_NEAR(world.GetMass(body), 1.5707964f, 1.0e-6f);
    EXPECT_NEAR(world.GetInvMass(body), 1.0f / 1.5707964f, 1.0e-6f);
    const p2::Body* attached = world.GetBody(body);
    ASSERT_NE(attached, nullptr);
    EXPECT_NEAR(attached->inertia, 0.19634954f, 1.0e-6f);
    EXPECT_NEAR(attached->invInertia, 1.0f / 0.19634954f, 1.0e-3f);

    // The cached AABB must be valid immediately after the attach (UpdateContacts
    // runs before the first Synchronize phase).
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.x, -0.5f);
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.y, -0.5f);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.x, 0.5f);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.y, 0.5f);
}

TEST(WorldShapeTest, StaticBodyStaysInfiniteMassAfterAttach) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f), 5.0f));
    const ShapeHandle circle = world.CreateCircleShape(1.0f);
    ASSERT_TRUE(world.AttachShape(body, circle));

    // Infinite mass is represented by zero (never inf) so every product stays
    // finite in the integrator and the future solver.
    EXPECT_FLOAT_EQ(world.GetMass(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetInvMass(body), 0.0f);
    const p2::Body* attached = world.GetBody(body);
    ASSERT_NE(attached, nullptr);
    EXPECT_FLOAT_EQ(attached->inertia, 0.0f);
    EXPECT_FLOAT_EQ(attached->invInertia, 0.0f);
}

TEST(WorldShapeTest, KinematicBodyStaysInfiniteMassAfterAttach) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Kinematic, Vec2(0.0f, 0.0f), 5.0f));
    const ShapeHandle circle = world.CreateCircleShape(1.0f);
    ASSERT_TRUE(world.AttachShape(body, circle));
    EXPECT_FLOAT_EQ(world.GetMass(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetInvMass(body), 0.0f);
}

TEST(WorldShapeTest, FixedRotationKeepsInvInertiaZeroAfterAttach) {
    World world(Vec2(0.0f, 0.0f));
    BodyDef def = MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0f);
    def.fixedRotation = true;
    const BodyHandle body = world.CreateBody(def);
    ASSERT_TRUE(world.AttachShape(body, world.CreateCircleShape(0.5f)));

    const p2::Body* attached = world.GetBody(body);
    ASSERT_NE(attached, nullptr);
    EXPECT_TRUE(attached->fixedRotation);
    // Mass/inertia are still derived (they are the fallback for a non-zero
    // inertia if the flag is ever cleared), but the angular inverse mass is 0.
    EXPECT_GT(attached->mass, 0.0f);
    EXPECT_GT(attached->inertia, 0.0f);
    EXPECT_FLOAT_EQ(attached->invInertia, 0.0f);
}

TEST(WorldShapeTest, PlaneOnStaticBodyIsAccepted) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    const ShapeHandle plane = world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f);
    ASSERT_TRUE(world.AttachShape(ground, plane));

    const p2::Shape* shape = world.GetShape(plane);
    ASSERT_NE(shape, nullptr);
    EXPECT_EQ(shape->type, ShapeType::Plane);
    // An infinite half-space has no finite bounds: the Week 2 proxy is the huge
    // box of kPlaneAabbExtent centred on the closest plane point (here origin).
    const p2::Body* attached = world.GetBody(ground);
    ASSERT_NE(attached, nullptr);
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.x, -p2::kPlaneAabbExtent);
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.y, -p2::kPlaneAabbExtent);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.x, p2::kPlaneAabbExtent);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.y, p2::kPlaneAabbExtent);
}

TEST(WorldShapeTest, ReplacingShapeRecomputesMassAndReleasesOldSlot) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0f));
    const ShapeHandle smallCircle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.AttachShape(body, smallCircle));
    // mass = pi * 0.5^2 = 0.7853982 kg.
    EXPECT_NEAR(world.GetMass(body), 0.7853982f, 1.0e-6f);
    EXPECT_EQ(world.GetShapeCount(), 1);

    const ShapeHandle bigCircle = world.CreateCircleShape(1.0f);
    ASSERT_EQ(world.GetShapeCount(), 2); // both shapes alive before the replace
    ASSERT_TRUE(world.AttachShape(body, bigCircle));

    // The old shape slot is released, so exactly one shape is alive again.
    EXPECT_EQ(world.GetShapeCount(), 1);
    EXPECT_FALSE(world.IsValidShape(smallCircle));
    EXPECT_TRUE(world.IsValidShape(bigCircle));
    // mass = pi * 1^2 = 3.1415927 kg, inertia = 0.5 * m * r^2 = 1.5707964.
    EXPECT_NEAR(world.GetMass(body), p2::kPi, 1.0e-6f);
    EXPECT_NEAR(world.GetBody(body)->inertia, 0.5f * p2::kPi, 1.0e-6f);
}

TEST(WorldShapeTest, ReattachingSameShapeKeepsTheSlotAlive) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0f));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.AttachShape(body, circle));
    ASSERT_TRUE(world.AttachShape(body, circle));
    EXPECT_EQ(world.GetShapeCount(), 1);
    EXPECT_TRUE(world.IsValidShape(circle));
    EXPECT_NEAR(world.GetMass(body), p2::kPi * 0.25f, 1.0e-6f);
}

TEST(WorldShapeTest, DestroyBodyReleasesAttachedShape) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.AttachShape(body, circle));
    EXPECT_EQ(world.GetShapeCount(), 1);

    world.DestroyBody(body);
    EXPECT_EQ(world.GetBodyCount(), 0);
    EXPECT_EQ(world.GetShapeCount(), 0);
    EXPECT_FALSE(world.IsValidShape(circle));
}

TEST(WorldShapeTest, DestroyShapeReleasesUnattachedSlotOnly) {
    World world(Vec2(0.0f, 0.0f));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_EQ(world.GetShapeCount(), 1);
    world.DestroyShape(circle);
    EXPECT_EQ(world.GetShapeCount(), 0);
    EXPECT_FALSE(world.IsValidShape(circle));
    // Destruction is idempotent (same policy as DestroyBody).
    world.DestroyShape(circle);
    EXPECT_EQ(world.GetShapeCount(), 0);
}

TEST(WorldShapeTest, AttachShapeRejectsInvalidHandlesWithoutAsserting) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.IsValidShape(circle));

    // Invalid handles are a normal user-cleanup situation, not a programmer
    // error: the operation returns false instead of asserting.
    EXPECT_FALSE(world.AttachShape(BodyHandle{}, circle));
    EXPECT_FALSE(world.AttachShape(body, ShapeHandle{}));
    EXPECT_FALSE(world.AttachShape(BodyHandle{}, ShapeHandle{}));
    // A stale shape handle must also be rejected: it cannot silently revive.
    world.DestroyShape(circle);
    EXPECT_FALSE(world.AttachShape(body, circle));
    // The failed attaches must not have changed the body's mass fallback.
    EXPECT_FLOAT_EQ(world.GetMass(body), 1.0f); // BodyDef default mass
}

TEST(WorldShapeTest, ShapePoolReusesReleasedSlots) {
    World world(Vec2(0.0f, 0.0f));
    const ShapeHandle first = world.CreateCircleShape(0.25f);
    const ShapeHandle second = world.CreateCircleShape(0.5f);
    ASSERT_EQ(world.GetShapeCount(), 2);
    world.DestroyShape(first);
    EXPECT_EQ(world.GetShapeCount(), 1);

    const ShapeHandle third = world.CreateCircleShape(0.75f);
    EXPECT_EQ(world.GetShapeCount(), 2);
    // The released slot is reused (index 0) with a new generation, so the old
    // handle stays stale.
    EXPECT_EQ(third.index, first.index);
    EXPECT_NE(third.generation, first.generation);
    EXPECT_FALSE(world.IsValidShape(first));
    EXPECT_TRUE(world.IsValidShape(second));
    EXPECT_TRUE(world.IsValidShape(third));
}

TEST(WorldShapeTest, BodyWithoutShapeHasDegenerateAabb) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(1.0f, 2.0f)));
    const p2::Body* created = world.GetBody(body);
    ASSERT_NE(created, nullptr);
    // No collider -> a point-sized box at the body origin (keeps the contact
    // pair filter well-defined).
    EXPECT_FLOAT_EQ(created->aabb.lowerBound.x, 1.0f);
    EXPECT_FLOAT_EQ(created->aabb.lowerBound.y, 2.0f);
    EXPECT_FLOAT_EQ(created->aabb.upperBound.x, 1.0f);
    EXPECT_FLOAT_EQ(created->aabb.upperBound.y, 2.0f);
}

TEST(WorldShapeTest, MaterialAccessorsRoundTrip) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    EXPECT_FLOAT_EQ(world.GetFriction(body), 0.3f);    // BodyDef default
    EXPECT_FLOAT_EQ(world.GetRestitution(body), 0.0f); // BodyDef default
    world.SetFriction(body, 0.85f);
    world.SetRestitution(body, 0.6f);
    EXPECT_FLOAT_EQ(world.GetFriction(body), 0.85f);
    EXPECT_FLOAT_EQ(world.GetRestitution(body), 0.6f);
    // Stale handles return the neutral default instead of dereferencing dead data.
    world.DestroyBody(body);
    EXPECT_FLOAT_EQ(world.GetFriction(body), 0.0f);
    EXPECT_FLOAT_EQ(world.GetRestitution(body), 0.0f);
}

// ---------------------------------------------------------------------------
// Debug-only rejection paths (assert). The fixture skips these where the host
// sandbox forbids the temp-file write GoogleTest's death tests need; CI runs them.
// ---------------------------------------------------------------------------

class ShapeValidationDeathTest : public p2test::DeathTestFixture {};

TEST_F(ShapeValidationDeathTest, CircleRejectsInvalidGeometry) {
    World world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // Non-positive or non-finite radius, and a non-finite local centre, are
    // rejected with an invalid handle + debug assert (never repaired).
    EXPECT_DEATH({ (void)world.CreateCircleShape(0.0f); }, "CreateCircleShape");
    EXPECT_DEATH({ (void)world.CreateCircleShape(-1.0f); }, "CreateCircleShape");
    EXPECT_DEATH({ (void)world.CreateCircleShape(nan); }, "CreateCircleShape");
    EXPECT_DEATH({ (void)world.CreateCircleShape(0.5f, Vec2(nan, 0.0f)); }, "CreateCircleShape");
}

TEST_F(ShapeValidationDeathTest, PlaneRejectsInvalidGeometry) {
    World world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // The signed-distance formula assumes |normal| == 1, so a zero or non-unit
    // normal is rejected instead of silently scaling every penetration.
    EXPECT_DEATH({ (void)world.CreatePlaneShape(Vec2(0.0f, 0.0f), 0.0f); }, "CreatePlaneShape");
    EXPECT_DEATH({ (void)world.CreatePlaneShape(Vec2(0.0f, 2.0f), 0.0f); }, "CreatePlaneShape");
    EXPECT_DEATH({ (void)world.CreatePlaneShape(Vec2(nan, 1.0f), 0.0f); }, "CreatePlaneShape");
}

TEST_F(ShapeValidationDeathTest, PlaneCannotAttachToDynamicBody) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    const ShapeHandle plane = world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f);
    ASSERT_TRUE(world.IsValid(body));
    ASSERT_TRUE(world.IsValidShape(plane));
    EXPECT_DEATH({ (void)world.AttachShape(body, plane); }, "PlaneShape");
}

TEST_F(ShapeValidationDeathTest, DestroyAttachedShapeAsserts) {
    World world;
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    const ShapeHandle circle = world.CreateCircleShape(0.5f);
    ASSERT_TRUE(world.AttachShape(body, circle));
    EXPECT_DEATH({ world.DestroyShape(circle); }, "DestroyShape");
}

} // namespace
