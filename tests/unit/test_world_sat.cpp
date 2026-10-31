// Week 4 D1/D3/D5/D6/D7: World integration tests for convex polygon shapes,
// polygon narrow-phase dispatch, SAT cache persistence and the BoxStack scene.
//
// Expectations are hand-computed from the polygon mass formulas (P04) and the
// SAT/analytic contact contracts (P07/P06):
//   box half extents (hx, hy), density rho:  mass = rho * 4*hx*hy,
//   inertia about the centre = mass * (hx^2 + hy^2) / 3 (that is m*(w^2+h^2)/12
//   with w = 2hx, h = 2hy); AttachShape performs the parallel-axis transfer to
//   the body origin.
//   resting box on the ground: two contact points (the bottom face), normal +y
//   when the plane body has the lower index.
//   triangle (0,0),(1,0),(0,1): area 0.5, centroid (1/3,1/3), I about origin =
//   rho/6 (one non-zero edge term 2 in the rho/12 sum).

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Collision/Contact.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace {

using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::ShapeHandle;
using p2::ShapeType;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;

// Week 4 has no warm starting yet (that lands in Week 5), so every frame the PGS
// starts from zero accumulated impulse and must re-converge. For a symmetric
// face-face stack the two contact points are solved sequentially and the leftover
// per-frame bias becomes a net torque. With zero friction (friction lands week 5)
// nothing damps the resulting lateral drift, so a multi-box tower is physically
// unstable at the default 8 iterations; raising the iteration count only delays it.
// Measured collapse matrix (10 s, ground plane + N boxes):
//   N=2:  stable at 40 iterations (gap 1.000); default 8 drifts apart
//   N=5:  ~80 iterations needed; N=10: never stable at any tested iteration count.
// The two-box test below therefore pins the solver capability with 40 iterations;
// the ten-box test records the honest W4 baseline instead of asserting a stand.
constexpr int32_t kStableTwoBoxIterations = 40;
// The SAT-cache comparison runs on the two-box scene with 20 iterations (near
// stable: gap 0.997) so the cache signal is not drowned by a collapse.
constexpr int32_t kCacheComparisonIterations = 20;

BodyDef MakeBodyDef(BodyType type, Vec2 position, float density = 1.0f) {
    BodyDef def;
    def.type = type;
    def.position = position;
    def.density = density;
    return def;
}

BodyHandle AddGroundPlane(World& world) {
    const BodyHandle ground = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f)));
    return ground;
}

BodyHandle AddBox(World& world, Vec2 position, float halfWidth, float halfHeight, float density = 1.0f) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position, density));
    EXPECT_TRUE(world.AttachShape(body, world.CreateBoxShape(halfWidth, halfHeight)));
    return body;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

int32_t TotalContactPoints(const World& world) {
    int32_t points = 0;
    for (const p2::Contact& contact : world.GetContacts()) {
        points += contact.manifold.pointCount;
    }
    return points;
}

float MaxPenetration(const World& world) {
    float maxPenetration = 0.0f;
    for (const p2::Contact& contact : world.GetContacts()) {
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    return maxPenetration;
}

bool IsFiniteBody(const World& world, BodyHandle body) {
    const Vec2 position = world.GetPosition(body);
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(world.GetAngle(body)) &&
           std::isfinite(world.GetLinearVelocity(body).x) && std::isfinite(world.GetLinearVelocity(body).y) &&
           std::isfinite(world.GetAngularVelocity(body));
}

TEST(WorldSatTest, CreateBoxShapeDerivesMassAndInertia) {
    World world(Vec2(0.0f, 0.0f));
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 1.0f));
    const ShapeHandle box = world.CreateBoxShape(1.0f, 1.0f);
    ASSERT_TRUE(world.IsValidShape(box));
    ASSERT_TRUE(world.AttachShape(body, box));

    EXPECT_EQ(world.GetShapeCount(), 1);
    const p2::Shape* shape = world.GetShape(box);
    ASSERT_NE(shape, nullptr);
    EXPECT_EQ(shape->type, ShapeType::Polygon);
    EXPECT_EQ(shape->polygon.count, 4);
    EXPECT_FLOAT_EQ(shape->polygon.centroid.x, 0.0f);
    EXPECT_FLOAT_EQ(shape->polygon.centroid.y, 0.0f);

    // 2 x 2 box, rho = 1: area 4 -> mass 4; I = m*(w^2+h^2)/12 = 4*(4+4)/12 = 2.6667.
    EXPECT_NEAR(world.GetMass(body), 4.0f, 1.0e-5f);
    EXPECT_NEAR(world.GetInvMass(body), 0.25f, 1.0e-6f);
    const p2::Body* attached = world.GetBody(body);
    ASSERT_NE(attached, nullptr);
    EXPECT_NEAR(attached->inertia, 8.0f / 3.0f, 1.0e-5f);
    EXPECT_NEAR(attached->invInertia, 3.0f / 8.0f, 1.0e-6f);

    // Cached AABB is the box bounds immediately after AttachShape.
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.x, -1.0f);
    EXPECT_FLOAT_EQ(attached->aabb.lowerBound.y, -1.0f);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.x, 1.0f);
    EXPECT_FLOAT_EQ(attached->aabb.upperBound.y, 1.0f);
}

TEST(WorldSatTest, CreatePolygonShapeTriangleDerivesAnalyticMass) {
    World world(Vec2(0.0f, 0.0f));
    const Vec2 vertices[3] = {Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f)}; // CCW
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f), 2.0f));
    const ShapeHandle triangle = world.CreatePolygonShape(vertices, 3);
    ASSERT_TRUE(world.IsValidShape(triangle));
    ASSERT_TRUE(world.AttachShape(body, triangle));

    const p2::Shape* shape = world.GetShape(triangle);
    ASSERT_NE(shape, nullptr);
    EXPECT_EQ(shape->polygon.count, 3);
    // area = 0.5 -> mass = rho * area = 1; centroid = ((0+1+0)/3, (0+0+1)/3).
    EXPECT_NEAR(world.GetMass(body), 1.0f, 1.0e-5f);
    EXPECT_NEAR(shape->polygon.centroid.x, 1.0f / 3.0f, 1.0e-6f);
    EXPECT_NEAR(shape->polygon.centroid.y, 1.0f / 3.0f, 1.0e-6f);
    // I about the body origin = rho/6 = 1/3 for this triangle (hand-computed:
    // only the (1,0)->(0,1) edge contributes 2 to sum(cross * (Pi.Pi + Pi.Pj + Pj.Pj));
    // parallel-axis transfer from the centroid is undone by AttachShape, which is
    // why the stored inertia is the about-origin value, not the centroid one).
    const p2::Body* attached = world.GetBody(body);
    ASSERT_NE(attached, nullptr);
    EXPECT_NEAR(attached->inertia, 1.0f / 3.0f, 1.0e-5f);
}

TEST(WorldSatTest, BoxOnGroundRestsOnTwoContactPoints) {
    World world;
    const BodyHandle ground = AddGroundPlane(world);                    // index 0
    const BodyHandle box = AddBox(world, Vec2(0.0f, 0.5f), 0.5f, 0.5f); // bottom at y = 0
    ASSERT_TRUE(world.IsValid(box));

    StepFor(world, 300);
    ASSERT_GE(world.GetContactCount(), 1);
    const p2::Contact& contact = world.GetContacts()[0];
    // A = plane (index 0), B = box: the plane's outward normal is +y and
    // CollidePlaneAndPolygon flips the polygon-plane manifold, so A -> B is +y.
    EXPECT_EQ(contact.bodyA, ground);
    EXPECT_EQ(contact.bodyB, box);
    EXPECT_FLOAT_EQ(contact.manifold.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, 1.0f);
    // A flat box resting on the plane touches along an edge -> exactly two points.
    EXPECT_EQ(contact.manifold.pointCount, 2);
    EXPECT_LE(contact.manifold.penetration, p2::kLinearSlop * 2.0f);
    EXPECT_LT(std::fabs(world.GetLinearVelocity(box).y), 0.02f);

    // Position drift over the last second is bounded: the split-impulse position
    // layer corrects only the overlap above the slop band, so the resting height
    // is a constant (Week 5 replaced the Week 2-4 TEMPORARY push-out).
    const float heightAtFourSeconds = world.GetPosition(box).y;
    StepFor(world, 60);
    const float heightAtFiveSeconds = world.GetPosition(box).y;
    EXPECT_LT(std::fabs(heightAtFiveSeconds - heightAtFourSeconds), 1.0e-2f);
    EXPECT_TRUE(IsFiniteBody(world, box));
}

TEST(WorldSatTest, ReversedCreationOrderFlipsThePolygonPlaneNormal) {
    World world;
    const BodyHandle box = AddBox(world, Vec2(0.0f, 0.5f), 0.5f, 0.5f); // index 0
    const BodyHandle ground = AddGroundPlane(world);                    // index 1

    world.Step(kDt);
    ASSERT_GE(world.GetContactCount(), 1);
    const p2::Contact& contact = world.GetContacts()[0];
    EXPECT_EQ(contact.bodyA, box);
    EXPECT_EQ(contact.bodyB, ground);
    // A = polygon, B = plane -> normal points from the box into the plane (−y),
    // and the manifold is produced by CollidePolygonAndPlane (no flip bit).
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, -1.0f);
    EXPECT_EQ(contact.manifold.pointCount, 2);
    EXPECT_EQ(contact.manifold.points[0].id.flip, 0);
}

TEST(WorldSatTest, CirclePolygonDispatchBothOrders) {
    // Static box (top at y = 0.5) with a dynamic circle tangent on top.
    World world;
    const BodyHandle box = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f), 1.0f)); // index 0
    ASSERT_TRUE(world.AttachShape(box, world.CreateBoxShape(2.0f, 0.5f)));
    const BodyHandle circle = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.8f)));
    ASSERT_TRUE(world.AttachShape(circle, world.CreateCircleShape(0.3f))); // centre 0.8, top 0.5

    world.Step(kDt);
    ASSERT_EQ(world.GetContactCount(), 1);
    const p2::Contact& contact = world.GetContacts()[0];
    EXPECT_EQ(contact.bodyA, box); // lower index
    EXPECT_EQ(contact.bodyB, circle);
    // A = polygon, B = circle -> CollidePolygonAndCircle: normal from box to
    // circle is +y.
    EXPECT_FLOAT_EQ(contact.manifold.normal.y, 1.0f);
    EXPECT_NEAR(contact.manifold.penetration, 0.0f, 1.0e-5f);
    EXPECT_EQ(contact.manifold.points[0].id.type, p2::kContactTypeVertexFace);

    // Reversed creation order exercises the other dispatch branch
    // (CollideCircleAndPolygon): normal now points from circle to box = −y.
    World reversed;
    const BodyHandle reversedCircle =
        reversed.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.8f))); // index 0
    ASSERT_TRUE(reversed.AttachShape(reversedCircle, reversed.CreateCircleShape(0.3f)));
    const BodyHandle reversedBox =
        reversed.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f))); // index 1
    ASSERT_TRUE(reversed.AttachShape(reversedBox, reversed.CreateBoxShape(2.0f, 0.5f)));

    reversed.Step(kDt);
    ASSERT_EQ(reversed.GetContactCount(), 1);
    EXPECT_EQ(reversed.GetContacts()[0].bodyA, reversedCircle);
    EXPECT_FLOAT_EQ(reversed.GetContacts()[0].manifold.normal.y, -1.0f);
}

TEST(WorldSatTest, TwoBoxStackStaysStackedWithEnoughIterations) {
    World world;
    AddGroundPlane(world);
    const BodyHandle lower = AddBox(world, Vec2(0.0f, 0.5f), 0.5f, 0.5f);
    const BodyHandle upper = AddBox(world, Vec2(0.0f, 1.5f), 0.5f, 0.5f);

    // Week 4 needed 40 velocity iterations to re-converge this two-box stack every
    // frame (no warm starting, no friction). Week 5 holds it at the default 8
    // iterations (covered by the ten-box milestone test); this test stays at 40 on
    // purpose as the high-iteration convergence check of the same scene.
    for (int32_t step = 0; step < 600; ++step) {
        world.Step(kDt, kStableTwoBoxIterations, 3);
    }
    // Ground-lower contributes 2 points, lower-upper contributes 2: at least three
    // points in total (the task's ">= 3 contacts" counts points).
    EXPECT_GE(TotalContactPoints(world), 3);
    EXPECT_GE(world.GetContactCount(), 2);
    EXPECT_TRUE(IsFiniteBody(world, lower));
    EXPECT_TRUE(IsFiniteBody(world, upper));
    // Week 5 tolerance change (physics reason, not a relaxed error bound): the
    // split-impulse position layer deliberately leaves contacts inside the
    // kLinearSlop = 0.005 m penetration band instead of forcing exact separation.
    // The Week 2-4 TEMPORARY push-out reached gap 1.00000 only by teleporting the
    // boxes apart, which injected potential energy at every contact. With split
    // impulse the resting state is penetration <= slop per contact, so the resting
    // gap is in [1 - slop, 1] and the lower box centre in [0.5 - slop, 0.5].
    // Measured with the Week 5 solver at step 600: gap = 0.9966 m, lower y =
    // 0.4961 m (both inside the slop band), x = 0, angle = 0.
    const Vec2 lowerPosition = world.GetPosition(lower);
    const Vec2 upperPosition = world.GetPosition(upper);
    std::printf("[world5] two-box stack (40 iterations, 600 steps): gap = %.5f m, lower y = %.5f m, "
                "lower x = %+.6f, upper x = %+.6f, angles = %+.6f / %+.6f rad\n",
                static_cast<double>(upperPosition.y - lowerPosition.y), static_cast<double>(lowerPosition.y),
                static_cast<double>(lowerPosition.x), static_cast<double>(upperPosition.x),
                static_cast<double>(world.GetAngle(lower)), static_cast<double>(world.GetAngle(upper)));
    EXPECT_NEAR(upperPosition.y - lowerPosition.y, 1.0f, p2::kLinearSlop);
    EXPECT_NEAR(lowerPosition.y, 0.5f, p2::kLinearSlop);
    EXPECT_LT(std::fabs(lowerPosition.x), 1.0e-3f);
    EXPECT_LT(std::fabs(upperPosition.x), 1.0e-3f);
    EXPECT_LT(std::fabs(world.GetAngle(lower)), 1.0e-2f);
    EXPECT_LT(std::fabs(world.GetAngle(upper)), 1.0e-2f);
}

TEST(WorldSatTest, TenBoxStackHeadlessDefaultIterations) {
    // Week 4 historical baseline (kept for the before/after record - see the
    // Week 4 devlog and README): with zero friction and no warm starting, the ten
    // 1 m boxes on a static plane at the DEFAULT 8 velocity iterations collapsed.
    // Measured then: collapse step ~= 138 (top box fell below y = 5 m) and peak
    // penetration during the collapse transient 0.1413 m. Raising the iteration
    // count only delayed it (N=2 stable at 40, N=5 at ~80, N=10 at none).
    //
    // Week 5 changes the EXPECTATION because the physics mechanism changed, not
    // because the thresholds were relaxed:
    //   - Coulomb friction (combined mu = 0.3) damps the lateral drift that the
    //     per-frame PGS torque bias used to amplify - that was the W4 root cause;
    //   - warm starting seeds each frame with last frame's accumulated impulse, so
    //     the default 8 iterations start from the previous solution instead of
    //     re-converging from zero every frame (the measured full seed, factor 1.0,
    //     is what keeps a 10-box tower upright; see Settings.h kWarmStartFactor);
    //   - the split-impulse position layer removes residual penetration without
    //     injecting energy into the real velocities (it replaces the Week 2
    //     push-out).
    // The iteration count is deliberately NOT raised: the milestone is that the
    // default 8 velocity / 3 position iterations are enough (HANDOFF section 5.1).
    // W5 contract asserted below: max penetration < 0.01 m, top box at y ~= 9.5 m,
    // per-box tilt/lateral drift bounded, no NaN. W5 measured values (printed):
    // no collapse, max penetration 0.00522 m, top y 9.47813 m, max tilt 0.00460
    // rad, max |x| over run 0.03063 m, final state all finite.
    World world;
    AddGroundPlane(world);
    std::vector<BodyHandle> boxes;
    for (int32_t index = 0; index < 10; ++index) {
        boxes.push_back(AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index)), 0.5f, 0.5f));
    }

    float maxPenetrationOverRun = 0.0f;
    float maxTiltOverRun = 0.0f;
    float maxLateralOverRun = 0.0f;
    int32_t collapseStep = -1; // same probe as the Week 4 baseline: top y < 5 m
    for (int32_t step = 0; step < 600; ++step) {
        world.Step(kDt); // stored defaults: 8 velocity / 3 position iterations
        maxPenetrationOverRun = std::max(maxPenetrationOverRun, MaxPenetration(world));
        if (collapseStep < 0 && world.GetPosition(boxes.back()).y < 5.0f) {
            collapseStep = step;
        }
        for (const BodyHandle box : boxes) {
            maxTiltOverRun = std::max(maxTiltOverRun, std::fabs(world.GetAngle(box)));
            maxLateralOverRun = std::max(maxLateralOverRun, std::fabs(world.GetPosition(box).x));
        }
    }

    // The tower still stands after 10 simulated seconds: no collapse, the top box
    // is one storey above the ninth (0.5 + 9 * 1.0 = 9.5 m), and every box keeps
    // its slot in the stack.
    EXPECT_EQ(collapseStep, -1);
    EXPECT_LT(maxPenetrationOverRun, 0.01f); // HANDOFF 5.1 milestone
    EXPECT_NEAR(world.GetPosition(boxes.back()).y, 9.5f, 0.05f);
    EXPECT_LT(maxTiltOverRun, 0.05f);      // rad: visually upright
    EXPECT_LT(maxLateralOverRun, 0.05f);   // m: no sideways wander
    EXPECT_GE(world.GetContactCount(), 9); // ground + 8..9 box-box pairs must touch
    EXPECT_LE(MaxPenetration(world), p2::kLinearSlop * 2.0f);
    for (std::size_t index = 0; index < boxes.size(); ++index) {
        EXPECT_TRUE(IsFiniteBody(world, boxes[index]));
        const Vec2 position = world.GetPosition(boxes[index]);
        EXPECT_NEAR(position.y, 0.5f + static_cast<float>(index), 0.05f);
        EXPECT_LT(std::fabs(position.x), 0.05f);
        EXPECT_LT(std::fabs(world.GetAngle(boxes[index])), 0.05f);
    }

    std::printf("[BoxStack10/W5 default 8] collapse step = %d, max penetration over run = %.5f m, "
                "max tilt = %.5f rad, max |x| = %.5f m\n",
                collapseStep, static_cast<double>(maxPenetrationOverRun), static_cast<double>(maxTiltOverRun),
                static_cast<double>(maxLateralOverRun));
    for (std::size_t index = 0; index < boxes.size(); ++index) {
        std::printf("[BoxStack10/W5 default 8] box %zu: x = %+.5f, y = %.5f, angle = %+.5f rad\n", index,
                    static_cast<double>(world.GetPosition(boxes[index]).x),
                    static_cast<double>(world.GetPosition(boxes[index]).y),
                    static_cast<double>(world.GetAngle(boxes[index])));
    }
}

TEST(WorldSatTest, SatCacheKeepsContactKeysMoreStable) {
    // Same two-box stack simulated twice (20 velocity iterations: near stable, gap
    // 0.997), once with and once without the SAT axis-hysteresis cache, 300 steps
    // each. The two-box scene is used on purpose: the ten-box tower collapses in
    // Week 4 and the collapse would drown the cache signal.
    //
    // The cache may only change which near-tied reference face is reported, so the
    // physics must stay equivalent (max penetration difference < 5 cm) while the
    // feature keys change less often. Measured over 300 steps: cache-on 4 key
    // changes vs cache-off 264; max penetration 0.00280 vs 0.00333 m.
    const auto buildStack = [](World& world) {
        std::vector<BodyHandle> boxes;
        AddGroundPlane(world);
        boxes.push_back(AddBox(world, Vec2(0.0f, 0.5f), 0.5f, 0.5f));
        boxes.push_back(AddBox(world, Vec2(0.0f, 1.5f), 0.5f, 0.5f));
        return boxes;
    };

    const auto runWithKeyTracking = [](World& world) {
        // pair (bodyA.index, bodyB.index) -> keys of the previous frame
        std::vector<std::pair<int64_t, std::vector<uint16_t>>> previousKeys;
        int64_t totalKeyChanges = 0;
        float maxPenetration = 0.0f;
        for (int32_t step = 0; step < 300; ++step) {
            world.Step(kDt, kCacheComparisonIterations, 3);
            maxPenetration = std::max(maxPenetration, MaxPenetration(world));
            std::vector<std::pair<int64_t, std::vector<uint16_t>>> currentKeys;
            currentKeys.reserve(world.GetContacts().size());
            for (const p2::Contact& contact : world.GetContacts()) {
                const int64_t pairKey = (static_cast<int64_t>(contact.bodyA.index) << 32) |
                                        static_cast<int64_t>(contact.bodyB.index);
                std::vector<uint16_t> keys;
                keys.reserve(static_cast<std::size_t>(contact.manifold.pointCount));
                for (int32_t point = 0; point < contact.manifold.pointCount; ++point) {
                    keys.push_back(contact.manifold.points[point].id.Key());
                }
                // Compare with the same pair in the previous frame (if present).
                for (const auto& previous : previousKeys) {
                    if (previous.first != pairKey) {
                        continue;
                    }
                    const std::size_t commonCount = std::min(keys.size(), previous.second.size());
                    for (std::size_t point = 0; point < commonCount; ++point) {
                        if (keys[point] != previous.second[point]) {
                            ++totalKeyChanges;
                        }
                    }
                    break;
                }
                currentKeys.emplace_back(pairKey, std::move(keys));
            }
            previousKeys = std::move(currentKeys);
        }
        return std::make_pair(totalKeyChanges, maxPenetration);
    };

    World cachedWorld;
    const std::vector<BodyHandle> cachedBoxes = buildStack(cachedWorld);
    const auto cached = runWithKeyTracking(cachedWorld);

    World plainWorld;
    const std::vector<BodyHandle> plainBoxes = buildStack(plainWorld);
    p2::SolverSettings plainSettings = plainWorld.GetSolverSettings();
    plainSettings.useSatCache = false;
    plainWorld.SetSolverSettings(plainSettings);
    const auto plain = runWithKeyTracking(plainWorld);

    const float cachedGap =
        cachedWorld.GetPosition(cachedBoxes[1]).y - cachedWorld.GetPosition(cachedBoxes[0]).y;
    const float plainGap = plainWorld.GetPosition(plainBoxes[1]).y - plainWorld.GetPosition(plainBoxes[0]).y;
    std::printf("[SAT cache] key changes: cache-on = %lld, cache-off = %lld\n",
                static_cast<long long>(cached.first), static_cast<long long>(plain.first));
    std::printf("[SAT cache] max penetration: cache-on = %.5f, cache-off = %.5f (gap %.5f / %.5f)\n",
                static_cast<double>(cached.second), static_cast<double>(plain.second),
                static_cast<double>(cachedGap), static_cast<double>(plainGap));

    EXPECT_TRUE(IsFiniteBody(cachedWorld, cachedBoxes.back()));
    EXPECT_TRUE(IsFiniteBody(plainWorld, plainBoxes.back()));
    // Hysteresis is a tie-break only: the physical result must be equivalent.
    EXPECT_LT(std::fabs(cached.second - plain.second), 0.05f);
    // With the cache enabled, at least as few key changes as without it.
    EXPECT_LE(cached.first, plain.first);
    // Both variants stay near-stable in this config, so the comparison is not a
    // collapse artefact.
    EXPECT_NEAR(cachedGap, 1.0f, 1.0e-2f);
    EXPECT_NEAR(plainGap, 1.0f, 1.0e-2f);

    // The cache is only stored for polygon-polygon contacts; it is valid when the
    // feature is enabled and stays invalid when it is disabled.
    bool cachedPolygonPairSeen = false;
    for (const p2::Contact& contact : cachedWorld.GetContacts()) {
        if (contact.bodyA.index >= cachedBoxes.front().index &&
            contact.bodyB.index >= cachedBoxes.front().index) {
            cachedPolygonPairSeen = true;
            EXPECT_TRUE(contact.satCache.IsValid());
        }
    }
    EXPECT_TRUE(cachedPolygonPairSeen);
    for (const p2::Contact& contact : plainWorld.GetContacts()) {
        if (contact.bodyA.index >= plainBoxes.front().index &&
            contact.bodyB.index >= plainBoxes.front().index) {
            EXPECT_FALSE(contact.satCache.IsValid());
        }
    }
}

TEST(WorldSatTest, TwoIdenticalPolygonWorldsStayBitIdentical) {
    const auto buildScene = [](World& world) {
        std::vector<BodyHandle> bodies;
        bodies.push_back(AddGroundPlane(world));
        bodies.push_back(AddBox(world, Vec2(-0.2f, 2.5f), 0.4f, 0.4f));
        bodies.push_back(AddBox(world, Vec2(0.2f, 1.5f), 0.4f, 0.4f));
        // A polygon (not a box) as well, to cover non-box vertex counts.
        const Vec2 triangleVertices[3] = {Vec2(-0.3f, -0.3f), Vec2(0.3f, -0.3f), Vec2(0.0f, 0.3f)};
        BodyDef triangleDef = MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 3.5f), 1.0f);
        const BodyHandle triangle = world.CreateBody(triangleDef);
        const ShapeHandle triangleShape = world.CreatePolygonShape(triangleVertices, 3);
        EXPECT_TRUE(world.AttachShape(triangle, triangleShape));
        bodies.push_back(triangle);
        return bodies;
    };

    World worldA;
    World worldB;
    const std::vector<BodyHandle> bodiesA = buildScene(worldA);
    const std::vector<BodyHandle> bodiesB = buildScene(worldB);
    ASSERT_EQ(bodiesA.size(), bodiesB.size());

    StepFor(worldA, 300);
    StepFor(worldB, 300);
    EXPECT_EQ(worldA.GetContactCount(), worldB.GetContactCount());
    for (std::size_t index = 0; index < bodiesA.size(); ++index) {
        const Vec2 positionA = worldA.GetPosition(bodiesA[index]);
        const Vec2 positionB = worldB.GetPosition(bodiesB[index]);
        EXPECT_EQ(std::memcmp(&positionA, &positionB, sizeof(Vec2)), 0);
        const float angleA = worldA.GetAngle(bodiesA[index]);
        const float angleB = worldB.GetAngle(bodiesB[index]);
        EXPECT_EQ(std::memcmp(&angleA, &angleB, sizeof(float)), 0);
    }
}

// ---------------------------------------------------------------------------
// Debug-only rejection paths (assert + invalid handle). The fixture skips these
// where the host sandbox forbids the temp-file write GoogleTest's death tests
// need; CI executes them.
// ---------------------------------------------------------------------------

class PolygonValidationDeathTest : public p2test::DeathTestFixture {};

TEST_F(PolygonValidationDeathTest, InvalidPolygonArraysAreRejected) {
    World world;
    const Vec2 clockwise[3] = {Vec2(0.0f, 0.0f), Vec2(0.0f, 1.0f), Vec2(1.0f, 0.0f)}; // CW
    const Vec2 concave[4] = {Vec2(0.0f, 0.0f), Vec2(2.0f, 0.0f), Vec2(2.0f, 2.0f), Vec2(1.0f, 0.5f)};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Vec2 nonFinite[3] = {Vec2(0.0f, 0.0f), Vec2(nan, 0.0f), Vec2(0.0f, 1.0f)};
    const Vec2 duplicate[3] = {Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f), Vec2(1.0f, 0.0f)};

    EXPECT_DEATH({ (void)world.CreatePolygonShape(nullptr, 3); }, "CreatePolygonShape");
    EXPECT_DEATH({ (void)world.CreatePolygonShape(clockwise, 3); }, "CreatePolygonShape");
    EXPECT_DEATH({ (void)world.CreatePolygonShape(concave, 4); }, "CreatePolygonShape");
    EXPECT_DEATH({ (void)world.CreatePolygonShape(nonFinite, 3); }, "CreatePolygonShape");
    EXPECT_DEATH({ (void)world.CreatePolygonShape(duplicate, 3); }, "CreatePolygonShape");
    EXPECT_DEATH({ (void)world.CreatePolygonShape(clockwise, 2); }, "CreatePolygonShape");
    // Rejections must not consume a slot.
    EXPECT_EQ(world.GetShapeCount(), 0);
}

TEST_F(PolygonValidationDeathTest, InvalidBoxExtentsAreRejected) {
    World world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_DEATH({ (void)world.CreateBoxShape(0.0f, 1.0f); }, "CreateBoxShape");
    EXPECT_DEATH({ (void)world.CreateBoxShape(-1.0f, 1.0f); }, "CreateBoxShape");
    EXPECT_DEATH({ (void)world.CreateBoxShape(1.0f, nan); }, "CreateBoxShape");
    EXPECT_EQ(world.GetShapeCount(), 0);
}

} // namespace
