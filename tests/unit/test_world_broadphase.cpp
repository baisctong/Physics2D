// Week 6 (P11/P12/D5): World broad-phase integration tests.
//
// Weeks 2-5 enumerated contact candidates with a hard-coded O(N^2) loop in
// World::UpdateContacts. Week 6 installs an IBroadPhase (BruteForce / SAP)
// underneath the same narrow phase. The acceptance contract is that this is a
// *pure enumeration change*:
//
//   1. pair-source equivalence: a scene stepped through (a) no broad phase, (b)
//      BroadPhaseBruteForce, (c) BroadPhaseSAP must be BIT-IDENTICAL every step
//      (transforms, real/pseudo velocities, forces, and the full contact list
//      including accumulated impulses). World::UpdateBroadPhase canonicalises the
//      pair list to body indices (i < j, sorted, deduped) so the contact order -
//      and therefore the Gauss-Seidel solve order - cannot change;
//   2. candidate-pair set = fat-AABB brute-force reference, with tolerance 0:
//      World::GetBroadPhasePairCount() must equal the number of overlapping fat
//      AABBs, every step;
//   3. proxy lifecycle follows the shape lifecycle: one proxy per shaped body,
//      reused on shape replacement, released by DestroyBody/Clear/runtime switch;
//   4. the fat AABB is exactly tight-AABB + kAabbMargin, and the margin only
//      widens the candidate set - the narrow phase still makes the exact contact
//      decision (so the contact count cannot grow because of the margin);
//   5. World (not the broad phase) filters body pairs that can never respond:
//      static/static, kinematic/kinematic and collider-less bodies.
//
// W6 deviation (registered with the Lead): there is NO detach API - a live body
// cannot lose its shape (DestroyShape asserts+returns for attached shapes), so
// "destroying a shape releases its proxy" is orthogonal to the proxy bookkeeping.
// The lifecycle test proves DestroyShape on an *unattached* shape never touches
// the proxy count instead.
//
// Measurement convention: every test prints its data with the [world6] prefix so
// the devlog/README can cite real numbers.

#include <gtest/gtest.h>

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

namespace {

using p2::AABB;
using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::BroadPhaseBruteForce;
using p2::BroadPhaseSAP;
using p2::Contact;
using p2::IBroadPhase;
using p2::Shape;
using p2::ShapeHandle;
using p2::Vec2;
using p2::World;

constexpr float kDt = p2::kFixedTimeStep;
constexpr int32_t kEquivalenceSteps = 300;
constexpr int32_t kCandidateReferenceSteps = 60;

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

BodyHandle AddBox(World& world, Vec2 position, float halfExtent = 0.5f) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position));
    EXPECT_TRUE(world.AttachShape(body, world.CreateBoxShape(halfExtent, halfExtent)));
    return body;
}

BodyHandle AddCircle(World& world, Vec2 position, float radius = 0.4f) {
    const BodyHandle body = world.CreateBody(MakeBodyDef(BodyType::Dynamic, position));
    EXPECT_TRUE(world.AttachShape(body, world.CreateCircleShape(radius)));
    return body;
}

void StepFor(World& world, int32_t stepCount) {
    for (int32_t step = 0; step < stepCount; ++step) {
        world.Step(kDt);
    }
}

// One fixed scene used by the equivalence tests: a ground plane, a ten-box
// 1 m tower, three circles falling next to it, and one free-falling box. The
// scene is deliberately asymmetric (contacts change every few frames, the tower
// settles, the circles roll) so any contact-order difference between pair sources
// would show up as a divergent trajectory.
struct EquivalenceScene {
    std::vector<BodyHandle> bodies; // creation order; identical in every World
};

EquivalenceScene BuildEquivalenceScene(World& world) {
    EquivalenceScene scene;
    scene.bodies.push_back(AddGroundPlane(world));
    for (int32_t index = 0; index < 10; ++index) {
        scene.bodies.push_back(AddBox(world, Vec2(0.0f, 0.5f + static_cast<float>(index))));
    }
    scene.bodies.push_back(AddCircle(world, Vec2(-2.0f, 3.0f)));
    scene.bodies.push_back(AddCircle(world, Vec2(2.0f, 5.0f), 0.5f));
    scene.bodies.push_back(AddCircle(world, Vec2(-2.0f, 7.0f), 0.3f));
    scene.bodies.push_back(AddBox(world, Vec2(3.0f, 6.0f), 0.4f));
    return scene;
}

// Flattens the observable physics state into a float array: every live body's
// transform/velocities/pseudo-velocities/force accumulators, then every contact
// (pair identity, manifold, per-point anchors + accumulated impulses, persistence
// flag and the SAT hysteresis cache). Body contains a std::string, so the array is
// extracted field by field and compared with memcmp - not the raw Body struct.
std::vector<float> SnapshotPhysicsState(const World& world) {
    std::vector<float> values;
    const std::vector<Body>& bodies = world.GetBodies();
    values.reserve(bodies.size() * 13U + world.GetContacts().size() * 32U);
    for (const Body& body : bodies) {
        if (!body.handle.IsValid()) {
            continue;
        }
        values.push_back(body.xf.p.x);
        values.push_back(body.xf.p.y);
        values.push_back(body.xf.q.c);
        values.push_back(body.xf.q.s);
        values.push_back(body.linearVelocity.x);
        values.push_back(body.linearVelocity.y);
        values.push_back(body.angularVelocity);
        values.push_back(body.pseudoLinearVelocity.x);
        values.push_back(body.pseudoLinearVelocity.y);
        values.push_back(body.pseudoAngularVelocity);
        values.push_back(body.force.x);
        values.push_back(body.force.y);
        values.push_back(body.torque);
    }
    for (const Contact& contact : world.GetContacts()) {
        values.push_back(static_cast<float>(contact.bodyA.index));
        values.push_back(static_cast<float>(contact.bodyB.index));
        values.push_back(static_cast<float>(contact.manifold.pointCount));
        values.push_back(contact.manifold.normal.x);
        values.push_back(contact.manifold.normal.y);
        values.push_back(contact.manifold.penetration);
        values.push_back(contact.friction);
        values.push_back(contact.restitution);
        values.push_back(contact.isPersistent ? 1.0f : 0.0f);
        values.push_back(static_cast<float>(contact.satCache.referenceBody));
        values.push_back(static_cast<float>(contact.satCache.referenceEdge));
        for (int32_t pointIndex = 0; pointIndex < p2::kMaxManifoldPoints; ++pointIndex) {
            const p2::ContactPoint& point = contact.manifold.points[pointIndex];
            values.push_back(point.localPointA.x);
            values.push_back(point.localPointA.y);
            values.push_back(point.localPointB.x);
            values.push_back(point.localPointB.y);
            values.push_back(point.normalImpulse);
            values.push_back(point.tangentImpulse);
            values.push_back(static_cast<float>(point.id.Key()));
        }
    }
    return values;
}

// Returns the flat index of the first float whose bit pattern differs, or -1 when
// the arrays are identical.
int32_t FindFirstBitMismatch(const std::vector<float>& a, const std::vector<float>& b) {
    const std::size_t common = std::min(a.size(), b.size());
    for (std::size_t index = 0; index < common; ++index) {
        if (std::memcmp(&a[index], &b[index], sizeof(float)) != 0) {
            return static_cast<int32_t>(index);
        }
    }
    return a.size() == b.size() ? -1 : static_cast<int32_t>(common);
}

// Snapshot of the phase-2 input: one (body slot, fat AABB) entry per shaped body.
// Captured BEFORE Step, because UpdateBroadPhase queries the proxies with the
// AABBs Synchronize wrote at the end of the previous step.
struct BodyAabbSnapshot {
    std::vector<int32_t> bodyIndices;
    std::vector<AABB> aabbs;
};

BodyAabbSnapshot CaptureBodyAabbs(const World& world) {
    BodyAabbSnapshot snapshot;
    const std::vector<Body>& bodies = world.GetBodies();
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        const Body& body = bodies[index];
        if (!body.handle.IsValid() || !world.IsValidShape(body.shape)) {
            continue;
        }
        snapshot.bodyIndices.push_back(static_cast<int32_t>(index));
        snapshot.aabbs.push_back(body.aabb);
    }
    return snapshot;
}

// Number of unordered overlapping pairs in a captured AABB set: the reference the
// broad phase's QueryPairs output must reproduce exactly (static/kinematic bodies
// included - the broad phase is body-agnostic; World filters later).
int32_t CountOverlappingPairs(const BodyAabbSnapshot& snapshot) {
    int32_t count = 0;
    for (std::size_t i = 0; i < snapshot.aabbs.size(); ++i) {
        for (std::size_t j = i + 1; j < snapshot.aabbs.size(); ++j) {
            if (p2::Overlaps(snapshot.aabbs[i], snapshot.aabbs[j])) {
                ++count;
            }
        }
    }
    return count;
}

int32_t CountShapedBodies(const World& world) {
    int32_t count = 0;
    const std::vector<Body>& bodies = world.GetBodies();
    for (const Body& body : bodies) {
        if (body.handle.IsValid() && world.IsValidShape(body.shape)) {
            ++count;
        }
    }
    return count;
}

// Test-only broad phase decorator: delegates storage and the correct pair set to
// BroadPhaseBruteForce, but emits every pair twice (reversed order and duplicate,
// iterating the list backwards). World::UpdateBroadPhase must normalise this to
// the canonical (i<j, sorted, deduped) list; otherwise the contact order changes
// and the physics diverges from the fallback run.
class NoisyBroadPhase final : public IBroadPhase {
  public:
    [[nodiscard]] int32_t AddProxy(const AABB& aabb) override { return m_inner.AddProxy(aabb); }
    void RemoveProxy(int32_t proxyId) override { m_inner.RemoveProxy(proxyId); }
    void UpdateAABB(int32_t proxyId, const AABB& aabb) override { m_inner.UpdateAABB(proxyId, aabb); }

    void QueryPairs(const p2::PairCallback& callback) override {
        std::vector<std::pair<int32_t, int32_t>> pairs;
        m_inner.QueryPairs(
            [&pairs](int32_t proxyIdA, int32_t proxyIdB) { pairs.emplace_back(proxyIdA, proxyIdB); });
        for (auto iterator = pairs.rbegin(); iterator != pairs.rend(); ++iterator) {
            callback(iterator->second, iterator->first); // reversed (B,A)
            callback(iterator->first, iterator->second); // duplicate (A,B)
        }
    }

    [[nodiscard]] int32_t GetProxyCount() const override { return m_inner.GetProxyCount(); }
    [[nodiscard]] const char* Name() const override { return "NoisyBruteForce"; }

  private:
    BroadPhaseBruteForce m_inner;
};

TEST(WorldBroadPhaseTest, FallbackBruteForceAndSapAreBitIdenticalEveryStep) {
    // The core W6 acceptance test: changing the candidate enumeration must not
    // change one bit of the simulation. Any unsorted/duplicated/mismatched pair
    // would reorder the contact list and therefore the Gauss-Seidel iterations.
    World fallbackWorld;
    World bruteForceWorld;
    World sapWorld;
    const EquivalenceScene fallbackScene = BuildEquivalenceScene(fallbackWorld);
    const EquivalenceScene bruteForceScene = BuildEquivalenceScene(bruteForceWorld);
    const EquivalenceScene sapScene = BuildEquivalenceScene(sapWorld);
    ASSERT_EQ(fallbackScene.bodies.size(), bruteForceScene.bodies.size());
    ASSERT_EQ(fallbackScene.bodies.size(), sapScene.bodies.size());
    bruteForceWorld.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    sapWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    ASSERT_NE(bruteForceWorld.GetBroadPhase(), nullptr);
    ASSERT_NE(sapWorld.GetBroadPhase(), nullptr);
    EXPECT_EQ(fallbackWorld.GetBroadPhase(), nullptr);

    int32_t firstMismatchStep = -1;
    int32_t firstMismatchIndex = -1;
    int32_t firstPairCountMismatchStep = -1;
    std::size_t floatsPerStep = 0;
    int32_t finalContactCount = 0;
    for (int32_t step = 0; step < kEquivalenceSteps; ++step) {
        fallbackWorld.Step(kDt);
        bruteForceWorld.Step(kDt);
        sapWorld.Step(kDt);

        const std::vector<float> fallbackSnapshot = SnapshotPhysicsState(fallbackWorld);
        const std::vector<float> bruteForceSnapshot = SnapshotPhysicsState(bruteForceWorld);
        const std::vector<float> sapSnapshot = SnapshotPhysicsState(sapWorld);
        if (fallbackSnapshot.size() != bruteForceSnapshot.size() ||
            fallbackSnapshot.size() != sapSnapshot.size()) {
            firstMismatchStep = step;
            firstMismatchIndex = -1;
            break;
        }
        floatsPerStep = fallbackSnapshot.size();
        const int32_t mismatchAgainstBruteForce = FindFirstBitMismatch(fallbackSnapshot, bruteForceSnapshot);
        const int32_t mismatchAgainstSap = FindFirstBitMismatch(fallbackSnapshot, sapSnapshot);
        if (mismatchAgainstBruteForce >= 0 || mismatchAgainstSap >= 0) {
            firstMismatchStep = step;
            firstMismatchIndex =
                (mismatchAgainstBruteForce >= 0) ? mismatchAgainstBruteForce : mismatchAgainstSap;
            break;
        }
        if (bruteForceWorld.GetBroadPhasePairCount() != sapWorld.GetBroadPhasePairCount() &&
            firstPairCountMismatchStep < 0) {
            firstPairCountMismatchStep = step;
        }
        finalContactCount = fallbackWorld.GetContactCount();
        // The fallback path owns no pair list; the counter must stay 0 for it.
        EXPECT_EQ(fallbackWorld.GetBroadPhasePairCount(), 0);
    }

    std::printf("[world6] pair-source equivalence: steps = %d, floats/step = %d, "
                "contacts at end = %d, first mismatch step = %d index = %d, "
                "BF/SAP pair-count mismatch step = %d\n",
                static_cast<int>(kEquivalenceSteps), static_cast<int>(floatsPerStep),
                static_cast<int>(finalContactCount), static_cast<int>(firstMismatchStep),
                static_cast<int>(firstMismatchIndex), static_cast<int>(firstPairCountMismatchStep));
    EXPECT_EQ(firstMismatchStep, -1);
    EXPECT_EQ(firstPairCountMismatchStep, -1);
    // Sanity: the scene is dynamic enough that the pair sources were exercised.
    EXPECT_GT(finalContactCount, 0);

    auto* sapBroadPhase = static_cast<BroadPhaseSAP*>(sapWorld.GetBroadPhase());
    ASSERT_NE(sapBroadPhase, nullptr);
    EXPECT_TRUE(sapBroadPhase->ValidateInternalState());
}

TEST(WorldBroadPhaseTest, CanonicalPairOrderingAbsorbsNoisyBroadPhase) {
    // A broad phase is allowed to emit pairs in any order (Core/BroadPhaseInterface.h).
    // This decorator emits every pair reversed AND duplicated, in reverse list
    // order. World must still reproduce the fallback run exactly: pair counts must
    // match the fat-AABB reference (dedupe) and the physics must stay bit-identical
    // (i<j normalisation + ascending sort).
    World referenceWorld;
    World noisyWorld;
    const EquivalenceScene referenceScene = BuildEquivalenceScene(referenceWorld);
    const EquivalenceScene noisyScene = BuildEquivalenceScene(noisyWorld);
    ASSERT_EQ(referenceScene.bodies.size(), noisyScene.bodies.size());
    noisyWorld.SetBroadPhase(std::make_unique<NoisyBroadPhase>());
    ASSERT_NE(noisyWorld.GetBroadPhase(), nullptr);

    int32_t firstMismatchStep = -1;
    int32_t firstMismatchIndex = -1;
    int32_t firstPairCountMismatchStep = -1;
    std::size_t floatsPerStep = 0;
    for (int32_t step = 0; step < kEquivalenceSteps; ++step) {
        const BodyAabbSnapshot aabbsBeforeStep = CaptureBodyAabbs(noisyWorld);
        referenceWorld.Step(kDt);
        noisyWorld.Step(kDt);

        const std::vector<float> referenceSnapshot = SnapshotPhysicsState(referenceWorld);
        const std::vector<float> noisySnapshot = SnapshotPhysicsState(noisyWorld);
        if (referenceSnapshot.size() != noisySnapshot.size()) {
            firstMismatchStep = step;
            firstMismatchIndex = -1;
            break;
        }
        floatsPerStep = referenceSnapshot.size();
        const int32_t mismatch = FindFirstBitMismatch(referenceSnapshot, noisySnapshot);
        if (mismatch >= 0) {
            firstMismatchStep = step;
            firstMismatchIndex = mismatch;
            break;
        }
        const int32_t referencePairCount = CountOverlappingPairs(aabbsBeforeStep);
        if (noisyWorld.GetBroadPhasePairCount() != referencePairCount && firstPairCountMismatchStep < 0) {
            firstPairCountMismatchStep = step;
        }
    }

    std::printf("[world6] noisy broad phase (reversed+duplicated emission): steps = %d, "
                "floats/step = %d, first mismatch step = %d index = %d, "
                "pair-count mismatch step = %d\n",
                static_cast<int>(kEquivalenceSteps), static_cast<int>(floatsPerStep),
                static_cast<int>(firstMismatchStep), static_cast<int>(firstMismatchIndex),
                static_cast<int>(firstPairCountMismatchStep));
    EXPECT_EQ(firstMismatchStep, -1);
    // A missing dedupe would inflate the count (each pair is emitted twice); a
    // missing sort would reorder contacts and show up as a physics mismatch above.
    EXPECT_EQ(firstPairCountMismatchStep, -1);
}

TEST(WorldBroadPhaseTest, CandidatePairCountMatchesFatAabbBruteForceReference) {
    // World::GetBroadPhasePairCount() is the canonical pair list length. It must
    // equal the number of overlapping fat-AABB pairs computed by brute force,
    // with tolerance 0, every step - for both implementations.
    World bruteForceWorld;
    World sapWorld;
    const EquivalenceScene bruteForceScene = BuildEquivalenceScene(bruteForceWorld);
    const EquivalenceScene sapScene = BuildEquivalenceScene(sapWorld);
    ASSERT_EQ(bruteForceScene.bodies.size(), sapScene.bodies.size());
    bruteForceWorld.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    sapWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());

    int32_t firstBruteForceMismatchStep = -1;
    int32_t firstSapMismatchStep = -1;
    int32_t referencePairCountTotal = 0;
    for (int32_t step = 0; step < kCandidateReferenceSteps; ++step) {
        const BodyAabbSnapshot bruteForceAabbs = CaptureBodyAabbs(bruteForceWorld);
        const BodyAabbSnapshot sapAabbs = CaptureBodyAabbs(sapWorld);
        bruteForceWorld.Step(kDt);
        sapWorld.Step(kDt);
        const int32_t referenceCount = CountOverlappingPairs(bruteForceAabbs);
        referencePairCountTotal += referenceCount;
        if (bruteForceWorld.GetBroadPhasePairCount() != referenceCount && firstBruteForceMismatchStep < 0) {
            firstBruteForceMismatchStep = step;
        }
        if (sapWorld.GetBroadPhasePairCount() != CountOverlappingPairs(sapAabbs) &&
            firstSapMismatchStep < 0) {
            firstSapMismatchStep = step;
        }
    }

    std::printf("[world6] candidate-pair reference: steps = %d, reference pairs sum = %d, "
                "BF mismatch step = %d, SAP mismatch step = %d\n",
                static_cast<int>(kCandidateReferenceSteps), static_cast<int>(referencePairCountTotal),
                static_cast<int>(firstBruteForceMismatchStep), static_cast<int>(firstSapMismatchStep));
    EXPECT_EQ(firstBruteForceMismatchStep, -1);
    EXPECT_EQ(firstSapMismatchStep, -1);
    EXPECT_GT(referencePairCountTotal, 0); // the scene must actually produce candidates
}

TEST(WorldBroadPhaseTest, RuntimeBroadPhaseSwitchKeepsPhysicsIdentical) {
    // Switching the broad phase on a LIVE World (not rebuilding it) must not
    // disturb the simulation: the old proxies are released, the next step inserts
    // fresh ones from the same fat AABBs, and the canonical pair list is unchanged.
    // 30 steps per phase: no broad phase -> BruteForce -> SAP -> BruteForce ->
    // no broad phase again, compared against a World that never had one.
    World referenceWorld;
    World switchedWorld;
    const EquivalenceScene referenceScene = BuildEquivalenceScene(referenceWorld);
    const EquivalenceScene switchedScene = BuildEquivalenceScene(switchedWorld);
    ASSERT_EQ(referenceScene.bodies.size(), switchedScene.bodies.size());

    int32_t firstMismatchStep = -1;
    int32_t firstMismatchIndex = -1;
    std::size_t floatsPerStep = 0;
    int32_t comparedSteps = 0;
    const auto compareStep = [&](int32_t step) {
        referenceWorld.Step(kDt);
        switchedWorld.Step(kDt);
        ++comparedSteps;
        const std::vector<float> referenceSnapshot = SnapshotPhysicsState(referenceWorld);
        const std::vector<float> switchedSnapshot = SnapshotPhysicsState(switchedWorld);
        if (referenceSnapshot.size() != switchedSnapshot.size()) {
            if (firstMismatchStep < 0) {
                firstMismatchStep = step;
                firstMismatchIndex = -1;
            }
            return;
        }
        floatsPerStep = referenceSnapshot.size();
        const int32_t mismatch = FindFirstBitMismatch(referenceSnapshot, switchedSnapshot);
        if (mismatch >= 0 && firstMismatchStep < 0) {
            firstMismatchStep = step;
            firstMismatchIndex = mismatch;
        }
    };

    int32_t step = 0;
    for (; step < 30; ++step) {
        compareStep(step); // no broad phase
    }
    switchedWorld.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    for (; step < 60; ++step) {
        compareStep(step);
    }
    EXPECT_EQ(switchedWorld.GetBroadPhase()->GetProxyCount(), CountShapedBodies(switchedWorld));
    switchedWorld.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
    for (; step < 90; ++step) {
        compareStep(step);
    }
    EXPECT_EQ(switchedWorld.GetBroadPhase()->GetProxyCount(), CountShapedBodies(switchedWorld));
    switchedWorld.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    for (; step < 120; ++step) {
        compareStep(step);
    }
    switchedWorld.SetBroadPhase(nullptr);
    for (; step < 150; ++step) {
        compareStep(step); // back to the O(N^2) fallback
    }
    EXPECT_EQ(switchedWorld.GetBroadPhase(), nullptr);
    EXPECT_EQ(switchedWorld.GetBroadPhasePairCount(), 0);

    std::printf("[world6] runtime switch (none 30 -> BF 30 -> SAP 30 -> BF 30 -> none 30): "
                "steps = %d, floats/step = %d, first mismatch step = %d index = %d\n",
                static_cast<int>(comparedSteps), static_cast<int>(floatsPerStep),
                static_cast<int>(firstMismatchStep), static_cast<int>(firstMismatchIndex));
    EXPECT_EQ(firstMismatchStep, -1);
    EXPECT_EQ(comparedSteps, 150);
}

TEST(WorldBroadPhaseTest, ProxyLifecycleFollowsTheShapeLifecycle) {
    World world(Vec2(0.0f, 0.0f)); // gravity off: keep the lifecycle deterministic
    world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    ASSERT_NE(world.GetBroadPhase(), nullptr);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);

    // 1. A body without a collider never gets a proxy.
    const BodyHandle noShape = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    ASSERT_TRUE(world.IsValid(noShape));
    StepFor(world, 3);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);
    ASSERT_NE(world.GetBody(noShape), nullptr);
    EXPECT_EQ(world.GetBody(noShape)->proxyId, -1);

    // 2. Proxies are synchronised by UpdateBroadPhase (phase 2), not by AttachShape:
    //    no proxy exists until the next Step.
    EXPECT_TRUE(world.AttachShape(noShape, world.CreateBoxShape(0.5f, 0.5f)));
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);
    StepFor(world, 1);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 1);
    ASSERT_NE(world.GetBody(noShape), nullptr);
    EXPECT_GE(world.GetBody(noShape)->proxyId, 0);

    // 3. Replacing the attached shape keeps exactly one proxy (the old AABB slot
    //    is updated, not leaked).
    EXPECT_TRUE(world.AttachShape(noShape, world.CreateCircleShape(0.4f)));
    StepFor(world, 1);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 1);

    // 4. A second shaped body adds exactly one proxy.
    const BodyHandle second = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(3.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(second, world.CreateBoxShape(0.5f, 0.5f)));
    StepFor(world, 1);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 2);

    // 5. DestroyShape is only legal for an *unattached* shape (W6 has no detach
    //    API), so the destroy-shape path must be orthogonal to proxy bookkeeping.
    const ShapeHandle spare = world.CreateBoxShape(0.2f, 0.2f);
    const int32_t proxiesBeforeDestroyShape = world.GetBroadPhase()->GetProxyCount();
    world.DestroyShape(spare);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), proxiesBeforeDestroyShape);

    // 6. DestroyBody releases its proxy immediately (no step required).
    world.DestroyBody(second);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 1);
    EXPECT_EQ(world.GetBodyCount(), 1);

    // 7. Clear releases every proxy and every cached pair.
    world.Clear();
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 0);
    EXPECT_EQ(world.GetBroadPhasePairCount(), 0);

    // 8. The released slots are reusable: after a Clear the same World inserts a
    //    fresh proxy for a fresh body (no stale proxy id from the previous scene).
    const BodyHandle rebuilt = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(rebuilt, world.CreateBoxShape(0.5f, 0.5f)));
    StepFor(world, 1);
    EXPECT_EQ(world.GetBroadPhase()->GetProxyCount(), 1);
    ASSERT_NE(world.GetBody(rebuilt), nullptr);
    EXPECT_GE(world.GetBody(rebuilt)->proxyId, 0);
    EXPECT_EQ(world.GetBody(rebuilt)->handle.index, 0); // slot 0 was released by Clear
    std::printf("[world6] proxy lifecycle: no-shape body 0 proxies, attach -> 1, replace -> 1, "
                "second body -> 2, DestroyBody -> 1, Clear -> 0, rebuild -> 1; DestroyShape "
                "(unattached) left proxies untouched\n");
}

TEST(WorldBroadPhaseTest, FatAabbIsTightAabbPlusMargin) {
    World world; // default gravity: bodies move, so the AABBs are non-trivial
    const BodyHandle ground = AddGroundPlane(world);
    const BodyHandle box = AddBox(world, Vec2(0.0f, 3.0f));
    const BodyHandle circle = AddCircle(world, Vec2(2.0f, 4.0f));
    StepFor(world, 5); // Synchronize has run: the cached AABBs are fat

    const BodyHandle bodies[] = {box, circle};
    for (const BodyHandle handle : bodies) {
        const Body* body = world.GetBody(handle);
        ASSERT_NE(body, nullptr);
        ASSERT_TRUE(world.IsValidShape(body->shape));
        const Shape* shape = world.GetShape(body->shape);
        ASSERT_NE(shape, nullptr);
        const AABB tight = p2::ComputeShapeAABB(*shape, body->xf);
        AABB expected = tight;
        expected.Extend(p2::kAabbMargin);
        // Exact float equality is the contract: Synchronize recomputes the tight
        // AABB with the same expression and then calls Extend(kAabbMargin).
        EXPECT_FLOAT_EQ(body->aabb.lowerBound.x, expected.lowerBound.x);
        EXPECT_FLOAT_EQ(body->aabb.lowerBound.y, expected.lowerBound.y);
        EXPECT_FLOAT_EQ(body->aabb.upperBound.x, expected.upperBound.x);
        EXPECT_FLOAT_EQ(body->aabb.upperBound.y, expected.upperBound.y);
        // The fat AABB must contain the tight AABB.
        EXPECT_TRUE(p2::Contains(body->aabb, tight.lowerBound));
        EXPECT_TRUE(p2::Contains(body->aabb, tight.upperBound));
    }

    // Plane special case: the infinite half-space keeps the huge conservative box
    // (kPlaneAabbExtent each way); Extend only adds the (relative) margin.
    const Body* groundBody = world.GetBody(ground);
    ASSERT_NE(groundBody, nullptr);
    ASSERT_TRUE(world.IsValidShape(groundBody->shape));
    const Shape* groundShape = world.GetShape(groundBody->shape);
    ASSERT_NE(groundShape, nullptr);
    EXPECT_EQ(groundShape->type, p2::ShapeType::Plane);
    const AABB groundTight = p2::ComputeShapeAABB(*groundShape, groundBody->xf);
    EXPECT_FLOAT_EQ(groundTight.lowerBound.x, -p2::kPlaneAabbExtent);
    EXPECT_LE(groundBody->aabb.lowerBound.x, -p2::kPlaneAabbExtent);
    EXPECT_GE(groundBody->aabb.upperBound.x, p2::kPlaneAabbExtent);
    EXPECT_GE(groundBody->aabb.GetWidth(), 2.0f * p2::kPlaneAabbExtent);
    std::printf("[world6] fat AABB: box lower=(%.5f, %.5f) vs tight+margin, plane width = %.1f m, "
                "margin = %.3f m\n",
                static_cast<double>(world.GetBody(box)->aabb.lowerBound.x),
                static_cast<double>(world.GetBody(box)->aabb.lowerBound.y),
                static_cast<double>(groundBody->aabb.GetWidth()), static_cast<double>(p2::kAabbMargin));
}

TEST(WorldBroadPhaseTest, MarginWidensCandidatesButNotContacts) {
    // Two circles with a 0.15 m gap: the tight AABBs do not overlap, but the fat
    // AABBs do (2 * kAabbMargin = 0.2 m > 0.15 m). The margin may only add a broad-
    // phase candidate - the narrow phase must still report zero contacts.
    World world(Vec2(0.0f, 0.0f)); // gravity off: positions stay where we put them
    world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    const BodyHandle left = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.0f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(left, world.CreateCircleShape(0.5f)));
    const BodyHandle right = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(1.15f, 0.0f)));
    EXPECT_TRUE(world.AttachShape(right, world.CreateCircleShape(0.5f)));

    // Step 1: proxies are inserted with the tight AABBs AttachShape cached, so the
    // gap pair is not even a candidate yet.
    world.Step(kDt);
    EXPECT_EQ(world.GetBroadPhasePairCount(), 0);
    EXPECT_EQ(world.GetContactCount(), 0);

    // Step 2: Synchronize has widened both AABBs; the pair is now a candidate ...
    world.Step(kDt);
    EXPECT_EQ(world.GetBroadPhasePairCount(), 1);
    // ... but the narrow phase is exact, so the margin creates no contact.
    EXPECT_EQ(world.GetContactCount(), 0);
    const Body* leftBody = world.GetBody(left);
    const Body* rightBody = world.GetBody(right);
    ASSERT_NE(leftBody, nullptr);
    ASSERT_NE(rightBody, nullptr);
    const AABB leftTight = p2::ComputeShapeAABB(*world.GetShape(leftBody->shape), leftBody->xf);
    const AABB rightTight = p2::ComputeShapeAABB(*world.GetShape(rightBody->shape), rightBody->xf);
    EXPECT_FALSE(p2::Overlaps(leftTight, rightTight)); // the gap pair is margin-only
    EXPECT_TRUE(p2::Overlaps(leftBody->aabb, rightBody->aabb));
    EXPECT_GT(p2::kAabbMargin, 0.0f);

    // Move them to exact tangency: now the tight AABBs touch (inclusive overlap),
    // so the pair is a candidate for the physical reason and a contact appears.
    world.SetTransform(right, Vec2(1.0f, 0.0f), 0.0f);
    world.Step(kDt);
    EXPECT_GE(world.GetBroadPhasePairCount(), 1);
    EXPECT_EQ(world.GetContactCount(), 1);
    std::printf("[world6] margin vs contact: gap pair candidates step1 = 0 (tight), step2 = 1 (fat), "
                "contacts = 0; tangent pair contacts = %d\n",
                static_cast<int>(world.GetContactCount()));
}

TEST(WorldBroadPhaseTest, NonRespondingBodyTypesAreFilteredByWorld) {
    // The broad phase is body-agnostic: it reports every overlapping proxy pair,
    // including static/static, static/kinematic and kinematic/kinematic pairs.
    // World::CreateContactForPair must drop pairs where neither body is dynamic
    // (nothing could be pushed apart), while keeping every pair that has one.
    World world(Vec2(0.0f, 0.0f)); // gravity off: keep the overlap configuration
    world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
    const BodyHandle staticA = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.0f, 0.0f)));
    ASSERT_TRUE(world.AttachShape(staticA, world.CreateBoxShape(0.5f, 0.5f)));
    const BodyHandle staticB = world.CreateBody(MakeBodyDef(BodyType::Static, Vec2(0.1f, 0.0f)));
    ASSERT_TRUE(world.AttachShape(staticB, world.CreateBoxShape(0.5f, 0.5f)));
    const BodyHandle kinematic = world.CreateBody(MakeBodyDef(BodyType::Kinematic, Vec2(0.2f, 0.0f)));
    ASSERT_TRUE(world.AttachShape(kinematic, world.CreateBoxShape(0.5f, 0.5f)));
    const BodyHandle dynamic = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.3f, 0.0f)));
    ASSERT_TRUE(world.AttachShape(dynamic, world.CreateBoxShape(0.5f, 0.5f)));
    // A collider-less body at the same place must not add a proxy or a pair.
    const BodyHandle noShape = world.CreateBody(MakeBodyDef(BodyType::Dynamic, Vec2(0.4f, 0.0f)));
    ASSERT_TRUE(world.IsValid(noShape));

    world.Step(kDt);
    // 4 shaped bodies, all fat AABBs overlap -> C(4,2) = 6 broad-phase pairs. The
    // broad phase cannot know body types, so all 6 are reported.
    EXPECT_EQ(world.GetBroadPhasePairCount(), 6);
    // Contacts: only the 3 pairs that involve the dynamic box can respond.
    EXPECT_EQ(world.GetContactCount(), 3);
    for (const Contact& contact : world.GetContacts()) {
        const Body* bodyA = world.GetBody(contact.bodyA);
        const Body* bodyB = world.GetBody(contact.bodyB);
        ASSERT_NE(bodyA, nullptr);
        ASSERT_NE(bodyB, nullptr);
        EXPECT_TRUE(bodyA->IsDynamic() || bodyB->IsDynamic());
    }
    // No contact references the collider-less body.
    for (const Contact& contact : world.GetContacts()) {
        EXPECT_NE(contact.bodyA, noShape);
        EXPECT_NE(contact.bodyB, noShape);
    }
    std::printf("[world6] body-type filter: broad-phase pairs = %d (static/static + kinematic pairs "
                "included), contacts = %d (dynamic pairs only)\n",
                static_cast<int>(world.GetBroadPhasePairCount()), static_cast<int>(world.GetContactCount()));
}

} // namespace
