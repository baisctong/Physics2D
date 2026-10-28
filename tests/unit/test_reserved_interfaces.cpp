// Week 1 reserved-interface compile + behaviour tests (D6/P05).
//
// This translation unit includes EVERY public physics2d header at the top, in
// one place: compiling it proves that the frozen interfaces are usable and that
// the headers are self-contained. The runtime assertions then exercise the parts
// that are already meaningful in Week 1 (broad-phase interface, contact feature
// ids, manifold/contact data layout), while the shape/solver implementations
// themselves stay reserved for later weeks.

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/Manifold.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/BroadPhaseInterface.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/IDPool.h"
#include "physics2d/Core/Profiler.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/TimeStep.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Mat2.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"
#include "physics2d/Math/Transform.h"
#include "physics2d/Math/Vec2.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

// Minimal, fully functional broad phase. Its purpose is to prove that
// IBroadPhase can be implemented and owned by World exactly as documented in
// BroadPhaseInterface.h (Week 6 replaces this with BruteForce and SAP).
class DummyBroadPhase final : public p2::IBroadPhase {
  public:
    int32_t AddProxy(const p2::AABB& aabb) override {
        const int32_t proxyId = static_cast<int32_t>(m_aabbs.size());
        m_aabbs.push_back(aabb);
        m_alive.push_back(true);
        return proxyId;
    }

    void RemoveProxy(int32_t proxyId) override {
        if (!IsKnownProxy(proxyId)) {
            return;
        }
        m_alive[static_cast<std::size_t>(proxyId)] = false;
    }

    void UpdateAABB(int32_t proxyId, const p2::AABB& aabb) override {
        if (!IsKnownProxy(proxyId)) {
            return;
        }
        m_aabbs[static_cast<std::size_t>(proxyId)] = aabb;
    }

    void QueryPairs(const p2::PairCallback& callback) override {
        const int32_t proxyCount = static_cast<int32_t>(m_aabbs.size());
        for (int32_t a = 0; a < proxyCount; ++a) {
            if (!m_alive[static_cast<std::size_t>(a)]) {
                continue;
            }
            for (int32_t b = a + 1; b < proxyCount; ++b) {
                if (!m_alive[static_cast<std::size_t>(b)]) {
                    continue;
                }
                const p2::AABB& boxA = m_aabbs[static_cast<std::size_t>(a)];
                const p2::AABB& boxB = m_aabbs[static_cast<std::size_t>(b)];
                if (p2::Overlaps(boxA, boxB)) {
                    // The interface allows either order; this implementation
                    // normalises to a < b for determinism.
                    callback(a, b);
                }
            }
        }
    }

    [[nodiscard]] int32_t GetProxyCount() const override {
        int32_t aliveCount = 0;
        for (const bool alive : m_alive) {
            if (alive) {
                ++aliveCount;
            }
        }
        return aliveCount;
    }

    [[nodiscard]] const char* Name() const override { return "DummyBroadPhase"; }

  private:
    [[nodiscard]] bool IsKnownProxy(int32_t proxyId) const {
        return proxyId >= 0 && proxyId < static_cast<int32_t>(m_aabbs.size());
    }

    std::vector<p2::AABB> m_aabbs;
    std::vector<bool> m_alive;
};

TEST(ReservedInterfacesTest, BroadPhaseInterfaceIsImplementableAndUsable) {
    DummyBroadPhase broadPhase;
    EXPECT_STREQ(broadPhase.Name(), "DummyBroadPhase");

    const p2::Vec2 zero(0.0f, 0.0f);
    const int32_t proxyA = broadPhase.AddProxy(p2::AABB{zero, p2::Vec2(1.0f, 1.0f)});
    const int32_t proxyB = broadPhase.AddProxy(p2::AABB{p2::Vec2(0.5f, 0.5f), p2::Vec2(1.5f, 1.5f)});
    const int32_t proxyC = broadPhase.AddProxy(p2::AABB{p2::Vec2(5.0f, 5.0f), p2::Vec2(6.0f, 6.0f)});
    EXPECT_EQ(proxyA, 0);
    EXPECT_EQ(proxyB, 1);
    EXPECT_EQ(proxyC, 2);
    EXPECT_EQ(broadPhase.GetProxyCount(), 3);

    std::vector<std::pair<int32_t, int32_t>> pairs;
    const auto collectPairs = [&pairs](int32_t a, int32_t b) { pairs.emplace_back(a, b); };
    broadPhase.QueryPairs(collectPairs);
    // A and B overlap on [0.5, 1] x [0.5, 1]; C is far away.
    ASSERT_EQ(pairs.size(), 1U);
    EXPECT_EQ(pairs[0].first, proxyA);
    EXPECT_EQ(pairs[0].second, proxyB);

    // Removing B removes the pair; the removed proxy must not be reported again.
    broadPhase.RemoveProxy(proxyB);
    EXPECT_EQ(broadPhase.GetProxyCount(), 2);
    pairs.clear();
    broadPhase.QueryPairs(collectPairs);
    EXPECT_TRUE(pairs.empty());

    // Moving C on top of A creates a new pair (UpdateAABB path).
    broadPhase.UpdateAABB(proxyC, p2::AABB{p2::Vec2(0.25f, 0.25f), p2::Vec2(0.75f, 0.75f)});
    pairs.clear();
    broadPhase.QueryPairs(collectPairs);
    ASSERT_EQ(pairs.size(), 1U);
    EXPECT_EQ(pairs[0].first, proxyA);
    EXPECT_EQ(pairs[0].second, proxyC);
}

TEST(ReservedInterfacesTest, WorldOwnsPluggableBroadPhase) {
    p2::World world;
    EXPECT_EQ(world.GetBroadPhase(), nullptr);

    auto broadPhase = std::make_unique<DummyBroadPhase>();
    DummyBroadPhase* rawPointer = broadPhase.get();
    world.SetBroadPhase(std::move(broadPhase));
    ASSERT_NE(world.GetBroadPhase(), nullptr);
    EXPECT_EQ(world.GetBroadPhase(), rawPointer);
    EXPECT_STREQ(world.GetBroadPhase()->Name(), "DummyBroadPhase");

    // Passing nullptr disables broad-phase work, as documented in World.h.
    world.SetBroadPhase(nullptr);
    EXPECT_EQ(world.GetBroadPhase(), nullptr);
}

TEST(ReservedInterfacesTest, ContactIDPacksFeatureBits) {
    p2::ContactID id;
    id.referenceEdge = 3;  // bits 0-3 -> 0x003
    id.incidentVertex = 5; // bits 4-7 -> 5 << 4 = 0x050
    id.type = 2;           // bits 8-9 -> 2 << 8 = 0x200
    id.flip = 1;           // bit 10   -> 1 << 10 = 0x400
    // Hand-computed: 0x400 | 0x200 | 0x050 | 0x003 = 0x653.
    EXPECT_EQ(id.Key(), 0x653U);

    // Each field is masked to its documented width before packing.
    p2::ContactID saturated;
    saturated.referenceEdge = 0xFF;  // & 0x0F -> 0x00F
    saturated.incidentVertex = 0xFF; // & 0x0F -> 0x0F0
    saturated.type = 0xFF;           // & 0x03 -> 0x300
    saturated.flip = 0xFF;           // & 0x01 -> 0x400
    EXPECT_EQ(saturated.Key(), 0x7FFU);

    // Equality is defined on Key(), so extra high bits that are masked away do
    // not distinguish two ids (the collision code only fills valid ranges).
    p2::ContactID maskedOverflow = id;
    maskedOverflow.referenceEdge = 0x13; // & 0x0F == 3
    EXPECT_EQ(maskedOverflow, id);
    EXPECT_EQ(maskedOverflow.Key(), id.Key());

    // The swap bit is part of the key: a flipped contact is a different feature.
    p2::ContactID flipped = id;
    flipped.flip = 0;
    EXPECT_NE(flipped, id);
    EXPECT_NE(flipped.Key(), id.Key());

    p2::ContactID referenceEdgeChanged = id;
    referenceEdgeChanged.referenceEdge = 4;
    EXPECT_NE(referenceEdgeChanged, id);
}

TEST(ReservedInterfacesTest, ManifoldDataLayoutIsUsable) {
    p2::Manifold manifold;
    // Documented defaults: normal points from A to B, nothing touches yet.
    EXPECT_EQ(manifold.normal, p2::Vec2(0.0f, 1.0f));
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
    EXPECT_EQ(manifold.pointCount, 0);
    EXPECT_FALSE(manifold.IsTouching());

    manifold.normal = p2::Vec2(0.0f, -1.0f);
    manifold.penetration = 0.01f;
    manifold.points[0].localPointA = p2::Vec2(1.0f, 0.0f);
    manifold.points[0].localPointB = p2::Vec2(2.0f, 0.0f);
    manifold.points[0].normalImpulse = 5.0f;
    manifold.points[0].tangentImpulse = -0.5f;
    manifold.points[0].id.referenceEdge = 1;
    manifold.points[1].localPointA = p2::Vec2(-1.0f, 0.0f);
    manifold.pointCount = p2::kMaxManifoldPoints;
    EXPECT_TRUE(manifold.IsTouching());
    // A 2D polygon contact has at most two points (Section 4.4 / P06).
    EXPECT_EQ(p2::kMaxManifoldPoints, 2);
    EXPECT_EQ(p2::kMaxPolygonVertices, 8);
    EXPECT_FLOAT_EQ(manifold.points[0].normalImpulse, 5.0f);

    manifold.Clear();
    EXPECT_EQ(manifold.pointCount, 0);
    EXPECT_FLOAT_EQ(manifold.penetration, 0.0f);
    EXPECT_FALSE(manifold.IsTouching());
}

TEST(ReservedInterfacesTest, ContactDataLayoutIsUsable) {
    p2::Contact contact;
    EXPECT_FALSE(contact.bodyA.IsValid());
    EXPECT_FALSE(contact.bodyB.IsValid());
    EXPECT_FALSE(contact.manifold.IsTouching());
    EXPECT_FALSE(contact.isPersistent);
    EXPECT_FLOAT_EQ(contact.friction, 0.0f);
    EXPECT_FLOAT_EQ(contact.restitution, 0.0f);

    contact.bodyA = p2::BodyHandle{0, 1U};
    contact.bodyB = p2::BodyHandle{1, 1U};
    contact.manifold.pointCount = 1;
    contact.manifold.points[0].id.type = 0;
    contact.friction = std::sqrt(0.3f * 0.5f); // documented geometric mean
    contact.restitution = 0.5f;                // documented max(A, B)
    contact.isPersistent = true;

    EXPECT_TRUE(contact.bodyA.IsValid());
    EXPECT_TRUE(contact.bodyB.IsValid());
    EXPECT_NE(contact.bodyA, contact.bodyB);
    EXPECT_TRUE(contact.manifold.IsTouching());
    EXPECT_TRUE(contact.isPersistent);
}

TEST(ReservedInterfacesTest, HandleIsCheapGenerationCheckedValueType) {
    static_assert(std::is_trivially_copyable_v<p2::BodyHandle>);
    static_assert(std::is_trivially_copyable_v<p2::ShapeHandle>);

    const p2::BodyHandle invalid;
    EXPECT_FALSE(invalid.IsValid());
    EXPECT_EQ(invalid.index, -1);
    EXPECT_EQ(invalid.generation, 0U);

    const p2::BodyHandle handle{2, 7U};
    EXPECT_EQ(handle, (p2::BodyHandle{2, 7U}));
    EXPECT_NE(handle, (p2::BodyHandle{2, 8U})); // generation differs -> stale
    EXPECT_NE(handle, (p2::BodyHandle{3, 7U})); // index differs -> other slot
    // The hash is for tooling only, but it must be consistent with equality.
    EXPECT_EQ(p2::BodyHandleHash{}(handle), p2::BodyHandleHash{}(p2::BodyHandle{2, 7U}));
}

TEST(ReservedInterfacesTest, ProfilerMacrosAreNoOpsWhenTracyIsDisabled) {
#ifndef TRACY_ENABLE
    int value = 0;
    P2_PROFILE_ZONE();
    P2_PROFILE_ZONE_N("week1");
    P2_PROFILE_FRAME_MARK();
    P2_PROFILE_PLOT("value", value);
    P2_PROFILE_MESSAGE("no-op");
    EXPECT_EQ(value, 0);
#else
    GTEST_SKIP() << "Tracy is enabled; the no-op semantics are not under test.";
#endif
}

TEST(ReservedInterfacesTest, AllPublicHeadersCompileTogether) {
    // The value of this test is the translation unit itself: every public header
    // is included at the top of this file. These checks keep gtest output useful.
    p2::World world;
    EXPECT_EQ(world.GetBodyCount(), 0);
    const p2::TimeStep timeStep;
    EXPECT_FLOAT_EQ(timeStep.dt, p2::kFixedTimeStep);
    EXPECT_FLOAT_EQ(timeStep.invDt * timeStep.dt, 1.0f);
    EXPECT_EQ(timeStep.velocityIterations, p2::kVelocityIterations);
    EXPECT_EQ(timeStep.positionIterations, p2::kPositionIterations);
}

} // namespace
