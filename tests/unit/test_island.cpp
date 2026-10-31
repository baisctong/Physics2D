// Week 7 P13 — p2::Island (island partition) contract tests.
//
// The partition is the foundation of whole-island sleeping and grouped solving, so
// its contract is pinned independently of World:
//  - dynamic-dynamic contacts are edges; static/kinematic bodies are boundaries
//    (their contacts are solved with the island but do not connect islands);
//  - every dynamic body belongs to exactly one island; no-contact dynamic bodies
//    are singleton islands (their sleep timer needs a home);
//  - the output is canonical: islands ordered by smallest member slot, bodies
//    ascending, contacts in ascending global contact order. That order is what
//    makes the grouped solve bit-identical to the flat one;
//  - the DFS is iterative: a 2000-body chain must not overflow the call stack;
//  - dead slots and output vectors that already contain data are handled.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Handle.h"
#include "physics2d/Core/IDPool.h"
#include "physics2d/Solver/Island.h"
#include <cstddef>

namespace {

using p2::Body;
using p2::BodyHandle;
using p2::BodyType;
using p2::Contact;
using p2::IDPool;

// Minimal body/contact pool for partition tests: the DFS only reads body type and
// liveness, so no shapes/mass properties are needed.
class BodySlotPool {
  public:
    int32_t Add(BodyType type) {
        const int32_t index = m_ids.Allocate();
        if (index >= static_cast<int32_t>(m_bodies.size())) {
            m_bodies.emplace_back();
        }
        Body& body = m_bodies[static_cast<size_t>(index)];
        body = Body{};
        body.type = type;
        body.handle = BodyHandle{index, m_ids.GetGeneration(index)};
        return index;
    }

    void Remove(int32_t index) { m_ids.Release(index); }

    [[nodiscard]] BodyHandle Handle(int32_t index) const {
        return BodyHandle{index, m_ids.GetGeneration(index)};
    }

    [[nodiscard]] const std::vector<Body>& Bodies() const { return m_bodies; }
    [[nodiscard]] const IDPool& Ids() const { return m_ids; }

  private:
    std::vector<Body> m_bodies;
    IDPool m_ids;
};

Contact MakeContact(BodyHandle bodyA, BodyHandle bodyB) {
    Contact contact;
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    // The partition only looks at the body pair; an empty manifold is fine.
    contact.manifold.pointCount = 0;
    return contact;
}

void Build(const BodySlotPool& pool, const std::vector<Contact>& contacts,
           std::vector<std::vector<int32_t>>& islandBodies,
           std::vector<std::vector<int32_t>>& islandContacts) {
    p2::BuildIslandPartition(pool.Bodies(), pool.Ids(), contacts, islandBodies, islandContacts);
}

using Partition = std::vector<std::vector<int32_t>>;

void ExpectPartition(const Partition& actual, const Partition& expected, const char* what) {
    ASSERT_EQ(actual.size(), expected.size()) << what;
    for (size_t islandIndex = 0; islandIndex < expected.size(); ++islandIndex) {
        EXPECT_EQ(actual[islandIndex], expected[islandIndex]) << what << " island " << islandIndex;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Basic connectivity: three separate stacks (each dynamic box pair + ground).
// ---------------------------------------------------------------------------

TEST(IslandTest, ThreeSeparateStacksFormThreeIslands) {
    BodySlotPool pool;
    const int32_t ground = pool.Add(BodyType::Static); // 0
    const int32_t box1 = pool.Add(BodyType::Dynamic);  // 1
    const int32_t box2 = pool.Add(BodyType::Dynamic);  // 2
    const int32_t box3 = pool.Add(BodyType::Dynamic);  // 3
    const int32_t box4 = pool.Add(BodyType::Dynamic);  // 4
    const int32_t box5 = pool.Add(BodyType::Dynamic);  // 5
    const int32_t box6 = pool.Add(BodyType::Dynamic);  // 6

    std::vector<Contact> contacts{
        MakeContact(pool.Handle(ground), pool.Handle(box1)), // 0
        MakeContact(pool.Handle(box1), pool.Handle(box2)),   // 1 (box-box edge)
        MakeContact(pool.Handle(ground), pool.Handle(box3)), // 2
        MakeContact(pool.Handle(box3), pool.Handle(box4)),   // 3
        MakeContact(pool.Handle(ground), pool.Handle(box5)), // 4
        MakeContact(pool.Handle(box5), pool.Handle(box6)),   // 5
    };

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    // Ground is a boundary: it appears in every island's contact list (twice, for
    // the box1/box3 stacks... actually once per stack) but in no island body list.
    ExpectPartition(islandBodies, Partition{{box1, box2}, {box3, box4}, {box5, box6}}, "body partition");
    ExpectPartition(islandContacts, Partition{{0, 1}, {2, 3}, {4, 5}}, "contact partition");
    for (const std::vector<int32_t>& members : islandBodies) {
        EXPECT_EQ(std::find(members.begin(), members.end(), ground), members.end())
            << "static boundary body must not join an island";
    }
    std::printf("[island] three stacks: %d islands, bodies={%d,%d},{%d,%d},{%d,%d}\n",
                static_cast<int>(islandBodies.size()), box1, box2, box3, box4, box5, box6);
}

TEST(IslandTest, BridgeBodyMergesTwoStacksIntoOneIsland) {
    BodySlotPool pool;
    const int32_t ground = pool.Add(BodyType::Static);  // 0
    const int32_t boxA1 = pool.Add(BodyType::Dynamic);  // 1
    const int32_t boxA2 = pool.Add(BodyType::Dynamic);  // 2
    const int32_t boxB1 = pool.Add(BodyType::Dynamic);  // 3
    const int32_t boxB2 = pool.Add(BodyType::Dynamic);  // 4
    const int32_t bridge = pool.Add(BodyType::Dynamic); // 5

    std::vector<Contact> contacts{
        MakeContact(pool.Handle(ground), pool.Handle(boxA1)), // 0
        MakeContact(pool.Handle(boxA1), pool.Handle(boxA2)),  // 1
        MakeContact(pool.Handle(ground), pool.Handle(boxB1)), // 2
        MakeContact(pool.Handle(boxB1), pool.Handle(boxB2)),  // 3
        MakeContact(pool.Handle(boxA2), pool.Handle(bridge)), // 4
        MakeContact(pool.Handle(bridge), pool.Handle(boxB2)), // 5
    };

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    ExpectPartition(islandBodies, Partition{{boxA1, boxA2, boxB1, boxB2, bridge}}, "merged body partition");
    ExpectPartition(islandContacts, Partition{{0, 1, 2, 3, 4, 5}}, "merged contact partition");
    std::printf("[island] bridge: merged into %d island with %d bodies and %d contacts\n",
                static_cast<int>(islandBodies.size()), static_cast<int>(islandBodies[0].size()),
                static_cast<int>(islandContacts[0].size()));
}

TEST(IslandTest, StaticAndKinematicBodiesAreBoundariesNotConnectivity) {
    // Two independent dynamic boxes both touch the same static ground and the same
    // kinematic platform. If either boundary transmitted connectivity the two
    // boxes would merge into one island; they must stay two.
    BodySlotPool pool;
    const int32_t ground = pool.Add(BodyType::Static);      // 0
    const int32_t platform = pool.Add(BodyType::Kinematic); // 1
    const int32_t boxA = pool.Add(BodyType::Dynamic);       // 2
    const int32_t boxB = pool.Add(BodyType::Dynamic);       // 3

    std::vector<Contact> contacts{
        MakeContact(pool.Handle(ground), pool.Handle(boxA)),   // 0
        MakeContact(pool.Handle(platform), pool.Handle(boxA)), // 1
        MakeContact(pool.Handle(ground), pool.Handle(boxB)),   // 2
        MakeContact(pool.Handle(platform), pool.Handle(boxB)), // 3
    };

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    ExpectPartition(islandBodies, Partition{{boxA}, {boxB}}, "boundary body partition");
    // Each island owns its two boundary contacts, in global contact order.
    ExpectPartition(islandContacts, Partition{{0, 1}, {2, 3}}, "boundary contact partition");
    for (const std::vector<int32_t>& members : islandBodies) {
        EXPECT_EQ(std::find(members.begin(), members.end(), ground), members.end());
        EXPECT_EQ(std::find(members.begin(), members.end(), platform), members.end());
    }
    // The design document's "two boxes stacked on the ground are one island" holds
    // through the box-box contact, not through the shared ground.
    std::printf("[island] boundaries: static+kinematic shared, still %d islands\n",
                static_cast<int>(islandBodies.size()));
}

// ---------------------------------------------------------------------------
// Scale: 2000-body chain, one island, explicit stack (no recursion).
// ---------------------------------------------------------------------------

TEST(IslandTest, TwoThousandBodyChainIsOneIslandWithoutStackOverflow) {
    constexpr int32_t kBodyCount = 2000;
    BodySlotPool pool;
    std::vector<int32_t> bodies;
    bodies.reserve(static_cast<size_t>(kBodyCount));
    for (int32_t index = 0; index < kBodyCount; ++index) {
        bodies.push_back(pool.Add(BodyType::Dynamic));
    }
    std::vector<Contact> contacts;
    contacts.reserve(static_cast<size_t>(kBodyCount - 1));
    for (int32_t index = 0; index + 1 < kBodyCount; ++index) {
        contacts.push_back(MakeContact(pool.Handle(bodies[static_cast<size_t>(index)]),
                                       pool.Handle(bodies[static_cast<size_t>(index + 1)])));
    }

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    ASSERT_EQ(islandBodies.size(), 1u);
    ASSERT_EQ(islandBodies[0].size(), static_cast<size_t>(kBodyCount));
    for (int32_t index = 0; index < kBodyCount; ++index) {
        EXPECT_EQ(islandBodies[0][static_cast<size_t>(index)], index) << "body list must be ascending";
    }
    ASSERT_EQ(islandContacts[0].size(), static_cast<size_t>(kBodyCount - 1));
    for (int32_t index = 0; index + 1 < kBodyCount; ++index) {
        EXPECT_EQ(islandContacts[0][static_cast<size_t>(index)], index) << "contact list must be ascending";
    }
    std::printf("[island] 2000-body chain: 1 island, %d bodies, %d contacts (explicit stack)\n",
                static_cast<int>(islandBodies[0].size()), static_cast<int>(islandContacts[0].size()));
}

// ---------------------------------------------------------------------------
// Canonical ordering: islands by smallest member, contacts by global index.
// ---------------------------------------------------------------------------

TEST(IslandTest, PartitionIsCanonicalRegardlessOfContactOrder) {
    BodySlotPool pool;
    // Island A: bodies 0 and 5 (min 0). Island B: bodies 1, 2, 3 (min 1).
    const int32_t body0 = pool.Add(BodyType::Dynamic); // 0
    const int32_t body1 = pool.Add(BodyType::Dynamic); // 1
    const int32_t body2 = pool.Add(BodyType::Dynamic); // 2
    const int32_t body3 = pool.Add(BodyType::Dynamic); // 3
    const int32_t body4 = pool.Add(BodyType::Static);  // 4 (boundary, never joins)
    const int32_t body5 = pool.Add(BodyType::Dynamic); // 5

    // Contacts deliberately scrambled relative to island order: island B's first
    // contact is global 0, island A's contact is global 1, ...
    std::vector<Contact> contacts{
        MakeContact(pool.Handle(body2), pool.Handle(body3)), // 0 -> island B
        MakeContact(pool.Handle(body0), pool.Handle(body5)), // 1 -> island A
        MakeContact(pool.Handle(body1), pool.Handle(body2)), // 2 -> island B
        MakeContact(pool.Handle(body4), pool.Handle(body3)), // 3 -> island B (boundary)
    };

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    // Canonical: island A (min member 0) first, island B (min member 1) second.
    ExpectPartition(islandBodies, Partition{{body0, body5}, {body1, body2, body3}}, "canonical bodies");
    ExpectPartition(islandContacts, Partition{{1}, {0, 2, 3}}, "canonical contacts");
    std::printf("[island] canonical: islands={{%d,%d},{%d,%d,%d}} contacts={{1},{0,2,3}}\n", body0, body5,
                body1, body2, body3);
}

// ---------------------------------------------------------------------------
// Edge cases: empty scene, only static bodies, singleton islands, dead slots.
// ---------------------------------------------------------------------------

TEST(IslandTest, EmptySceneHasNoIslands) {
    BodySlotPool pool;
    Partition islandBodies{{1, 2, 3}}; // pre-filled: outputs must be cleared
    Partition islandContacts{{4, 5}};
    Build(pool, {}, islandBodies, islandContacts);
    EXPECT_TRUE(islandBodies.empty());
    EXPECT_TRUE(islandContacts.empty());
}

TEST(IslandTest, StaticOnlySceneHasNoIslands) {
    BodySlotPool pool;
    const int32_t ground = pool.Add(BodyType::Static);
    const int32_t wall = pool.Add(BodyType::Static);
    std::vector<Contact> contacts{MakeContact(pool.Handle(ground), pool.Handle(wall))};

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);
    EXPECT_TRUE(islandBodies.empty());
    EXPECT_TRUE(islandContacts.empty());
}

TEST(IslandTest, NoContactDynamicBodiesAreSingletonIslands) {
    BodySlotPool pool;
    const int32_t first = pool.Add(BodyType::Dynamic);
    pool.Add(BodyType::Static); // static slot in between must be skipped
    const int32_t second = pool.Add(BodyType::Dynamic);

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, {}, islandBodies, islandContacts);

    ExpectPartition(islandBodies, Partition{{first}, {second}}, "singleton islands");
    EXPECT_EQ(islandContacts.size(), 2u);
    EXPECT_TRUE(islandContacts[0].empty());
    EXPECT_TRUE(islandContacts[1].empty());
}

TEST(IslandTest, DeadSlotsAreIgnored) {
    BodySlotPool pool;
    const int32_t removed = pool.Add(BodyType::Dynamic); // 0: will be released
    const int32_t first = pool.Add(BodyType::Dynamic);   // 1
    const int32_t second = pool.Add(BodyType::Dynamic);  // 2
    pool.Remove(removed);

    std::vector<Contact> contacts{MakeContact(pool.Handle(first), pool.Handle(second))};

    Partition islandBodies;
    Partition islandContacts;
    Build(pool, contacts, islandBodies, islandContacts);

    ExpectPartition(islandBodies, Partition{{first, second}}, "dead slot partition");
    ExpectPartition(islandContacts, Partition{{0}}, "dead slot contacts");
    for (const std::vector<int32_t>& members : islandBodies) {
        EXPECT_EQ(std::find(members.begin(), members.end(), removed), members.end());
    }
    std::printf("[island] dead slot %d ignored; one island {%d,%d}\n", removed, first, second);
}
