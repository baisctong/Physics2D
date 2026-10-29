// p2::Island — Week 7 island partition (P13 / Section 6.5), real DFS.
//
// The partition is the connected-component decomposition of the dynamic-body
// contact graph. Two important pieces of the contract live here (see Island.h):
//
//  1. CONNECTIVITY: only dynamic-dynamic contacts are edges. static/kinematic
//     bodies are boundaries - their contacts are solved with the island, but a
//     shared ground plane must NOT merge every grounded stack into one island (the
//     design document's "two boxes stacked on the ground are one island" example
//     holds through the box-box contact).
//  2. DETERMINISM: the output is canonical - islands ordered by their smallest
//     member slot, bodies ascending inside an island, contacts ascending in the
//     global contact order. The solver's grouped Gauss-Seidel then applies the
//     same updates per island in the same relative order as the flat solve, which
//     is what makes island solving BIT-IDENTICAL to the Week 6 flat solve.
//
// Implementation notes:
// - the DFS is iterative with an explicit std::vector stack: a 2000-body chain
//   must not overflow the native call stack (P13 test);
// - the dynamic-dynamic adjacency is stored as two packed arrays (forward-star:
//   head per body + next per directed edge) instead of a vector-of-vectors: one
//   O(contacts) allocation per call, no per-body heap traffic;
// - the canonical lists are produced by two O(N)/O(C) ordered passes (bodies
//   ascending, then contacts ascending) rather than sorting each island, so the
//   whole function stays O(bodies + contacts).

#include "physics2d/Solver/Island.h"

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/IDPool.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace p2 {

void BuildIslandPartition(const std::vector<Body>& bodies, const IDPool& bodyIds,
                          const std::vector<Contact>& contacts,
                          std::vector<std::vector<int32_t>>& islandBodies,
                          std::vector<std::vector<int32_t>>& islandContacts) {
    islandBodies.clear();
    islandContacts.clear();

    const int32_t bodyCount = static_cast<int32_t>(bodies.size());

    // --- 1. Build the dynamic-dynamic contact adjacency (forward-star) --------
    std::vector<int32_t> firstEdge(static_cast<std::size_t>(bodyCount), -1);
    std::vector<int32_t> edgeTo(contacts.size() * 2, -1);
    std::vector<int32_t> edgeNext(contacts.size() * 2, -1);
    int32_t edgeCount = 0;
    const auto addDirectedEdge = [&](int32_t from, int32_t to) {
        const std::size_t edge = static_cast<std::size_t>(edgeCount);
        edgeTo[edge] = to;
        edgeNext[edge] = firstEdge[static_cast<std::size_t>(from)];
        firstEdge[static_cast<std::size_t>(from)] = edgeCount;
        ++edgeCount;
    };

    for (const Contact& contact : contacts) {
        const int32_t indexA = contact.bodyA.index;
        const int32_t indexB = contact.bodyB.index;
        assert(indexA >= 0 && indexA < bodyCount && indexB >= 0 && indexB < bodyCount &&
               "BuildIslandPartition: contact references a body outside the slot array");
        if (indexA < 0 || indexA >= bodyCount || indexB < 0 || indexB >= bodyCount) {
            continue; // defensive: a corrupt contact must not index out of bounds
        }
        if (!bodyIds.IsValid(indexA) || !bodyIds.IsValid(indexB)) {
            continue; // dead slot: no island edge, the contact is ignored below too
        }
        const Body& bodyA = bodies[static_cast<std::size_t>(indexA)];
        const Body& bodyB = bodies[static_cast<std::size_t>(indexB)];
        if (bodyA.type != BodyType::Dynamic || bodyB.type != BodyType::Dynamic) {
            // static/kinematic boundary: the contact is assigned to the dynamic
            // island in the second pass, but the two bodies are NOT connected here
            // (otherwise the ground would merge every island in the scene).
            continue;
        }
        addDirectedEdge(indexA, indexB);
        addDirectedEdge(indexB, indexA);
    }

    // --- 2. DFS every component (iterative, explicit stack) -------------------
    std::vector<uint8_t> visited(static_cast<std::size_t>(bodyCount), 0);
    std::vector<int32_t> islandOfBody(static_cast<std::size_t>(bodyCount), -1);
    std::vector<int32_t> stack;
    stack.reserve(64);

    int32_t islandCount = 0;
    for (int32_t seed = 0; seed < bodyCount; ++seed) {
        if (!bodyIds.IsValid(seed)) {
            continue;
        }
        if (bodies[static_cast<std::size_t>(seed)].type != BodyType::Dynamic) {
            continue; // no-contact static/kinematic bodies are not islands either
        }
        if (visited[static_cast<std::size_t>(seed)] != 0) {
            continue;
        }
        // Seeding in ascending slot order is what makes island index order equal
        // to "order by smallest member": if an earlier member existed it would
        // have been the seed of this component already.
        visited[static_cast<std::size_t>(seed)] = 1;
        islandOfBody[static_cast<std::size_t>(seed)] = islandCount;
        stack.push_back(seed);
        while (!stack.empty()) {
            const int32_t bodyIndex = stack.back();
            stack.pop_back();
            for (int32_t edge = firstEdge[static_cast<std::size_t>(bodyIndex)]; edge >= 0;
                 edge = edgeNext[static_cast<std::size_t>(edge)]) {
                const int32_t neighbour = edgeTo[static_cast<std::size_t>(edge)];
                if (visited[static_cast<std::size_t>(neighbour)] != 0) {
                    continue;
                }
                visited[static_cast<std::size_t>(neighbour)] = 1;
                islandOfBody[static_cast<std::size_t>(neighbour)] = islandCount;
                stack.push_back(neighbour);
            }
        }
        ++islandCount;
    }

    islandBodies.resize(static_cast<std::size_t>(islandCount));
    islandContacts.resize(static_cast<std::size_t>(islandCount));

    // --- 3. Canonical body lists (ascending) ----------------------------------
    // A single ascending slot pass appends every dynamic body to its island, so
    // each islandBodies[] comes out sorted without an O(k log k) sort.
    for (int32_t bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex) {
        if (!bodyIds.IsValid(bodyIndex)) {
            continue;
        }
        if (bodies[static_cast<std::size_t>(bodyIndex)].type != BodyType::Dynamic) {
            continue;
        }
        const int32_t islandIndex = islandOfBody[static_cast<std::size_t>(bodyIndex)];
        assert(islandIndex >= 0 && "BuildIslandPartition: dynamic body was not visited");
        islandBodies[static_cast<std::size_t>(islandIndex)].push_back(bodyIndex);
    }

    // --- 4. Canonical contact lists (ascending global contact order) ----------
    // Every contact with at least one dynamic endpoint belongs to exactly that
    // dynamic body's island. For a dynamic-dynamic contact both endpoints are in
    // the same island, so one channel is enough and no duplicate can occur;
    // static-static contacts belong to no island (World never creates them).
    for (std::size_t contactIndex = 0; contactIndex < contacts.size(); ++contactIndex) {
        const Contact& contact = contacts[contactIndex];
        const int32_t indexA = contact.bodyA.index;
        const int32_t indexB = contact.bodyB.index;

        int32_t dynamicBodyIndex = -1;
        if (indexA >= 0 && indexA < bodyCount && bodyIds.IsValid(indexA) &&
            bodies[static_cast<std::size_t>(indexA)].type == BodyType::Dynamic) {
            dynamicBodyIndex = indexA;
        } else if (indexB >= 0 && indexB < bodyCount && bodyIds.IsValid(indexB) &&
                   bodies[static_cast<std::size_t>(indexB)].type == BodyType::Dynamic) {
            dynamicBodyIndex = indexB;
        }
        if (dynamicBodyIndex < 0) {
            continue;
        }
        const int32_t islandIndex = islandOfBody[static_cast<std::size_t>(dynamicBodyIndex)];
        assert(islandIndex >= 0);
        islandContacts[static_cast<std::size_t>(islandIndex)].push_back(static_cast<int32_t>(contactIndex));
    }
}

} // namespace p2
