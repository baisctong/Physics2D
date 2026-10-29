#pragma once

// p2::Island — Week 7 island partition of the dynamic-body contact graph (P13,
// Section 6.5).
//
// WHY ISLANDS
// -----------
// The contact graph splits the dynamic bodies into independent components: two
// bodies in different islands share no contact, so the sequential impulses solved
// for one island cannot change any velocity of the other. That independence is
// what makes three things possible:
//   1. per-island solving with the island's velocity state packed into a
//      contiguous scratch array (cache locality; design document 3.3/6.5);
//   2. whole-island atomic sleeping — a sleeping island can never be half awake,
//      which would freeze one body while its neighbour keeps moving and open the
//      classic "island boundary" clipping hole;
//   3. skipping all work (AABB refresh, narrow phase, solving) for sleeping
//      islands while keeping their broad-phase proxies alive.
//
// The independence is also why the island-grouped solve is BIT-IDENTICAL to the
// flat Week 6 solve: constraints from different islands never touch the same
// body, so their Gauss-Seidel updates commute. Only the relative order of the
// contacts INSIDE one island must be preserved, which the canonical ordering
// below guarantees.
//
// CONNECTIVITY RULES (normative)
// ------------------------------
// - only DYNAMIC bodies are island members;
// - a contact between two dynamic bodies is an edge of the graph;
// - static and kinematic bodies are BOUNDARIES: their contacts are still solved
//   with the island (the ground pushes the box), but they neither join an island
//   nor connect two islands. If a shared ground plane connected islands, every
//   grounded stack in a scene would merge into one island and islands/sleeping
//   would stop working. The design document's "two boxes stacked on the ground
//   are one island" example holds through the box-box contact, not the ground.
// - every dynamic body belongs to exactly one island; a body without contacts
//   forms a singleton island so its sleeping timer still works.
//
// DETERMINISM
// -----------
// The partition is canonical:
// - islands are ordered by their smallest member body slot index;
// - `islandBodies[i]` is ascending;
// - `islandContacts[i]` is ascending in the GLOBAL contact order (relative order
//   inside the island == relative order in World::m_contacts), which is what
//   keeps the grouped solve bit-identical to the flat one.
//
// USAGE
// -----
// World::BuildIslands calls BuildIslandPartition with its body/contact storage,
// stores the two arrays, passes them to ContactSolver::SetIslandPartition and
// uses them in UpdateSleep / wake propagation. The two output arrays are plain
// std::vector<std::vector<int32_t>> on purpose: World.h can keep them as
// members without including a Solver header (Core must not depend on Solver).

#include <cstdint>
#include <vector>

namespace p2 {

struct Body;
struct Contact;
class IDPool;

// Builds the canonical island partition of the live dynamic bodies. `bodyIds` is
// the World's liveness pool (slot array contains dead slots). Both outputs are
// cleared first.
//
// Implementation contract (P13):
// - iterative DFS with an explicit stack; a 2000-body chain must not overflow the
//   call stack (no recursion);
// - contacts whose two bodies are dynamic are edges; static/kinematic bodies are
//   boundaries and never join an island;
// - O(bodies + contacts) expected.
void BuildIslandPartition(const std::vector<Body>& bodies, const IDPool& bodyIds,
                          const std::vector<Contact>& contacts,
                          std::vector<std::vector<int32_t>>& islandBodies,
                          std::vector<std::vector<int32_t>>& islandContacts);

} // namespace p2
