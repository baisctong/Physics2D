#pragma once

// p2::World — owns bodies and runs the simulation pipeline (Section 4.6).
//
// ============================================================================
// WORK INTERFACE MAP (contract Weeks 2-8 build on)
// ============================================================================
// The Step() sequence below is fixed by Section 4.6 of the design document and
// is implemented in its final shape, with the not-yet-landed phases present as
// empty private methods. Later weeks only fill in bodies of those methods:
//
//   Phase                            Week   Method
//   ------------------------------   ----   -------------------------------
//   1. integrate forces -> velocity    1    IntegrateForces()      [done]
//   2. broad phase update              6    UpdateBroadPhase(dt)   [done W6]
//   3. narrow phase / contacts         2/6  UpdateContacts()       [done W2; W6 pair source = broad phase]
//   4. island construction             7    BuildIslands()         [done W7]
//      solver preparation             3/5  PrepareConstraints()   [done W5: friction + warm-start seeds]
//   5. warm start                      5    WarmStart()            [done W5]
//   6. velocity iterations             3/5/7 SolveVelocityConstraints() [done W7: island-grouped, packed]
//   7. integrate velocity -> position   1   IntegratePositions()   [done]
//   8. position iterations (split imp) 5    SolvePositionConstraints(dt) [done W5: pseudo-velocity layer]
//   9. synchronise AABB/transform      2/6  Synchronize()          [done W2; W6 adds the fat margin]
//   10. sleep evaluation               7    UpdateSleep()          [done W7]
//   11. debug validation               1    Validate()             [done]
//
// Stable from Week 2 on:
// - Shape pool: one shape per body (Body::shape), created via Create*Shape and
//   attached with AttachShape (which derives mass/inertia from the density).
// - Contact list: World owns it (narrow-phase output), rebuilt every frame by
//   UpdateContacts(). Week 6: the candidate pairs come from the installed
//   IBroadPhase (or from the O(N^2) fallback when none is installed) and are
//   canonicalised to (i < j, sorted, deduped) so the contact list - and
//   therefore the Gauss-Seidel solve order - is bit-identical to the pre-W6
//   enumeration. Only the enumeration changed, never the Contact data contract.
// - Persistence: per-point accumulated impulses are inherited across frames by
//   ContactID::Key() in MatchAndInheritImpulses; the Week 5 solver consumes them
//   for warm starting, so this mapping is now on the hot path (do not weaken it).
// - Solver: ContactSolver (Solver/ContactSolver.h) is owned via unique_ptr and
//   only forward-declared here, so Core does not include Solver headers. The
//   World prepares constraints once per step, runs warm starting + velocityIterations
//   sequential impulses (normal + friction), integrates positions, then runs the
//   split-impulse position iterations with the same dt (phase 8).
// - Remaining temporary response: none. The Week 2 position push-out was replaced
//   by the split-impulse pseudo-velocity layer in Week 5 (useSplitImpulse=false
//   switches to the Baumgarte bias for the comparison experiment, not back to the
//   push-out).
//
// Why define the empty phases early: inserting a phase in the middle of Step()
// later means re-auditing every ordering assumption. With the skeleton in
// place, a week's work is "fill one method + one test", and the ordering can
// never regress.
//
// Ownership: World owns bodies in one contiguous vector indexed by
// BodyHandle::index. Handles are validated against IDPool generations, so a
// destroyed body's handle goes stale instead of aliasing a new body. Bodies
// are never relocated in a way that changes an index while alive (the planned
// Week 4 hot/cold split keeps the same index space).
// ============================================================================

#include "physics2d/Collision/Contact.h"
#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/BroadPhaseInterface.h"
#include "physics2d/Core/IDPool.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/Shape.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace p2 {

// Forward-declared: World owns the solver but Core headers must not pull in the
// Solver module's implementation headers (see the work interface map above).
class ContactSolver;

class World {
  public:
    explicit World(Vec2 gravity = Vec2(0.0f, kDefaultGravityY));
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) = delete;
    World& operator=(World&&) = delete;

    // ------------------------------------------------------------------
    // Body lifecycle
    // ------------------------------------------------------------------

    // Returns an invalid handle if `def` is rejected (non-finite transform,
    // non-positive dynamic mass/inertia, ...). See Section 3.4: the engine never
    // silently repairs bad input.
    [[nodiscard]] BodyHandle CreateBody(const BodyDef& def);
    void DestroyBody(BodyHandle handle);

    [[nodiscard]] bool IsValid(BodyHandle handle) const;

    // Debug/internal access. Prefer the primitive accessors below: they are the
    // stable API across the Week 4 hot/cold split. Returns nullptr for stale
    // handles.
    [[nodiscard]] const Body* GetBody(BodyHandle handle) const;

    // ------------------------------------------------------------------
    // Shapes (Week 2). World owns a shape pool; a body references one shape.
    // ------------------------------------------------------------------

    // Invalid input (non-positive radius, non-finite values) is rejected with an
    // invalid handle plus a debug assert - never silently repaired.
    [[nodiscard]] ShapeHandle CreateCircleShape(float radius, Vec2 localCenter = Vec2(0.0f, 0.0f));
    [[nodiscard]] ShapeHandle CreatePlaneShape(Vec2 normal, float offset = 0.0f);

    // Week 4: convex polygon from a CCW vertex array / axis-aligned box centred on
    // the body origin. Invalid input (too few/many vertices, duplicate, collinear,
    // non-convex or clockwise winding) returns an invalid handle plus a debug
    // assert; see PolygonShape.h for the full validation contract.
    [[nodiscard]] ShapeHandle CreatePolygonShape(const Vec2* vertices, int32_t count);
    [[nodiscard]] ShapeHandle CreateBoxShape(float halfWidth, float halfHeight);

    // Attaches (or replaces) the body's shape and recomputes mass/inertia from
    // density * shape mass properties (parallel-axis transfer to the body
    // origin). A PlaneShape is only valid on a Static body (infinite mass);
    // attaching it to a kinematic/dynamic body fails.
    [[nodiscard]] bool AttachShape(BodyHandle body, ShapeHandle shape);

    // Releases an unattached shape slot. A shape attached to a body must be
    // detached first (destroy the body instead); misuse trips a debug assert.
    void DestroyShape(ShapeHandle shape);

    [[nodiscard]] bool IsValidShape(ShapeHandle shape) const;
    [[nodiscard]] const Shape* GetShape(ShapeHandle shape) const;

    // ------------------------------------------------------------------
    // Stable state accessors (layout-independent public API)
    // ------------------------------------------------------------------
    [[nodiscard]] Vec2 GetPosition(BodyHandle handle) const;
    [[nodiscard]] Transform GetTransform(BodyHandle handle) const;
    void SetTransform(BodyHandle handle, Vec2 position, float angle); // wakes the body (Week 7)
    [[nodiscard]] float GetAngle(BodyHandle handle) const;
    [[nodiscard]] Vec2 GetLinearVelocity(BodyHandle handle) const;
    void SetLinearVelocity(BodyHandle handle, Vec2 velocity); // wakes the body
    [[nodiscard]] float GetAngularVelocity(BodyHandle handle) const;
    void SetAngularVelocity(BodyHandle handle, float angularVelocity);
    [[nodiscard]] float GetMass(BodyHandle handle) const;
    [[nodiscard]] float GetInvMass(BodyHandle handle) const;
    [[nodiscard]] BodyType GetBodyType(BodyHandle handle) const;
    [[nodiscard]] bool IsAwake(BodyHandle handle) const;
    [[nodiscard]] void* GetUserData(BodyHandle handle) const;
    void SetUserData(BodyHandle handle, void* userData);

    // Material properties (Week 2: consumed by the temporary response; Week 5:
    // combined per contact by friction = sqrt(fA*fB), restitution = max(eA,eB)).
    [[nodiscard]] float GetFriction(BodyHandle handle) const;
    void SetFriction(BodyHandle handle, float friction);
    [[nodiscard]] float GetRestitution(BodyHandle handle) const;
    void SetRestitution(BodyHandle handle, float restitution);

    // ------------------------------------------------------------------
    // External forces / impulses (applied before the next Step)
    // ------------------------------------------------------------------
    void ApplyForce(BodyHandle handle, Vec2 force, Vec2 worldPoint);
    void ApplyLinearImpulse(BodyHandle handle, Vec2 impulse, Vec2 worldPoint);

    // ------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------

    // Runs one fixed step using the World's SolverSettings iteration counts.
    // `dt` is also handed to the split-impulse position layer (its bias is a
    // velocity target, so it needs the same step the integrator used).
    void Step(float dt);

    // Same, with a per-call iteration override (testbed sliders, experiments).
    // Does not mutate the stored SolverSettings.
    void Step(float dt, int32_t velocityIterations, int32_t positionIterations);

    void SetGravity(Vec2 gravity) noexcept { m_gravity = gravity; }
    [[nodiscard]] Vec2 GetGravity() const noexcept { return m_gravity; }

    void SetSolverSettings(const SolverSettings& settings) noexcept { m_solverSettings = settings; }
    [[nodiscard]] const SolverSettings& GetSolverSettings() const noexcept { return m_solverSettings; }

    // ------------------------------------------------------------------
    // Pluggable broad phase (Week 6). Passing nullptr disables broad-phase work
    // and falls back to the O(N^2) enumeration; World owns the object. Switching
    // at runtime releases the old implementation's proxies and re-inserts the
    // bodies into the new one on the next step.
    // ------------------------------------------------------------------
    void SetBroadPhase(std::unique_ptr<IBroadPhase> broadPhase);
    [[nodiscard]] IBroadPhase* GetBroadPhase() const noexcept { return m_broadPhase.get(); }

    // Candidate pairs the installed broad phase produced for the current step
    // (0 when no broad phase is installed). Status panel / tests only.
    [[nodiscard]] int32_t GetBroadPhasePairCount() const noexcept {
        return static_cast<int32_t>(m_broadPhasePairs.size());
    }

    // ------------------------------------------------------------------
    // Contacts (Week 2): read-only view for tests, testbed panels and the
    // future solver. Rebuilt every Step(); do not hold references across steps.
    // ------------------------------------------------------------------
    [[nodiscard]] const std::vector<Contact>& GetContacts() const noexcept { return m_contacts; }

    // ------------------------------------------------------------------
    // Statistics panels / regression output (reserved values return 0 until the
    // owning week lands)
    // ------------------------------------------------------------------
    [[nodiscard]] int32_t GetBodyCount() const noexcept { return m_bodyIds.GetAliveCount(); }
    [[nodiscard]] int32_t GetShapeCount() const noexcept { return m_shapeIds.GetAliveCount(); }
    [[nodiscard]] int32_t GetAwakeBodyCount() const;
    [[nodiscard]] int32_t GetIslandCount() const noexcept;  // Week 7: islands built this step
    [[nodiscard]] int32_t GetContactCount() const noexcept; // Week 2: m_contacts.size()
    [[nodiscard]] int32_t GetSleepingBodyCount() const;     // Week 7: dynamic bodies asleep

    // ------------------------------------------------------------------
    // Week 7: sleeping and waking
    // ------------------------------------------------------------------

    // Wakes every body of the island `handle` belongs to. External forces,
    // impulses and direct transform/velocity changes let the user affect a
    // sleeping body; those calls must wake the WHOLE island (waking a single body
    // would leave its sleeping neighbours frozen while it moves away - the island
    // boundary clipping bug the whole-island sleep design exists to prevent).
    // ApplyForce/ApplyLinearImpulse/SetTransform/SetLinearVelocity/SetAngularVelocity
    // already do this automatically; this entry point is for tests/testbed.
    void WakeBody(BodyHandle handle);

    // ------------------------------------------------------------------
    // Debug
    // ------------------------------------------------------------------

    // Destroys all bodies/shapes and resets counters (used by demo Reset()).
    void Clear();

    // Scans every live body for NaN/Inf and asserts with the offending body
    // index/name. Compiles to nothing outside debug builds. This single check
    // is the most effective tool against solver explosions (Section 3.4).
    void CheckSane() const;

    // Read-only view of all body slots (dead slots included; validate handles
    // before use). Debug/testbed only - not part of the simulation contract.
    [[nodiscard]] const std::vector<Body>& GetBodies() const noexcept { return m_bodies; }

  private:
    // ---- Step phases (Section 4.6 order is normative) -------------------
    void IntegrateForces(float dt);
    void UpdateBroadPhase(float dt); // Week 6: proxy sync + canonical candidate pairs
    void UpdateContacts();           // Week 2/6: narrow phase over the candidate pairs
    void BuildIslands();             // Week 7: canonical island partition + islandId
    void
    PrepareConstraints(float dt);    // Week 5: rA/rB, K_n/K_t, restitution + Baumgarte bias, warm-start seeds
    void WarmStart();                // Week 5: apply the seeded accumulated impulses once
    void SolveVelocityConstraints(); // Week 5: normal + friction sequential impulses
    void IntegratePositions(float dt);
    void SolvePositionConstraints(float dt); // Week 5: split-impulse pseudo-velocity position layer
    void Synchronize();                      // Week 2/6: tight AABB + fat margin
    void UpdateSleep(float dt);              // Week 7: whole-island atomic sleeping
    void Validate() const;                   // debug-only NaN scan

    // Week 7: wakes the island that contains body slot `bodyIndex` (no-op for a
    // dead slot or a static/kinematic body, which are never island members).
    // Uses Body::islandId + m_islandBodyIndices from the last BuildIslands.
    void WakeIslandOfBody(int32_t bodyIndex);

    // Week 6: narrow-phase dispatch for ONE candidate body pair (indexA < indexB).
    // Shared by the broad-phase pair list and the O(N^2) fallback enumeration, so
    // the two paths cannot drift apart.
    void CreateContactForPair(int32_t indexA, int32_t indexB);

    // Week 6: removes every proxy from the installed broad phase and clears the
    // proxy <-> body bookkeeping (used by SetBroadPhase and Clear).
    void ReleaseBroadPhaseProxies();

    // Returns the body for a valid handle, asserting on misuse (debug).
    [[nodiscard]] Body* GetBodyMutable(BodyHandle handle);
    [[nodiscard]] const Body* GetBodyChecked(BodyHandle handle) const;
    [[nodiscard]] Shape* GetShapeMutable(ShapeHandle shape);

    Vec2 m_gravity;
    SolverSettings m_solverSettings;

    // Slot storage: index == BodyHandle::index. Slots are reused, never
    // compacted, so handles stay stable. m_bodyIds is the source of truth for
    // liveness/generation.
    std::vector<Body> m_bodies;
    IDPool m_bodyIds;

    // Shape pool (Week 2). One shape per body; geometry is owned here, the body
    // only stores the handle. Same slot/generation discipline as bodies.
    std::vector<Shape> m_shapes;
    IDPool m_shapeIds;

    // Contact list (Week 2). Rebuilt each frame; m_previousContacts keeps the
    // last frame for persistence matching (warm starting prep, Week 5).
    std::vector<Contact> m_contacts;
    std::vector<Contact> m_previousContacts;

    // Week 3 solver (forward-declared type; complete in Solver/ContactSolver.h).
    std::unique_ptr<ContactSolver> m_contactSolver;

    // Per-step iteration counts from Step(dt, velIters, posIters). The velocities
    // and positions phases read these so the public override stays meaningful.
    int32_t m_activeVelocityIterations = kVelocityIterations;
    int32_t m_activePositionIterations = kPositionIterations;

    // Week 6 broad phase. `m_broadPhase` may be null (O(N^2) fallback path).
    //  - m_broadPhasePairs: canonical candidate pairs for this step, as body slot
    //    indices with pair.first < pair.second, sorted lexicographically so the
    //    narrow-phase order reproduces the O(N^2) enumeration exactly (physics must
    //    not change when the broad phase changes);
    //  - m_proxyToBody: broad-phase proxy id -> body slot index (-1 = free). Body
    //    owns the forward mapping in Body::proxyId.
    std::unique_ptr<IBroadPhase> m_broadPhase;
    std::vector<std::pair<int32_t, int32_t>> m_broadPhasePairs;
    std::vector<int32_t> m_proxyToBody;

    // Week 7 island partition (canonical; contract in Solver/Island.h). Rebuilt
    // every step by BuildIslands and consumed by the solver (non-owning pointers
    // installed through SetIslandPartition), by UpdateSleep and by wake
    // propagation. Both arrays are empty when useIslandSolving is false.
    //  - m_islandBodyIndices[i]: dynamic body slot indices of island i, ascending;
    //  - m_islandContactIndices[i]: contact indices (ascending global order) of
    //    the contacts touching island i.
    // Plain std::vector members on purpose: World.h must not include a Solver
    // header (Core does not depend on Solver in its public surface).
    std::vector<std::vector<int32_t>> m_islandBodyIndices;
    std::vector<std::vector<int32_t>> m_islandContactIndices;
};

} // namespace p2
