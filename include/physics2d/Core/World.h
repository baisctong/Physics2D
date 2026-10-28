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
//   2. broad phase update              6    UpdateBroadPhase()     [reserved]
//   3. narrow phase / contacts         2    UpdateContacts()       [done W2]
//   4. island construction             7    BuildIslands()         [reserved]
//   5. warm start                      5    WarmStart()            [reserved]
//   6. velocity iterations             3-5  SolveVelocityConstraints() [reserved]
//   7. integrate velocity -> position   1   IntegratePositions()   [done]
//   8. position iterations (split imp) 2/5  SolvePositionConstraints() [temporary W2]
//   9. synchronise AABB/transform      2    Synchronize()          [done W2]
//   10. sleep evaluation               7    UpdateSleep()          [reserved]
//   11. debug validation               1    Validate()             [done]
//
// Stable from Week 2 on:
// - Shape pool: one shape per body (Body::shape), created via Create*Shape and
//   attached with AttachShape (which derives mass/inertia from the density).
// - Contact list: World owns it (narrow-phase output), rebuilt every frame by
//   UpdateContacts() with O(N^2) pair enumeration + AABB filter. Week 6 replaces
//   only the enumeration with IBroadPhase; the Contact data contract stays.
// - Persistence: per-point accumulated impulses are inherited across frames by
//   ContactID::Key() in MatchAndInheritImpulses (Week 5 completes the solver
//   side; the matching logic already exists so warm starting can rely on it).
// - Temporary response: Week 2's SolvePositionConstraints() is a position-level
//   hack (push out + direct normal-velocity projection). Week 3 replaces the
//   velocity part with Sequential Impulse, Week 5 the position part with split
//   impulse. The two temporary pieces are marked in the .cpp file.
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
#include <vector>

namespace p2 {

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
    void Step(float dt);

    // Same, with a per-call iteration override (testbed sliders, experiments).
    // Does not mutate the stored SolverSettings.
    void Step(float dt, int32_t velocityIterations, int32_t positionIterations);

    void SetGravity(Vec2 gravity) noexcept { m_gravity = gravity; }
    [[nodiscard]] Vec2 GetGravity() const noexcept { return m_gravity; }

    void SetSolverSettings(const SolverSettings& settings) noexcept { m_solverSettings = settings; }
    [[nodiscard]] const SolverSettings& GetSolverSettings() const noexcept { return m_solverSettings; }

    // ------------------------------------------------------------------
    // RESERVED: pluggable broad phase (Week 6).
    // Passing nullptr disables broad-phase work; World owns the object.
    // ------------------------------------------------------------------
    void SetBroadPhase(std::unique_ptr<IBroadPhase> broadPhase);
    [[nodiscard]] IBroadPhase* GetBroadPhase() const noexcept { return m_broadPhase.get(); }

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
    [[nodiscard]] int32_t GetIslandCount() const noexcept;  // reserved (Week 7)
    [[nodiscard]] int32_t GetContactCount() const noexcept; // Week 2: m_contacts.size()

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
    void UpdateBroadPhase(float dt); // reserved Week 6
    void UpdateContacts();           // Week 2: O(N^2) + AABB filter + analytic narrow phase
    void BuildIslands();             // reserved Week 7
    void WarmStart();                // reserved Week 5
    void SolveVelocityConstraints(); // reserved Weeks 3-5
    void IntegratePositions(float dt);
    void SolvePositionConstraints(); // Week 2 temporary position-level response
    void Synchronize();              // Week 2: body AABBs from shapes
    void UpdateSleep(float dt);      // reserved Week 7
    void Validate() const;           // debug-only NaN scan

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

    // RESERVED (Week 6): broad phase implementation, may be null until then.
    std::unique_ptr<IBroadPhase> m_broadPhase;
};

} // namespace p2
