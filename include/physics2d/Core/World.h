#pragma once

// p2::World — owns bodies and runs the simulation pipeline (Section 4.6).
//
// ============================================================================
// RESERVED WORK INTERFACE (this is the contract Weeks 2-8 build on)
// ============================================================================
// The Step() sequence below is fixed by Section 4.6 of the design document and
// is implemented NOW in its final shape, with the not-yet-landed phases present
// as empty private methods. Later weeks only fill in bodies of those methods:
//
//   Phase                            Week   Method
//   ------------------------------   ----   -------------------------------
//   1. integrate forces -> velocity    1    IntegrateForces()      [done]
//   2. broad phase update              6    UpdateBroadPhase()     [reserved]
//   3. narrow phase / contacts         2-5  UpdateContacts()       [reserved]
//   4. island construction             7    BuildIslands()         [reserved]
//   5. warm start                      5    WarmStart()            [reserved]
//   6. velocity iterations             3-5  SolveVelocityConstraints() [reserved]
//   7. integrate velocity -> position   1   IntegratePositions()   [done]
//   8. position iterations (split imp) 5    SolvePositionConstraints() [reserved]
//   9. synchronise AABB/transform      6    Synchronize()          [reserved]
//   10. sleep evaluation               7    UpdateSleep()          [reserved]
//   11. debug validation               1    Validate()             [done]
//
// Why define the empty phases now: inserting a phase in the middle of Step()
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

#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/BroadPhaseInterface.h"
#include "physics2d/Core/IDPool.h"
#include "physics2d/Core/Settings.h"

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
    // Statistics panels / regression output (reserved values return 0 in Week 1)
    // ------------------------------------------------------------------
    [[nodiscard]] int32_t GetBodyCount() const noexcept { return m_bodyIds.GetAliveCount(); }
    [[nodiscard]] int32_t GetAwakeBodyCount() const;
    [[nodiscard]] int32_t GetIslandCount() const noexcept;  // reserved (Week 7)
    [[nodiscard]] int32_t GetContactCount() const noexcept; // reserved (Week 2+)

    // ------------------------------------------------------------------
    // Debug
    // ------------------------------------------------------------------

    // Destroys all bodies and resets counters (used by demo Reset()).
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
    void UpdateContacts();           // reserved Weeks 2-5
    void BuildIslands();             // reserved Week 7
    void WarmStart();                // reserved Week 5
    void SolveVelocityConstraints(); // reserved Weeks 3-5
    void IntegratePositions(float dt);
    void SolvePositionConstraints(); // reserved Week 5
    void Synchronize();              // reserved Week 6
    void UpdateSleep(float dt);      // reserved Week 7
    void Validate() const;           // debug-only NaN scan

    // Returns the body for a valid handle, asserting on misuse (debug).
    [[nodiscard]] Body* GetBodyMutable(BodyHandle handle);
    [[nodiscard]] const Body* GetBodyChecked(BodyHandle handle) const;

    Vec2 m_gravity;
    SolverSettings m_solverSettings;

    // Slot storage: index == BodyHandle::index. Slots are reused, never
    // compacted, so handles stay stable. m_bodyIds is the source of truth for
    // liveness/generation.
    std::vector<Body> m_bodies;
    IDPool m_bodyIds;

    // RESERVED (Week 6): broad phase implementation, may be null in Week 1.
    std::unique_ptr<IBroadPhase> m_broadPhase;
};

} // namespace p2
