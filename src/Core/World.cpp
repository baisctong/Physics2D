// p2::World — body/shape ownership and the fixed simulation pipeline
// (Section 4.6). Week 5 completes the solver wiring: UpdateContacts (phase 3)
// produces the manifolds and inherits the per-point accumulated impulses,
// PrepareConstraints caches the normal/tangent effective masses plus the
// restitution/Baumgarte bias, WarmStart (phase 5) applies the inherited impulses,
// SolveVelocityConstraints (phase 6) runs normal + friction iterations, and
// SolvePositionConstraints (phase 8) runs the split-impulse pseudo-velocity
// layer. The Week 2 temporary position push-out is gone.
// See World.h for the frozen pipeline and API contracts.
//
// Scale note: pair enumeration is O(N^2) with an AABB pre-filter on purpose.
// Week 6 replaces only the enumeration with IBroadPhase; the contact data
// contract and the narrow-phase dispatch below stay exactly the same.

#include "physics2d/Core/World.h"

// World.h only forward-declares ContactSolver (Core must not include Solver
// headers in its public surface); the .cpp includes it because the unique_ptr
// destructor needs the complete type.
#include "physics2d/Solver/ContactSolver.h"

#include "physics2d/Collision/Analytic.h"
#include "physics2d/Collision/CircleShape.h"
#include "physics2d/Collision/ContactID.h"
#include "physics2d/Collision/PlaneShape.h"
#include "physics2d/Collision/SAT.h"
#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>
#include <vector>

namespace p2 {

namespace {

// Boundary validation for user input (Section 3.4: never silently repair).
// A rejected BodyDef makes CreateBody return an invalid handle and leaves both
// the body pool and the slot array untouched, so a rejected call is a no-op.
[[nodiscard]] bool IsValidBodyDef(const BodyDef& def) {
    if (!IsValid(def.position) || !std::isfinite(def.angle) || !IsValid(def.linearVelocity) ||
        !std::isfinite(def.angularVelocity)) {
        return false;
    }
    if (!std::isfinite(def.mass) || !std::isfinite(def.inertia) || !std::isfinite(def.density) ||
        !std::isfinite(def.friction) || !std::isfinite(def.restitution)) {
        return false;
    }
    if (def.friction < 0.0f || def.restitution < 0.0f || def.density < 0.0f) {
        return false;
    }
    // Mass properties only matter for dynamic bodies. Static/kinematic bodies
    // ignore them by design, so their values are not a rejection criterion.
    if (def.type == BodyType::Dynamic && (def.mass <= 0.0f || def.inertia <= 0.0f)) {
        return false;
    }
    return true;
}

// Allocates a shape slot and copies `shape` into it. Same slot-reuse discipline
// as the body pool: index == ShapeHandle::index, generation checked on use.
[[nodiscard]] ShapeHandle AllocateShapeSlot(std::vector<Shape>& shapes, IDPool& shapeIds,
                                            const Shape& shape) {
    const int32_t index = shapeIds.Allocate();
    if (index >= static_cast<int32_t>(shapes.size())) {
        shapes.emplace_back();
    }
    Shape& slot = shapes[static_cast<std::size_t>(index)];
    slot = shape;
    return ShapeHandle{index, shapeIds.GetGeneration(index)};
}

// Returns the previous frame's contact for the same body pair, or nullptr. Used
// both for SAT hysteresis (the cache hint) and as the persistence source.
[[nodiscard]] const Contact* FindPreviousContact(const std::vector<Contact>& previousContacts,
                                                 BodyHandle bodyA, BodyHandle bodyB) {
    for (const Contact& previous : previousContacts) {
        if (previous.bodyA == bodyA && previous.bodyB == bodyB) {
            return &previous;
        }
    }
    return nullptr;
}

// Inherits per-point accumulated impulses from the previous frame's contact for
// the same body pair (Section 6.3, P09 warm-start preparation). Matching is by
// ContactID::Key(): the physical feature must be the same point, otherwise the
// cached impulse would be applied to the wrong anchor. The pair is persistent
// only when EVERY new point found its predecessor.
[[nodiscard]] bool MatchAndInheritImpulses(Contact& contact, const std::vector<Contact>& previousContacts) {
    for (const Contact& previous : previousContacts) {
        if (previous.bodyA != contact.bodyA || previous.bodyB != contact.bodyB) {
            continue;
        }
        bool allPointsMatched = true;
        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            ContactPoint& point = contact.manifold.points[pointIndex];
            bool matched = false;
            for (int32_t oldPointIndex = 0; oldPointIndex < previous.manifold.pointCount; ++oldPointIndex) {
                const ContactPoint& oldPoint = previous.manifold.points[oldPointIndex];
                if (oldPoint.id.Key() == point.id.Key()) {
                    point.normalImpulse = oldPoint.normalImpulse;
                    point.tangentImpulse = oldPoint.tangentImpulse;
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                allPointsMatched = false;
            }
        }
        return allPointsMatched;
    }
    return false;
}

} // namespace

World::World(Vec2 gravity) : m_gravity(gravity), m_contactSolver(std::make_unique<ContactSolver>()) {
    assert(p2::IsValid(gravity) && "World: non-finite gravity");
}

World::~World() = default;

// ---------------------------------------------------------------------------
// Body lifecycle
// ---------------------------------------------------------------------------

BodyHandle World::CreateBody(const BodyDef& def) {
    if (!IsValidBodyDef(def)) {
        return BodyHandle{};
    }

    const int32_t index = m_bodyIds.Allocate();
    // Allocate() reuses a released slot when one is available (index < size) and
    // appends the next fresh index otherwise, so the slot array grows only in
    // the latter case. Slots are never compacted: handles stay stable.
    if (index >= static_cast<int32_t>(m_bodies.size())) {
        m_bodies.emplace_back();
    }

    Body& body = m_bodies[static_cast<std::size_t>(index)];
    body.handle = BodyHandle{index, m_bodyIds.GetGeneration(index)};
    body.Initialize(def);
    // A body without a collider is a point; this keeps the contact pair filter
    // well-defined until AttachShape replaces it with the real bounds.
    body.aabb = AABB{body.xf.p, body.xf.p};
    return body.handle;
}

void World::DestroyBody(BodyHandle handle) {
    // Stale/unknown handles are ignored on purpose: destruction is idempotent,
    // so cleanup paths may run twice. The generation check also means a stale
    // handle can never destroy the newer body that reused its slot.
    if (!IsValid(handle)) {
        return;
    }

    Body& body = m_bodies[static_cast<std::size_t>(handle.index)];
    // The body owns its shape slot: release it so the shape pool does not leak.
    if (IsValidShape(body.shape)) {
        m_shapeIds.Release(body.shape.index);
    }
    body.shape = ShapeHandle{};
    // Drop cached contacts referencing this body so readers never observe a
    // contact with a handle that has just gone stale. The list is rebuilt at the
    // next Step anyway.
    const auto referencesDestroyed = [&handle](const Contact& contact) {
        return contact.bodyA == handle || contact.bodyB == handle;
    };
    m_contacts.erase(std::remove_if(m_contacts.begin(), m_contacts.end(), referencesDestroyed),
                     m_contacts.end());
    m_previousContacts.erase(
        std::remove_if(m_previousContacts.begin(), m_previousContacts.end(), referencesDestroyed),
        m_previousContacts.end());

    body.handle = BodyHandle{}; // a dead slot no longer advertises a handle
    body.isAwake = false;
    m_bodyIds.Release(handle.index);
}

bool World::IsValid(BodyHandle handle) const {
    if (!handle.IsValid()) {
        return false;
    }
    const int32_t index = handle.index;
    // Range check first: after Clear() a stale handle may point past the arrays.
    // Liveness + generation together reject both destroyed slots and the ABA
    // case where the slot has been reused by a newer body.
    return m_bodyIds.IsValidIndex(index) && m_bodyIds.IsValid(index) &&
           m_bodyIds.GetGeneration(index) == handle.generation;
}

const Body* World::GetBody(BodyHandle handle) const {
    // Stale handles yield nullptr (documented in the header) instead of tripping
    // the internal assertion: UI panels legitimately poll a handle for one frame
    // after destroying the body.
    if (!IsValid(handle)) {
        return nullptr;
    }
    return GetBodyChecked(handle);
}

// ---------------------------------------------------------------------------
// Shapes (Week 2)
// ---------------------------------------------------------------------------

ShapeHandle World::CreateCircleShape(float radius, Vec2 localCenter) {
    Shape shape;
    shape.type = ShapeType::Circle;
    shape.circle.radius = radius;
    shape.circle.localCenter = localCenter;
    // Delegate validity to the shape itself so the acceptance criteria live in
    // one place (Collision module); World only turns the rejection into an
    // invalid handle + debug assert (core has no exception path).
    if (!shape.IsValid()) {
        assert(false &&
               "World::CreateCircleShape: radius must be finite and > 0, localCenter must be finite");
        return ShapeHandle{};
    }
    return AllocateShapeSlot(m_shapes, m_shapeIds, shape);
}

ShapeHandle World::CreatePlaneShape(Vec2 normal, float offset) {
    Shape shape;
    shape.type = ShapeType::Plane;
    shape.plane.normal = normal;
    shape.plane.offset = offset;
    if (!shape.IsValid()) {
        assert(false && "World::CreatePlaneShape: normal must be unit length and finite, offset finite");
        return ShapeHandle{};
    }
    return AllocateShapeSlot(m_shapes, m_shapeIds, shape);
}

ShapeHandle World::CreatePolygonShape(const Vec2* vertices, int32_t count) {
    // The caller owns the vertex array; a null pointer is a programmer error and
    // must be rejected before SetPolygon reads it (core has no exception path).
    if (vertices == nullptr) {
        assert(false && "World::CreatePolygonShape: vertices must not be null");
        return ShapeHandle{};
    }
    Shape shape;
    shape.type = ShapeType::Polygon;
    // SetPolygon validates winding, convexity, duplicates/collinear vertices and
    // finite coordinates (see PolygonShape.h); the shape is left invalid on
    // rejection, so a rejected call never consumes a pool slot.
    if (!SetPolygon(shape.polygon, vertices, count)) {
        assert(false &&
               "World::CreatePolygonShape: rejected polygon (see PolygonShape.h validation contract)");
        return ShapeHandle{};
    }
    return AllocateShapeSlot(m_shapes, m_shapeIds, shape);
}

ShapeHandle World::CreateBoxShape(float halfWidth, float halfHeight) {
    Shape shape;
    shape.type = ShapeType::Polygon;
    // Boxes are the canonical polygon; SetPolygonAsBox orders vertices CCW from
    // (+hx,-hy) so the same input always yields the same edge indices, which is
    // what keeps ContactID (and therefore warm starting in Week 5) stable.
    if (!SetPolygonAsBox(shape.polygon, halfWidth, halfHeight)) {
        assert(false && "World::CreateBoxShape: half extents must be finite and > 0");
        return ShapeHandle{};
    }
    return AllocateShapeSlot(m_shapes, m_shapeIds, shape);
}

bool World::AttachShape(BodyHandle bodyHandle, ShapeHandle shapeHandle) {
    if (!IsValid(bodyHandle) || !IsValidShape(shapeHandle)) {
        return false;
    }

    Body& body = *GetBodyMutable(bodyHandle);
    const Shape& shape = *GetShapeMutable(shapeHandle);

    // A plane is an infinite half-space with infinite mass: only a static body
    // can own one. Attaching it to a dynamic body would also make the derived
    // mass properties meaningless, so this is a hard misuse (assert + reject).
    if (shape.type == ShapeType::Plane && body.type != BodyType::Static) {
        assert(false && "World::AttachShape: a PlaneShape can only be attached to a Static body");
        return false;
    }

    // Derive the new mass properties BEFORE mutating anything: a rejected attach
    // must leave the previous shape and mass properties untouched (Section 3.4).
    float newMass = 0.0f;
    float newInertia = 0.0f;
    float newInvMass = 0.0f;
    float newInvInertia = 0.0f;
    if (body.type == BodyType::Dynamic) {
        if (!(std::isfinite(body.density) && body.density > 0.0f)) {
            assert(false && "World::AttachShape: attaching to a Dynamic body requires density > 0");
            return false;
        }
        const MassData massData = ComputeShapeMass(shape, body.density);
        if (!(std::isfinite(massData.mass) && massData.mass > 0.0f) || !std::isfinite(massData.inertia) ||
            massData.inertia < 0.0f || !p2::IsValid(massData.center)) {
            assert(false && "World::AttachShape: shape produced non-physical mass data");
            return false;
        }
        newMass = massData.mass;
        newInvMass = 1.0f / newMass;
        // Parallel-axis transfer from the shape centre to the body origin (the
        // rotation centre in Weeks 1-3): I_origin = I_centre + m * |centre|^2.
        // A body-local centre-of-mass offset is deferred to the Week 4 refactor;
        // this is exact for a centred circle (centre == 0) and approximate for an
        // off-centre one (documented Week 2 limitation).
        newInertia = massData.inertia + newMass * LengthSquared(massData.center);
        newInvInertia = body.fixedRotation ? 0.0f : (1.0f / newInertia);
    }
    // Static/kinematic bodies stay infinite-mass: the four values remain 0.

    // Commit: release the previously attached shape slot (one shape per body),
    // then store the new handle and mass properties.
    if (IsValidShape(body.shape) && body.shape != shapeHandle) {
        m_shapeIds.Release(body.shape.index);
    }
    body.shape = shapeHandle;
    body.mass = newMass;
    body.inertia = newInertia;
    body.invMass = newInvMass;
    body.invInertia = newInvInertia;
    // Update immediately: UpdateContacts' AABB filter runs before the first
    // Synchronize phase of the step that follows the attach.
    body.aabb = ComputeShapeAABB(shape, body.xf);
    return true;
}

void World::DestroyShape(ShapeHandle shapeHandle) {
    // Stale/unknown handles are ignored on purpose (destruction is idempotent),
    // matching DestroyBody's policy.
    if (!IsValidShape(shapeHandle)) {
        return;
    }
    // A shape referenced by a live body must not be freed: the body would keep a
    // dangling ShapeHandle. Destroy the body (which detaches) or replace the
    // shape instead - this is a caller bug, not something to silently repair.
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (m_bodyIds.IsValid(index) && m_bodies[static_cast<std::size_t>(index)].shape == shapeHandle) {
            assert(false && "World::DestroyShape: shape is still attached to a body");
            return;
        }
    }
    m_shapeIds.Release(shapeHandle.index);
}

bool World::IsValidShape(ShapeHandle shape) const {
    if (!shape.IsValid()) {
        return false;
    }
    const int32_t index = shape.index;
    return m_shapeIds.IsValidIndex(index) && m_shapeIds.IsValid(index) &&
           m_shapeIds.GetGeneration(index) == shape.generation;
}

const Shape* World::GetShape(ShapeHandle shape) const {
    if (!IsValidShape(shape)) {
        return nullptr;
    }
    return &m_shapes[static_cast<std::size_t>(shape.index)];
}

// ---------------------------------------------------------------------------
// Stable state accessors
//
// All read accessors tolerate stale handles and return neutral defaults; the
// world has no dangling references to hand out. Mutating accessors are no-ops
// for stale handles (defensive) and debug-assert the internal fetch, so a bug
// in the engine itself is still caught.
// ---------------------------------------------------------------------------

Vec2 World::GetPosition(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->GetPosition() : Vec2{0.0f, 0.0f};
}

Transform World::GetTransform(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->xf : Transform{};
}

void World::SetTransform(BodyHandle handle, Vec2 position, float angle) {
    assert(p2::IsValid(position) && std::isfinite(angle) && "World::SetTransform: non-finite transform");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->xf.p = position;
    body->xf.q.Set(angle);
    // Keep the cached AABB consistent immediately: UpdateContacts may run before
    // the next Synchronize phase, and a teleport must not use stale bounds.
    if (IsValidShape(body->shape)) {
        body->aabb = ComputeShapeAABB(*GetShapeMutable(body->shape), body->xf);
    } else {
        body->aabb = AABB{body->xf.p, body->xf.p};
    }
    // Week 7 sleep hook: teleporting a body invalidates its sleep history. A
    // static body is always asleep - it has no velocity to integrate anyway.
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetAngle(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->GetAngle() : 0.0f;
}

Vec2 World::GetLinearVelocity(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->linearVelocity : Vec2{0.0f, 0.0f};
}

void World::SetLinearVelocity(BodyHandle handle, Vec2 velocity) {
    assert(p2::IsValid(velocity) && "World::SetLinearVelocity: non-finite velocity");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->linearVelocity = velocity;
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetAngularVelocity(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->angularVelocity : 0.0f;
}

void World::SetAngularVelocity(BodyHandle handle, float angularVelocity) {
    assert(std::isfinite(angularVelocity) && "World::SetAngularVelocity: non-finite velocity");
    if (!IsValid(handle)) {
        return;
    }
    Body* body = GetBodyMutable(handle);
    body->angularVelocity = angularVelocity;
    if (body->type != BodyType::Static) {
        body->isAwake = true;
    }
}

float World::GetMass(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->mass : 0.0f;
}

float World::GetInvMass(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->invMass : 0.0f;
}

BodyType World::GetBodyType(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    // A dead body is inert; Static is the neutral "never moves" answer and keeps
    // generic UI loops free of special cases.
    return body != nullptr ? body->type : BodyType::Static;
}

bool World::IsAwake(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr && body->isAwake;
}

void* World::GetUserData(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->userData : nullptr;
}

void World::SetUserData(BodyHandle handle, void* userData) {
    if (!IsValid(handle)) {
        return;
    }
    GetBodyMutable(handle)->userData = userData;
}

// ---------------------------------------------------------------------------
// Material properties (Week 2)
// ---------------------------------------------------------------------------

float World::GetFriction(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->friction : 0.0f;
}

void World::SetFriction(BodyHandle handle, float friction) {
    assert(std::isfinite(friction) && friction >= 0.0f &&
           "World::SetFriction: friction must be finite and >= 0");
    if (!IsValid(handle) || !std::isfinite(friction) || friction < 0.0f) {
        return;
    }
    GetBodyMutable(handle)->friction = friction;
}

float World::GetRestitution(BodyHandle handle) const {
    const Body* body = GetBody(handle);
    return body != nullptr ? body->restitution : 0.0f;
}

void World::SetRestitution(BodyHandle handle, float restitution) {
    assert(std::isfinite(restitution) && restitution >= 0.0f &&
           "World::SetRestitution: restitution must be finite and >= 0");
    if (!IsValid(handle) || !std::isfinite(restitution) || restitution < 0.0f) {
        return;
    }
    GetBodyMutable(handle)->restitution = restitution;
}

// ---------------------------------------------------------------------------
// External forces / impulses
// ---------------------------------------------------------------------------

void World::ApplyForce(BodyHandle handle, Vec2 force, Vec2 worldPoint) {
    if (!IsValid(handle)) {
        return;
    }
    // Body::ApplyForce ignores non-dynamic bodies (infinite mass).
    GetBodyMutable(handle)->ApplyForce(force, worldPoint);
}

void World::ApplyLinearImpulse(BodyHandle handle, Vec2 impulse, Vec2 worldPoint) {
    if (!IsValid(handle)) {
        return;
    }
    GetBodyMutable(handle)->ApplyLinearImpulse(impulse, worldPoint);
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

void World::Step(float dt) {
    // Single source of truth for the iteration defaults: the stored settings.
    Step(dt, m_solverSettings.velocityIterations, m_solverSettings.positionIterations);
}

void World::Step(float dt, int32_t velocityIterations, int32_t positionIterations) {
    // A non-positive or non-finite dt has no physical meaning and would turn the
    // whole body array into NaN within one step. The caller owns the fixed-step
    // contract (Section 4.3), so this is a programmer error: assert in debug and
    // refuse to corrupt the state in release.
    assert(std::isfinite(dt) && dt > 0.0f && "World::Step: dt must be finite and > 0");
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return;
    }

    // Store the per-call iteration override. The velocity count is consumed by
    // the Sequential Impulse iterations (phase 6) and the position count by the
    // split-impulse pseudo-velocity iterations (phase 8). Step(dt) passes the
    // stored SolverSettings defaults through this same path.
    assert(velocityIterations >= 0 && positionIterations >= 0 &&
           "World::Step: iteration counts must be non-negative");
    m_activeVelocityIterations = velocityIterations;
    m_activePositionIterations = positionIterations;

    // Section 4.6 pipeline: the ORDER is normative. When filling the reserved
    // phases in later weeks, do not move a call - only replace its body.
    IntegrateForces(dt);          //  1. force/gravity -> velocity            [done]
    UpdateBroadPhase(dt);         //  2. broad phase update                   [reserved Week 6]
    UpdateContacts();             //  3. narrow phase / contact generation     [done W2, O(N^2)]
    BuildIslands();               //  4. island construction                  [reserved Week 7]
    PrepareConstraints(dt);       //     solver cache + warm-start seeds       [done W5]
    WarmStart();                  //  5. warm start                           [done W5]
    SolveVelocityConstraints();   //  6. sequential impulses (normal+friction)[done W5]
    IntegratePositions(dt);       //  7. velocity -> position (semi-implicit) [done]
    SolvePositionConstraints(dt); //  8. split-impulse position layer         [done W5]
    Synchronize();                //  9. transform/AABB synchronisation        [done W2]
    UpdateSleep(dt);              // 10. sleep evaluation                     [reserved Week 7]
    Validate();                   // 11. debug validation (NaN/Inf scan)      [done]
}

void World::SetBroadPhase(std::unique_ptr<IBroadPhase> broadPhase) {
    // World owns the object; nullptr explicitly disables broad-phase work. Week
    // 2 always uses the O(N^2) enumeration in UpdateContacts.
    m_broadPhase = std::move(broadPhase);
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

int32_t World::GetAwakeBodyCount() const {
    // Static bodies are born asleep, so in practice this counts dynamic and
    // kinematic bodies. It is an O(n) scan; only the debug UI calls it.
    int32_t awakeCount = 0;
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (m_bodyIds.IsValid(index) && m_bodies[static_cast<std::size_t>(index)].isAwake) {
            ++awakeCount;
        }
    }
    return awakeCount;
}

int32_t World::GetIslandCount() const noexcept {
    // RESERVED (Week 7): islands only exist once the island builder lands.
    return 0;
}

int32_t World::GetContactCount() const noexcept {
    return static_cast<int32_t>(m_contacts.size());
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

void World::Clear() {
    // Contacts reference bodies by handle, so they must go before the bodies
    // (and shapes before the shape pool) to leave no stale references behind.
    // The solver's cached constraints point into the contact list, so they go
    // first of all.
    //
    // Week 5 pseudo-velocities need no explicit ClearPseudoVelocities() here:
    // they live on the Body hot state that is destroyed with m_bodies right
    // below, so no pseudo state can survive a scene rebuild.
    if (m_contactSolver != nullptr) {
        m_contactSolver->Clear();
    }
    m_contacts.clear();
    m_previousContacts.clear();
    m_shapes.clear();
    m_shapeIds.Clear();
    m_bodies.clear();
    m_bodyIds.Clear();
    // RESERVED (Week 6): the broad phase will need its proxies cleared here once
    // IBroadPhase grows a reset entry point; until then it holds no state.
}

void World::CheckSane() const {
#if defined(PHYSICS2D_DEBUG)
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        const Body& body = m_bodies[static_cast<std::size_t>(index)];
        const bool transformOk =
            p2::IsValid(body.xf.p) && std::isfinite(body.xf.q.c) && std::isfinite(body.xf.q.s);
        const bool velocityOk = p2::IsValid(body.linearVelocity) && std::isfinite(body.angularVelocity);
        const bool accumulatorOk = p2::IsValid(body.force) && std::isfinite(body.torque);
        // Week 5: the pseudo-velocity state is solver scratch, but a NaN there
        // would be integrated into the transform next step, so it gets the same
        // scan as the real state.
        const bool pseudoOk =
            p2::IsValid(body.pseudoLinearVelocity) && std::isfinite(body.pseudoAngularVelocity);
        if (transformOk && velocityOk && accumulatorOk && pseudoOk) {
            continue;
        }
        // Report before asserting: the abort itself carries no context, and
        // "which body, which field" is the most useful fact when a solver
        // explodes. This is the debug-only safety net of Section 3.4.
        std::fprintf(stderr,
                     "World::CheckSane: body %d ('%s') has non-finite state: pos=(%g, %g) "
                     "vel=(%g, %g) w=%g force=(%g, %g) torque=%g pseudoVel=(%g, %g) pseudoW=%g\n",
                     index, body.debugName.c_str(), static_cast<double>(body.xf.p.x),
                     static_cast<double>(body.xf.p.y), static_cast<double>(body.linearVelocity.x),
                     static_cast<double>(body.linearVelocity.y), static_cast<double>(body.angularVelocity),
                     static_cast<double>(body.force.x), static_cast<double>(body.force.y),
                     static_cast<double>(body.torque), static_cast<double>(body.pseudoLinearVelocity.x),
                     static_cast<double>(body.pseudoLinearVelocity.y),
                     static_cast<double>(body.pseudoAngularVelocity));
        assert(false && "World::CheckSane: non-finite body state (NaN/Inf)");
    }

    // Contact data is narrow-phase output: a NaN here would propagate into the
    // solver / response on the next step, so validate it with the same scan.
    const int32_t contactCount = static_cast<int32_t>(m_contacts.size());
    for (int32_t contactIndex = 0; contactIndex < contactCount; ++contactIndex) {
        const Contact& contact = m_contacts[static_cast<std::size_t>(contactIndex)];
        const Manifold& manifold = contact.manifold;
        const bool contactOk = p2::IsValid(manifold.normal) && std::isfinite(manifold.penetration) &&
                               std::isfinite(contact.friction) && std::isfinite(contact.restitution) &&
                               manifold.pointCount >= 0 && manifold.pointCount <= kMaxManifoldPoints;
        if (contactOk) {
            continue;
        }
        std::fprintf(stderr,
                     "World::CheckSane: contact %d/%d has invalid manifold state (pointCount=%d "
                     "normal=(%g, %g) penetration=%g)\n",
                     contact.bodyA.index, contact.bodyB.index, manifold.pointCount,
                     static_cast<double>(manifold.normal.x), static_cast<double>(manifold.normal.y),
                     static_cast<double>(manifold.penetration));
        assert(false && "World::CheckSane: non-finite contact state (NaN/Inf)");
    }
#endif
}

// ---------------------------------------------------------------------------
// Step phases
// ---------------------------------------------------------------------------

void World::IntegrateForces(float dt) {
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        Body& body = m_bodies[static_cast<std::size_t>(index)];
        if (body.type != BodyType::Dynamic || !body.isAwake) {
            // Static/kinematic bodies have zero inverse mass, sleeping bodies
            // must keep their velocity: neither group may be accelerated.
            continue;
        }

        // Semi-implicit Euler velocity update (Section 4.2):
        //   v += dt * (g + F/m)
        //   w += dt * (tau / I)
        // Precomputed inverse mass turns the division into a multiplication.
        body.linearVelocity += (m_gravity + body.force * body.invMass) * dt;
        body.angularVelocity += body.torque * body.invInertia * dt;
    }

    // Forces are per-step quantities and are consumed exactly once. Clear every
    // live body (not only the awake ones) so a force applied to a sleeping body
    // cannot bank up and kick it when it wakes up in Week 7.
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (m_bodyIds.IsValid(index)) {
            m_bodies[static_cast<std::size_t>(index)].ClearForces();
        }
    }
}

void World::UpdateBroadPhase(float dt) {
    // RESERVED (Week 6): with a broad phase installed, update/insert the fat
    // AABB of every live body and enumerate candidate pairs into UpdateContacts.
    // Week 2 uses the O(N^2) enumeration below instead.
    (void)dt;
}

void World::UpdateContacts() {
    // Rebuild the contact list from scratch every step (Section 6.3). The old
    // list is kept for one frame: it is the persistence/warm-start source.
    m_previousContacts = std::move(m_contacts);
    m_contacts.clear();

    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t indexA = 0; indexA < bodyCount; ++indexA) {
        if (!m_bodyIds.IsValid(indexA)) {
            continue;
        }
        Body& bodyA = m_bodies[static_cast<std::size_t>(indexA)];
        if (!IsValidShape(bodyA.shape)) {
            continue; // no collider -> cannot generate contacts
        }

        // Fixed i < j iteration: every pair is visited once and contact order is
        // independent of insertion history (determinism, Section 7.4). A is
        // always the lower handle index, so the narrow-phase normal convention
        // maps directly onto Contact::bodyA/bodyB.
        for (int32_t indexB = indexA + 1; indexB < bodyCount; ++indexB) {
            if (!m_bodyIds.IsValid(indexB)) {
                continue;
            }
            Body& bodyB = m_bodies[static_cast<std::size_t>(indexB)];
            if (!IsValidShape(bodyB.shape)) {
                continue;
            }

            // Infinite-mass pairs never generate contacts: nothing could be
            // pushed apart, and the response is mass-weighted.
            if (!bodyA.IsDynamic() && !bodyB.IsDynamic()) {
                continue;
            }

            // Broad-phase stand-in: cheap AABB rejection (Week 6 replaces the
            // enumeration, not this filter). Touching counts as overlap.
            if (!Overlaps(bodyA.aabb, bodyB.aabb)) {
                continue;
            }

            const Shape& shapeA = m_shapes[static_cast<std::size_t>(bodyA.shape.index)];
            const Shape& shapeB = m_shapes[static_cast<std::size_t>(bodyB.shape.index)];

            // Week 4 narrow phase: analytic circle paths + polygon SAT/clipping.
            // Plane-plane can never touch (two half-spaces can always be pushed
            // apart trivially), so it is not a contact. The dispatch is by shape
            // type of the lower-index body first, so the A/B normal convention
            // stays explicit at every call site.
            Manifold manifold;
            SATCache satCache; // only meaningful for polygon-polygon contacts
            const bool polygonPair = shapeA.type == ShapeType::Polygon && shapeB.type == ShapeType::Polygon;
            if (shapeA.type == ShapeType::Circle) {
                if (shapeB.type == ShapeType::Circle) {
                    manifold = CollideCircles(shapeA.circle, bodyA.xf, shapeB.circle, bodyB.xf);
                } else if (shapeB.type == ShapeType::Plane) {
                    manifold = CollideCircleAndPlane(shapeA.circle, bodyA.xf, shapeB.plane, bodyB.xf);
                } else {
                    manifold = CollideCircleAndPolygon(shapeA.circle, bodyA.xf, shapeB.polygon, bodyB.xf);
                }
            } else if (shapeA.type == ShapeType::Plane) {
                if (shapeB.type == ShapeType::Circle) {
                    manifold = CollidePlaneAndCircle(shapeA.plane, bodyA.xf, shapeB.circle, bodyB.xf);
                } else if (shapeB.type == ShapeType::Plane) {
                    continue;
                } else {
                    manifold = CollidePlaneAndPolygon(shapeA.plane, bodyA.xf, shapeB.polygon, bodyB.xf);
                }
            } else { // A is a Polygon
                if (shapeB.type == ShapeType::Circle) {
                    manifold = CollidePolygonAndCircle(shapeA.polygon, bodyA.xf, shapeB.circle, bodyB.xf);
                } else if (shapeB.type == ShapeType::Plane) {
                    manifold = CollidePolygonAndPlane(shapeA.polygon, bodyA.xf, shapeB.plane, bodyB.xf);
                } else {
                    // SAT hysteresis (Week 4 D6): feed the previous frame's
                    // reference face when the feature is enabled. The cache only
                    // breaks near-ties; it never changes the physical result, so
                    // enabling/disabling it must stay equivalent.
                    SATCache* cacheHint = nullptr;
                    if (m_solverSettings.useSatCache) {
                        if (const Contact* previous =
                                FindPreviousContact(m_previousContacts, bodyA.handle, bodyB.handle)) {
                            satCache = previous->satCache;
                        }
                        cacheHint = &satCache;
                    }
                    manifold = CollidePolygons(shapeA.polygon, bodyA.xf, shapeB.polygon, bodyB.xf, cacheHint);
                }
            }
            if (!manifold.IsTouching()) {
                continue;
            }

            Contact contact;
            contact.bodyA = bodyA.handle;
            contact.bodyB = bodyB.handle;
            contact.manifold = manifold;
            // Material combination contract (Contact.h): geometric mean friction,
            // larger restitution wins (the bouncier surface dominates).
            contact.friction = std::sqrt(bodyA.friction * bodyB.friction);
            contact.restitution =
                (bodyA.restitution > bodyB.restitution) ? bodyA.restitution : bodyB.restitution;
            contact.isPersistent = MatchAndInheritImpulses(contact, m_previousContacts);
            if (polygonPair) {
                // Persist the reference-face choice for the next frame (the cache
                // is invalid when caching is disabled, which is exactly the
                // nullptr-hint path above).
                contact.satCache = satCache;
            }
            m_contacts.push_back(contact);
        }
    }
}

void World::BuildIslands() {
    // RESERVED (Week 7): DFS/union-find over touching dynamic bodies. Static
    // bodies act as island boundaries and are never island members.
}

void World::PrepareConstraints(float dt) {
    // Solver precompute step (no body state changes, so it is not one of the
    // Section 4.6 phases): lever arms rA/rB, effective normal/tangent masses,
    // restitution bias (and the Baumgarte bias when split impulse is disabled),
    // and the warm-start seeds read from ContactPoint. Positions are frozen
    // between this call and SolveVelocityConstraints, so the cached levers stay
    // valid for the whole velocity phase.
    m_contactSolver->Prepare(m_bodies, m_contacts, m_solverSettings, dt);
}

void World::WarmStart() {
    // Phase 5: apply the inherited accumulated impulses (normal + tangent) once,
    // before the velocity iterations begin, so the solver starts from last
    // frame's solution. m_previousContacts already inherited the per-point values
    // by ContactID::Key() inside UpdateContacts.
    m_contactSolver->WarmStart(m_bodies, m_contacts, m_solverSettings);
}

void World::SolveVelocityConstraints() {
    // Phase 6 (W3 normal impulses, W5 normal + friction): Sequential Impulse
    // iterations. The solver writes the accumulated normal/tangent impulses back
    // into each ContactPoint, which is the persistent storage Week 5 inherits for
    // warm starting on the next frame.
    m_contactSolver->SolveVelocity(m_bodies, m_contacts, m_activeVelocityIterations, m_solverSettings);
}

void World::IntegratePositions(float dt) {
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        Body& body = m_bodies[static_cast<std::size_t>(index)];
        if (body.type == BodyType::Static || !body.isAwake) {
            // Static bodies never move; kinematic and awake dynamic bodies do.
            continue;
        }

        // Semi-implicit (symplectic) Euler, Section 4.2: use the velocity that
        // IntegrateForces just produced, x_{n+1} = x_n + dt * v_{n+1}. Using the
        // old velocity would be explicit Euler, which gains energy every step.
        Vec2 translation = body.linearVelocity * dt;
        const float translationLength = Length(translation);
        if (translationLength > kMaxTranslation) {
            // Tunnelling defence (Section 6.6): a body may not translate further
            // than kMaxTranslation in one step; direction is preserved so a fast
            // body still flies along its velocity.
            translation = translation * (kMaxTranslation / translationLength);
        }
        body.xf.p += translation;

        // Integrate the angle through Rot composition instead of storing an
        // unwrapped angle: the incremental rotation dq = Rot(w * dt) is clamped
        // to kMaxRotation for the same tunnelling reason.
        const float deltaAngle = Clamp(body.angularVelocity * dt, -kMaxRotation, kMaxRotation);
        // 2D rotations commute, so composing the increment body-locally (q * dq)
        // and world-locally (dq * q) gives the same orientation. q * dq matches
        // Transform's "local -> world" convention.
        body.xf.q = Mul(body.xf.q, Rot(deltaAngle));
        // Long chains of Mul() accumulate rounding error; renormalise so the Rot
        // invariant c^2 + s^2 == 1 holds for the future broad phase and the
        // renderer (which reads the axes straight from the transform).
        body.xf.q.Normalize();
    }
}

void World::SolvePositionConstraints(float dt) {
    // ========================================================================
    // Phase 8 (Week 5): split-impulse position layer. The Week 2 TEMPORARY
    // position push-out is GONE - it teleported bodies (not momentum conserving,
    // no angular response) and could not coexist with the velocity solver.
    //
    // ContactSolver::SolvePosition solves pseudo-velocity constraints (a
    // fraction beta of the overlap excess above the slop band), integrates those
    // pseudo-velocities into the transforms once, and clears them. Real
    // velocities are never touched: the correction removes penetration without
    // injecting energy. When `useSplitImpulse` is false the solver does nothing
    // here; penetration is then handled by the Baumgarte bias in Prepare
    // (comparison baseline).
    // ========================================================================
    m_contactSolver->SolvePosition(m_bodies, m_contacts, dt, m_activePositionIterations, m_solverSettings);
}

void World::Synchronize() {
    // Refresh each body's cached AABB from its shape and current transform. This
    // is what UpdateContacts filters on during the NEXT step (phase 3 runs before
    // this phase, so the bounds are always from the end of the previous step).
    const int32_t bodyCount = static_cast<int32_t>(m_bodies.size());
    for (int32_t index = 0; index < bodyCount; ++index) {
        if (!m_bodyIds.IsValid(index)) {
            continue;
        }
        Body& body = m_bodies[static_cast<std::size_t>(index)];
        if (IsValidShape(body.shape)) {
            body.aabb = ComputeShapeAABB(*GetShapeMutable(body.shape), body.xf);
        } else {
            // A collider-less body is a point: the degenerate box keeps the
            // candidate filter well-defined (no false positives).
            body.aabb = AABB{body.xf.p, body.xf.p};
        }
    }
}

void World::UpdateSleep(float dt) {
    // RESERVED (Week 7): accumulate sleepTime for slow bodies and put whole
    // islands to sleep after kSleepTimeThreshold seconds.
    (void)dt;
}

void World::Validate() const {
    // Phase 11, debug only: CheckSane compiles to an empty function outside
    // PHYSICS2D_DEBUG builds, so this phase costs nothing in release.
    CheckSane();
}

// ---------------------------------------------------------------------------
// Internal accessors
// ---------------------------------------------------------------------------

Body* World::GetBodyMutable(BodyHandle handle) {
    if (!IsValid(handle)) {
        assert(false && "World::GetBodyMutable: stale or invalid body handle");
        return nullptr;
    }
    return &m_bodies[static_cast<std::size_t>(handle.index)];
}

const Body* World::GetBodyChecked(BodyHandle handle) const {
    if (!IsValid(handle)) {
        assert(false && "World::GetBodyChecked: stale or invalid body handle");
        return nullptr;
    }
    return &m_bodies[static_cast<std::size_t>(handle.index)];
}

Shape* World::GetShapeMutable(ShapeHandle shape) {
    if (!IsValidShape(shape)) {
        assert(false && "World::GetShapeMutable: stale or invalid shape handle");
        return nullptr;
    }
    return &m_shapes[static_cast<std::size_t>(shape.index)];
}

} // namespace p2
