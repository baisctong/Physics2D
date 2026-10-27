#pragma once

// p2::Contact — persisted contact pair between two bodies
// (Section 6.3 / P08, Week 2-5).
//
// ============================================================================
// RESERVED INTERFACE (Week 2 creates contacts, Week 5 makes them persistent)
// ============================================================================
// Week 1 only fixes the data layout so that World's reserved
// UpdateContacts() phase and the testbed's future contact panel can be written
// against a stable type. Nothing constructs a Contact yet.
//
// Lifetime model (Section 6.3):
//  - each frame the narrow phase regenerates the manifold;
//  - if the pair existed last frame, per-point impulses are inherited from the
//    old manifold by ContactID::Key() (MatchAndInheritImpulses, Week 5);
//  - if the manifold is empty the pair is destroyed;
//  - `isPersistent` tells warm starting whether the cached impulses are valid.
// ============================================================================

#include "physics2d/Collision/Manifold.h"
#include "physics2d/Core/Handle.h"

namespace p2 {

struct Contact {
    BodyHandle bodyA;
    BodyHandle bodyB;
    Manifold manifold;

    // Combined material properties, computed when the contact is created:
    //   friction    = sqrt(frictionA * frictionB)   (geometric mean, Week 5)
    //   restitution = max(restitutionA, restitutionB)
    float friction = 0.0f;
    float restitution = 0.0f;

    // True when this pair also existed in the previous frame. Only persistent
    // contacts may warm-start from their accumulated impulses.
    bool isPersistent = false;
};

} // namespace p2
