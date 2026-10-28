#pragma once

// p2::Contact — persisted contact pair between two bodies (Week 2 D5, P06/P08).
//
// Moved from Solver/ to Collision/ in Week 2: the World owns the contact list
// (it is narrow-phase output, not solver state), and Core must not depend on
// Solver. The solver (Week 3+) consumes the same struct, so the data contract
// is unchanged - only the include path moved.
//
// Lifetime model (Section 6.3):
//  - each frame the narrow phase regenerates the manifold from scratch;
//  - if the pair existed last frame, per-point impulses are inherited by
//    ContactID::Key() (MatchAndInheritImpulses, Week 5);
//  - if the manifold is empty the pair disappears from the contact list;
//  - `isPersistent` tells warm starting whether the cached impulses are valid.
//
// Week 2 uses this struct for bookkeeping (count, persistence flag, debug
// visualisation); the accumulated impulses are already carried per contact
// point but stay zero until the Sequential Impulse solver lands in Week 3/5.

#include "physics2d/Collision/Manifold.h"
#include "physics2d/Collision/SATCache.h"
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
    // contacts may warm-start from their accumulated impulses (Week 5).
    bool isPersistent = false;

    // Week 4 D6: SAT hysteresis carried across frames for this pair (see
    // SATCache.h). Only meaningful for polygon-polygon contacts.
    SATCache satCache;
};

} // namespace p2
