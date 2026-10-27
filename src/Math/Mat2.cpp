// p2::Mat2 — out-of-line inverse for the column-major 2x2 matrix
// (Section 4.2 / P03 of the design document).
//
// Everything else in Mat2.h is a handful of multiply-adds and stays inline; the
// inverse is the only operation that needs a singularity guard, so it lives in
// a translation unit where that policy is visible.

#include "physics2d/Math/Mat2.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace p2 {

bool Inverse(const Mat2& m, Mat2& out) noexcept {
    const float det = Determinant(m);

    // Scale-relative singularity test. A fixed absolute threshold would reject
    // perfectly invertible matrices that happen to be uniformly small (a
    // 1e-4-scaled rotation has det == 1e-8), while a relative test only rejects
    // matrices whose inverse is meaningless at float precision. `max` of the
    // components (rather than their sum) keeps the estimate cheap and still
    // catches the degenerate zero matrix: there det == 0 and the threshold is 0,
    // so the strict `>` comparison reports failure.
    const float scale =
        std::max({std::fabs(m.cx.x), std::fabs(m.cx.y), std::fabs(m.cy.x), std::fabs(m.cy.y)});
    const float threshold = kEpsilon * scale * scale;
    if (!(std::fabs(det) > threshold)) {
        // Contract: `out` is left untouched on failure so that a caller can keep
        // reusing a buffer without clearing it and cannot accidentally consume a
        // half-written inverse.
        return false;
    }

    const float invDet = 1.0f / det;
    // With M = [[cx.x, cy.x], [cx.y, cy.y]] the closed-form inverse is
    //   (1/det) * [[ cy.y, -cy.x],
    //              [-cx.y,  cx.x]]
    // and column-major storage means the first column is (cy.y, -cx.y)/det and
    // the second column is (-cy.x, cx.x)/det.
    out.cx = {m.cy.y * invDet, -m.cx.y * invDet};
    out.cy = {-m.cy.x * invDet, m.cx.x * invDet};
    return true;
}

Mat2 GetInverse(const Mat2& m) noexcept {
    Mat2 inverse;
    if (!Inverse(m, inverse)) {
        // The convenience form promises invertibility (rotation matrices and
        // symmetric positive-definite mass matrices only). A singular input is a
        // programming error, so it is loud in debug builds and returns identity
        // in release (NDEBUG) rather than propagating NaNs into the simulation.
        assert(false && "GetInverse() called with a singular matrix");
        return Mat2::Identity();
    }
    return inverse;
}

} // namespace p2
