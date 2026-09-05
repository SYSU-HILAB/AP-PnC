/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_EXAMPLES_HYDROGEN_H
#define BSPLINE_EXAMPLES_HYDROGEN_H

#include <bspline/Spline.h>

#include <vector>

#include "misc.h"

namespace bspline::examples::hydrogen {

/**
 * The orbital quantum number.
 */
static constexpr int L = 1;

/**
 * Solves the radial hydrogen problem with a BSpline
 * basis.
 * @return Returns the (real) eigenenergies and the corresponding wavefunctions.
 */
std::vector<Eigenspace> solveRadialHydrogen();

}  // namespace bspline::examples::hydrogen
#endif  // BSPLINE_EXAMPLES_HYDROGEN_H
