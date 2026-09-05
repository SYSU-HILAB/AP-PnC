/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_EXAMPLES_SPLINEPOTENTIAL_H
#define BSPLINE_EXAMPLES_SPLINEPOTENTIAL_H

#include <bspline/Spline.h>

#include <functional>
#include <vector>

#include "misc.h"

namespace bspline::examples::spline_potential {

/**
 * Solves the 1D Schroedinger equation with a potential represented by a
 * spline.
 * @param[in] v The spline representing the potential v(x).
 * @return Returns the (real) eigenenergies and the corresponding
 * wavefunctions.
 */
std::vector<Eigenspace> solveSEWithSplinePotential(PSpline v);

/**
 * Interpolates the given function with a spline.
 * @param[in] gridPoints The grid points at which to interpolate the function.
 * @param[in] func The function to interpolate.
 */
PSpline interpolateFunction(std::vector<data_t> gridPoints,
                            const std::function<data_t(data_t)> &func);
}  // namespace bspline::examples::spline_potential
#endif  // BSPLINE_EXAMPLES_SPLINEPOTENTIAL_H
