/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_EXAMPLES_DIFFUSION_H
#define BSPLINE_EXAMPLES_DIFFUSION_H

#include <bspline/Spline.h>

#include <vector>

#include "misc.h"

namespace bspline::examples::diffusion {

/**
 * Solves the steady state of the diffusion equation with the given (spatially
 * inhomogeneous) diffusion coefficient and the boundary values.
 * @param diffusionCoeff A spline representing the diffusion coefficient. Must
 * be continuous.
 * @param startValue The concentration at the begin of the domain.
 * @param endValue The concentration at the end of the domain.
 * @return Returns the (real) eigenenergies and the steady-state solution.
 */
Spline solveDiffusionSteadyState(PSpline diffusionCoeff, data_t startValue,
                                 data_t endValue);

}  // namespace bspline::examples::diffusion
#endif  // BSPLINE_EXAMPLES_DIFFUSION_H
