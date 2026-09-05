/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_EXAMPLES_HARMONICOSCILLATOR_H
#define BSPLINE_EXAMPLES_HARMONICOSCILLATOR_H

#include <bspline/Spline.h>

#include <vector>

#include "misc.h"

namespace bspline::examples::harmonic_oscillator {
/**
 * Solves the quantum mechanical harmonic oscillator problem with a BSpline
 * basis.
 * @return Returns the (real) eigenenergies and the corresponding wavefunctions.
 */
std::vector<Eigenspace> solveHarmonicOscillator();

}  // namespace bspline::examples::harmonic_oscillator
#endif  // BSPLINE_EXAMPLES_HARMONICOSCILLATOR_H
