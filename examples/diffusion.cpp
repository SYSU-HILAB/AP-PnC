/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include "diffusion.h"

#include <bspline/BSplineGenerator.h>

namespace bspline::examples::diffusion {

using namespace bspline;
using namespace bspline::operators;

/**
 * @brief setUpKnotsVector Sets up the knots vector on which the basis splines
 * are defined.
 * @return A vector of data_t representing the knots.
 */
static std::vector<data_t> setUpKnotsVector(
    const support::Support<data_t> &support) {
  std::vector<data_t> ret;

  // Adding a knot multiple times alters the continuity properties of the
  // generated splines at the corresponding grid point (see literature on
  // BSplines). This guarantees that the radial wavefunctions have the correct
  // scaling in the vicinity of r=0 (at least for sufficiently low L).
  for (size_t i = 0; i < SPLINE_ORDER; i++) ret.push_back(support.front());

  for (const auto &val : support) ret.push_back(val);

  for (size_t i = 0; i < SPLINE_ORDER; i++) ret.push_back(support.back());

  return ret;
}

/**
 * @brief setUpBasis Sets up the BSpline basis.
 * @return A vector of BSplines representing the basis.
 */
static std::vector<Spline> setUpBasis(const support::Support<data_t> &support) {
  const BSplineGenerator generator(setUpKnotsVector(support),
                                   support.getGrid());
  return generator.template generateBSplines<SPLINE_ORDER>();
}

Spline solveDiffusionSteadyState(PSpline diffusionCoeff, data_t startValue,
                                 data_t endValue) {
  // Get the basis.
  std::vector<Spline> basis = setUpBasis(diffusionCoeff.getSupport());
  auto first = std::move(basis.front());
  first *= startValue;
  auto last = std::move(basis.back());
  last *= endValue;
  basis.erase(basis.begin());
  basis.erase(basis.end());

  const integration::BilinearForm bilinearForm{
      (static_cast<data_t>(1) / 2) *
      (Dx<1>{} * SplineOperator{std::move(diffusionCoeff)} * Dx<1>{})};

  DeVec b = DeVec::Zero(basis.size());
  const integration::ScalarProduct sp{};
  for (size_t i = 0; i < basis.size(); i++) {
    b(i) = -(bilinearForm.evaluate(basis.at(i), first) +
             bilinearForm.evaluate(basis.at(i), last));
  }

  const DeMat mat = setUpSymmetricMatrix(bilinearForm, basis);

  const auto coeffs = toStdVector(mat.colPivHouseholderQr().solve(b));

  return linearCombination(coeffs, basis) + first + last;
}

}  // namespace bspline::examples::diffusion
