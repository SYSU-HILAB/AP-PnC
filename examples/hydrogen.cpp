/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include "hydrogen.h"

#include <bspline/BSplineGenerator.h>

#include <algorithm>
#include <eigen3/Eigen/Eigenvalues>

namespace bspline::examples::hydrogen {

using namespace bspline;

// The radial Hamiltonian operator r^2 *(-d^2/dr^2 -2/r d/dr + L * (L + 1) /
// r^2 - 2/r). Includes the term r^2 from the functional determinant.
static const auto hamiltonOperator =
    -operators::X<2>{} * operators::Dx<2>{} -
    2 * operators::X<1>{} * operators::Dx<1>{} + L * (L + 1) -
    2 * operators::X<1>{};

/**
 * @brief setUpKnotsVector Sets up the knots vector on which the basis splines
 * are defined.
 * @return A vector of data_t representing the knots.
 */
static std::vector<data_t> setUpKnotsVector() {
  std::vector<data_t> ret;

  size_t numberOfZeros = 1;

  if (SPLINE_ORDER + 1 > L) {
    numberOfZeros = SPLINE_ORDER + 1 - L;
  }

  // Adding a knot multiple times alters the continuity properties of the
  // generated splines at the corresponding grid point (see literature on
  // BSplines). This guarantees that the radial wavefunctions have the correct
  // scaling in the vicinity of r=0 (at least for sufficiently low L).
  for (size_t i = 0; i < numberOfZeros; i++)
    ret.push_back(static_cast<data_t>(0));

  // First point of the logarithmic grid.
  const data_t rmin = static_cast<data_t>(1) / 100;

  // Last point of the logarithmic grid.
  const data_t rmax = static_cast<data_t>(2000);

  // Roughly the number of grid points on the logarithmic grid.
  const int numberOfGridPoints = 300;

  // logarithmic step
  const data_t step =
      pow(rmax / rmin, 1 / static_cast<data_t>(numberOfGridPoints));

  for (int i = 0; i <= numberOfGridPoints; i++) {
    ret.push_back(rmin * pow(step, i));
  }
  return ret;
}

/**
 * @brief setUpBasis Sets up the BSpline basis.
 * @return A vector of BSplines representing the basis.
 */
static std::vector<Spline> setUpBasis() {
  return generateBSplines<SPLINE_ORDER>(setUpKnotsVector());
}

std::vector<Eigenspace> solveRadialHydrogen() {
  static_assert(L >= 0, "L may not be below zero.");

  // Get the basis.
  const std::vector<Spline> basis = setUpBasis();

  static_assert(
      std::is_nothrow_move_constructible_v<decltype(hamiltonOperator)>,
      "Operator is not nothrow movable.");

  static_assert(std::is_nothrow_move_constructible_v<decltype(
                    integration::BilinearForm{std::move(hamiltonOperator)})>,
                "BilinearForm is not nothrow movable.");

  const DeMat hamiltonian =
      setUpSymmetricMatrix(integration::BilinearForm{hamiltonOperator}, basis);

  // Overlap matrix. Includes the term r^2 from the functional determinant.
  const DeMat overlapMatrix =
      setUpSymmetricMatrix(integration::BilinearForm{operators::X<2>{}}, basis);

  // Solve the generalized eigenvalue problem A.x = lambda B.x
  const Eigen::GeneralizedSelfAdjointEigenSolver<DeMat> ges{hamiltonian,
                                                            overlapMatrix};

  // Retrieve the eigenvalues and eigenvectors.
  const auto &eigenvalues = ges.eigenvalues();
  const auto &eigenvectors = ges.eigenvectors();

  std::vector<Eigenspace> ret;
  ret.reserve(10);
  // Return the eigenvalues and eigenfunctions corresponding to the ten lowest
  // eigenvalues.
  for (size_t i = 0; i < 10; i++) {
    const auto eigenvalue = eigenvalues(i);
    const auto eigenvector = toStdVector(eigenvectors.col(i));
    ret.push_back({eigenvalue, linearCombination(eigenvector, basis)});
  }
  return ret;
}

}  // namespace bspline::examples::hydrogen
