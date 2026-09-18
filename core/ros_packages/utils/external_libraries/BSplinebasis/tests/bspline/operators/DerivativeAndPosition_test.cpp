/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/BSplineGenerator.h>
#include <bspline/integration/BilinearForm.h>
#include <bspline/operators/Derivative.h>
#include <bspline/operators/Position.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <type_traits>

using namespace bspline;
using namespace bspline::operators;
using namespace bspline::support;

static_assert(std::is_nothrow_move_constructible_v<Derivative<2>> &&
                  std::is_nothrow_move_assignable_v<Derivative<2>>,
              "Derivative is not nothrow moveable.");

static_assert(std::is_nothrow_move_constructible_v<Position<2>> &&
                  std::is_nothrow_move_assignable_v<Position<2>>,
              "Position is not nothrow moveable.");

template <typename T, size_t order>
static T diffNorm(const Spline<T, order> &s1, const Spline<T, order> &s2) {
  const auto diff = s1 - s2;
  const integration::ScalarProduct sp;
  return std::sqrt(sp.evaluate(diff, diff));
}

template <typename T>
static Spline<T, 0> getOne(const Grid<T> &grid) {
  const T onet = static_cast<T>(1);
  auto support = Support<T>::createWholeGrid(grid);
  std::vector<std::array<T, 1>> coeffs(support.size() - 1, {onet});
  return Spline<T, 0>(std::move(support), std::move(coeffs));
}

template <typename T>
static Spline<T, 0> getOne() {
  const Grid<T> grid{std::vector<T>{-3.0l, -2.0l, -1.5l, -0.878l, -0.238l,
                                    0.4012l, 1.323l, 1.9238l, 2.057l, 2.4812l,
                                    3.182379l}};
  return getOne(grid);
}

BOOST_AUTO_TEST_SUITE(DerivativeAndPositionOperatorTestSuite)
/**
 * Passes if the derivative operator correctly reverts the effect of the
 * position operator.
 */
BOOST_AUTO_TEST_CASE(DerivativeAndPositionOperatorTest) {
  constexpr double tol = 5.0e-16;
  const auto one = getOne<double>();
  const auto zero = 0.0 * one;
  const auto x = X<1>{} * one;
  const auto halfXSquared = (0.5 * X<2>{}) * one;
  const auto oneSixthXCubed = ((1.0 / 6.0) * X<3>{}) * one;

  BOOST_CHECK_SMALL(diffNorm(Dx<1>{} * zero, zero), tol);

  BOOST_CHECK_SMALL(diffNorm(Dx<1>{} * one, zero), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<2>{} * one, zero), tol);

  BOOST_CHECK_SMALL(diffNorm(Dx<1>{} * x, one), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<2>{} * x, zero), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<3>{} * x, zero), tol);

  BOOST_CHECK_SMALL(diffNorm(Dx<1>{} * halfXSquared, x), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<2>{} * halfXSquared, one), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<3>{} * halfXSquared, zero), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<4>{} * halfXSquared, zero), tol);

  BOOST_CHECK_SMALL(diffNorm(Dx<1>{} * oneSixthXCubed, halfXSquared), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<2>{} * oneSixthXCubed, x), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<3>{} * oneSixthXCubed, one), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<4>{} * oneSixthXCubed, zero), tol);
  BOOST_CHECK_SMALL(diffNorm(Dx<5>{} * oneSixthXCubed, zero), tol);
}

/**
 * Passes if the splines derived from the application of the position operator
 * to the identity spline yield the correct function values upon evaluation.
 */
BOOST_AUTO_TEST_CASE(PositionOperatorTest) {
  constexpr double tol = 1.0e-14;
  constexpr size_t numSteps = 1000;
  const auto one = getOne<double>();
  const auto zero = 0.0 * one;
  const auto x = X<1>{} * one;
  const auto xSquared = X<2>{} * one;
  const auto xCubed = X<3>{} * one;

  const auto start = one.getSupport().front();
  const auto end = one.getSupport().back();
  const auto step = (end - start) / numSteps;

  for (size_t i = 0; i <= numSteps; i++) {
    // Make sure param <= end (it might else be larger due to rounding errors).
    const auto param = std::min(start + i * step, end);
    BOOST_CHECK_SMALL(zero(param) - 0.0, tol);
    BOOST_CHECK_SMALL(one(param) - 1.0, tol);
    BOOST_CHECK_SMALL(x(param) - param, tol);
    BOOST_CHECK_SMALL(xSquared(param) - std::pow(param, 2), tol);
    BOOST_CHECK_SMALL(xCubed(param) - std::pow(param, 3), tol);
  }
}
BOOST_AUTO_TEST_SUITE_END()
