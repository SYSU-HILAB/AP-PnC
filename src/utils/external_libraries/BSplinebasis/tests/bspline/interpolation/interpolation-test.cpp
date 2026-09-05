/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/interpolation/interpolation.h>
#include <bspline/support/Support.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(SplineInterpolationTestSuite)
#ifdef BSPLINE_INTERPOLATION_USE_EIGEN
template <typename T, size_t order>
void testInterpolationEigen(T tol) {
  using Spline = bspline::Spline<T, order>;
  using Support = bspline::support::Support<T>;
  using Grid = bspline::support::Grid<T>;

  const Grid grid(std::vector<T>{-3.0l, -2.5l, -1.5l, -1.0l, 0.0l, 0.5l, 1.5l,
                                 2.5l, 3.5l, 4.0l, 5.0l});
  const auto x = Support::createWholeGrid(grid);
  const std::vector<T> y{-3.0l, -2.5l, -1.5l, -1.0l, 0.0l, -0.5l,
                         -1.5l, -2.5l, -3.5l, -4.0l, 3.0l};
  Spline s = bspline::interpolation::interpolateUsingEigen<T, order>(x, y);
  for (size_t i = 0; i < x.size(); i++) {
    BOOST_CHECK_SMALL(s(x[i]) - y[i], tol);
  }
}

BOOST_AUTO_TEST_CASE(TestInterpolationEigen) {
  testInterpolationEigen<double, 1>(2.0e-14);
  testInterpolationEigen<double, 2>(2.0e-14);
  testInterpolationEigen<double, 3>(2.0e-14);
  testInterpolationEigen<double, 4>(2.0e-14);

  if constexpr (sizeof(long double) != sizeof(double)) {
    testInterpolationEigen<long double, 1>(1.0e-17l);
    testInterpolationEigen<long double, 2>(1.0e-17l);
    testInterpolationEigen<long double, 3>(1.0e-17l);
    testInterpolationEigen<long double, 4>(1.0e-17l);
  }
}
#endif

#ifdef BSPLINE_INTERPOLATION_USE_ARMADILLO
template <size_t order>
void testInterpolationArmadillo(double tol) {
  using Spline = bspline::Spline<double, order>;
  using Support = bspline::support::Support<double>;
  using Grid = bspline::support::Grid<double>;

  const Grid grid(std::vector<double>{-3.0l, -2.5l, -1.5l, -1.0l, 0.0l, 0.5l,
                                      1.5l, 2.5l, 3.5l, 4.0l, 5.0l});
  const auto x = Support::createWholeGrid(grid);
  const std::vector<double> y{-3.0l, -2.5l, -1.5l, -1.0l, 0.0l, -0.5l,
                              -1.5l, -2.5l, -3.5l, -4.0l, 3.0l};
  Spline s = bspline::interpolation::interpolateUsingArmadillo<order>(x, y);
  for (size_t i = 0; i < x.size(); i++) {
    BOOST_CHECK_SMALL(s(x[i]) - y[i], tol);
  }
}

BOOST_AUTO_TEST_CASE(TestInterpolationArmadillo) {
  testInterpolationArmadillo<1>(2.0e-14);
  testInterpolationArmadillo<2>(2.0e-14);
  testInterpolationArmadillo<3>(2.0e-14);
  testInterpolationArmadillo<4>(2.0e-14);
}
#endif
BOOST_AUTO_TEST_SUITE_END()
