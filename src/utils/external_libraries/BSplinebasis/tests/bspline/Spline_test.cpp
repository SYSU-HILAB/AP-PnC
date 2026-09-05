/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/BSplineGenerator.h>
#include <bspline/Core.h>
#include <bspline/integration/analytical.h>
#include <bspline/integration/numerical.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <type_traits>

using bspline::BSplineGenerator;
using bspline::Spline;

using namespace bspline::support;
using namespace bspline;

static_assert(std::is_nothrow_move_constructible_v<Spline<double, 3>> &&
                  std::is_nothrow_move_assignable_v<Spline<double, 3>>,
              "Spline is not nothrow moveable.");

template <typename T>
Spline<T, 0> getOne(const Grid<T> &grid) {
  T onet = static_cast<T>(1);
  auto support = Support<T>::createWholeGrid(grid);
  std::vector<std::array<T, 1>> coeffs(support.size() - 1, {onet});
  return Spline<T, 0>(std::move(support), std::move(coeffs));
}

template <typename T, size_t order>
void testIntegration(T tol) {
  using namespace bspline::integration;
  using namespace bspline::operators;

  using Spline = bspline::Spline<T, order>;
  using Spline0 = bspline::Spline<T, 0>;
  const BSplineGenerator generator(std::vector<T>{
      -7.0l,  -6.85l, -6.55l, -6.3l, -6.0l, -5.75l, -5.53l, -5.2l,
      -4.75l, -4.5l,  -3.0l,  -2.5l, -1.5l, -1.0l,  0.0l,   0.5l,
      1.5l,   2.5l,   3.5l,   4.0l,  4.35l, 4.55l,  4.95l,  5.4l,
      5.7l,   6.1l,   6.35l,  6.5l,  6.85l, 7.0l});

  const std::vector<Spline> splines =
      generator.template generateBSplines<order>();
  const Spline0 one = getOne(generator.getGrid());

  const auto f1 = [](const T & /*x*/) { return static_cast<T>(1); };
  const auto fx = [](const T &x) { return x; };

  ScalarProduct sp;
  BilinearForm bfx{X<1>{}};
  BilinearForm bfx2{X<2>{}};
  BilinearForm bfx_dx{X<1>{} * Dx<1>{}};
  BilinearForm bfdx{Dx<1>{}};
  BilinearForm bfdx2{Dx<2>{}};
  BilinearForm bfdxdx{Dx<1>{}, Dx<1>{}};
  BilinearForm bfx2_dx2{X<2>{} * Dx<2>{}};
  LinearForm lf{};

  for (const auto &s1 : splines) {
    for (const auto &s2 : splines) {
      BOOST_CHECK_SMALL(overlap<T>(s1, s2) - sp.evaluate(s1, s2), tol);
      BOOST_CHECK_SMALL(bfx.evaluate(s1, s2) - integrate_x<T>(s1, s2), tol);
      BOOST_CHECK_SMALL(bfx2.evaluate(s1, s2) - integrate_x2<T>(s1, s2),
                        static_cast<T>(5) * tol);
      BOOST_CHECK_SMALL(bfdx.evaluate(s1, s2) - integrate_dx<T>(s1, s2), tol);
      BOOST_CHECK_SMALL(bfx_dx.evaluate(s1, s2) - integrate_x_dx<T>(s1, s2),
                        static_cast<T>(2) * tol);
      BOOST_CHECK_SMALL(bfdx2.evaluate(s1, s2) - integrate_dx2<T>(s1, s2), tol);
      // using \int dx a(x) d^2/dx^2 b(x) = -\int dx (d/dx a(x)) (d/dx b(x)).
      BOOST_CHECK_SMALL(bfdx2.evaluate(s1, s2) + bfdxdx.evaluate(s1, s2),
                        static_cast<T>(25) * tol);
      BOOST_CHECK_SMALL(bfx2_dx2.evaluate(s1, s2) - integrate_x2_dx2<T>(s1, s2),
                        static_cast<T>(150) * tol);
      BOOST_CHECK_SMALL(sp.evaluate(s1, s2) - integrate<2 * order>(f1, s1, s2),
                        static_cast<T>(10) * tol);
      BOOST_CHECK_SMALL(bfx.evaluate(s1, s2) - integrate<2 * order>(fx, s1, s2),
                        static_cast<T>(10) * tol);
    }
    BOOST_CHECK_SMALL(lf.evaluate(s1) - integrate<T>(s1), tol);
    BOOST_CHECK_SMALL(lf.evaluate(s1 * s1) - sp.evaluate(s1, s1), tol);
    BOOST_CHECK_SMALL(lf(s1) - integrate<T>(s1), tol);
    BOOST_CHECK_SMALL(lf(s1 * s1) - sp.evaluate(s1, s1), tol);
  }
}

template <typename T, size_t order>
T lc(T x, const std::vector<T> &coeffs,
     const std::vector<bspline::Spline<T, order>> &splines) {
  T ret = static_cast<T>(0);
  for (size_t i = 0; i < coeffs.size(); i++) {
    ret += coeffs.at(i) * splines.at(i)(x);
  }
  return ret;
}

template <typename T, size_t order>
void testArithmetic(T tol) {
  static_assert(order >= 2, "For this test, order must be at least 2");
  using Spline = bspline::Spline<T, order>;
  using Spline6 = bspline::Spline<T, 2 * order>;
  using Spline0 = bspline::Spline<T, 0>;

  BSplineGenerator<T> generator(std::vector<T>{
      -7.0l,  -6.85l, -6.55l, -6.3l, -6.0l, -5.75l, -5.53l, -5.2l,
      -4.75l, -4.5l,  -3.0l,  -2.5l, -1.5l, -1.0l,  0.0l,   0.5l,
      1.5l,   2.5l,   3.5l,   4.0l,  4.35l, 4.55l,  4.95l,  5.4l,
      5.7l,   6.1l,   6.35l,  6.5l,  6.85l, 7.0l});

  const std::vector<Spline> splines =
      generator.template generateBSplines<order>();
  const Spline0 one = getOne(generator.getGrid());

  const std::vector<T> lcCoeffs{1, 2, 3, 4, 3};
  const std::vector<Spline> lcSplines{
      splines[0], splines[1], splines[splines.size() / 2],
      splines[splines.size() - 2], splines[splines.size() - 1]};
  Spline slc = bspline::linearCombination(lcCoeffs.begin(), lcCoeffs.end(),
                                          lcSplines.begin(), lcSplines.end());

  for (T x = slc.front(); x <= slc.back(); x += 0.01L) {
    BOOST_CHECK_SMALL(slc(x) - lc(x, lcCoeffs, lcSplines),
                      static_cast<T>(10) * tol);
  }

  for (const auto &s : splines) {
    Spline s2 = s * static_cast<T>(2);
    Spline sm = -s;
    Spline6 sprod = s * s;
    Spline s22 = s;
    s22 *= static_cast<T>(2);
    Spline shalf = s / static_cast<T>(2);
    Spline shalf2 = s;
    shalf2 /= static_cast<T>(2);
    Spline s5half = s2 + shalf;
    Spline s5half2 = s2;
    s5half2 += shalf;
    Spline s3half = s2 - shalf;
    Spline s3half2 = s2;
    s3half2 -= shalf;
    Spline splusone = s + one;

    for (T x = s.front(); x <= s.back(); x += 0.01L) {
      BOOST_CHECK_SMALL(sm(x) + s(x), tol);
      BOOST_CHECK_SMALL(s2(x) - static_cast<T>(2) * s(x), tol);
      BOOST_CHECK_SMALL(s22(x) - static_cast<T>(2) * s(x),
                        tol);  // Tests *= operator
      BOOST_CHECK_SMALL(shalf(x) - s(x) / static_cast<T>(2),
                        tol);  // Tests / operator
      BOOST_CHECK_SMALL(shalf2(x) - s(x) / static_cast<T>(2),
                        tol);  // Tests /= operator
      BOOST_CHECK_SMALL(
          s5half(x) - static_cast<T>(5) * s(x) / static_cast<T>(2),
          tol);  // Tests + operator
      BOOST_CHECK_SMALL(
          s5half2(x) - static_cast<T>(5) * s(x) / static_cast<T>(2),
          tol);  // Tests += operator
      BOOST_CHECK_SMALL(
          s3half(x) - static_cast<T>(3) * s(x) / static_cast<T>(2),
          tol);  // Tests - operator
      BOOST_CHECK_SMALL(
          s3half2(x) - static_cast<T>(3) * s(x) / static_cast<T>(2),
          tol);  // Tests -= operator
      BOOST_CHECK_SMALL(splusone(x) - s(x) - static_cast<T>(1),
                        tol);  // Tests + operator
    }
  }
  BOOST_TEST(static_cast<T>(1) == one(one.front()));
  BOOST_TEST(static_cast<T>(1) == one(one.back()));
}

BOOST_AUTO_TEST_SUITE(SplineArithmeticTestSuite)
BOOST_AUTO_TEST_CASE(TestIntegration) {
  constexpr double TOL = 1.0e-15;
  testIntegration<double, 2>(TOL);
  testIntegration<double, 3>(TOL);
  testIntegration<double, 4>(TOL);
  testIntegration<double, 5>(TOL);
  testIntegration<double, 6>(TOL);
  testIntegration<double, 7>(TOL);
  testIntegration<double, 8>(TOL);
  testIntegration<double, 9>(TOL);
  testIntegration<double, 10>(TOL);

  if constexpr (sizeof(long double) != sizeof(double)) {
    constexpr long double TOLL = 1.0e-18l;
    testIntegration<long double, 2>(TOLL);
    testIntegration<long double, 3>(TOLL);
    testIntegration<long double, 4>(TOLL);
    testIntegration<long double, 5>(TOLL);
    testIntegration<long double, 6>(TOLL);
    testIntegration<long double, 7>(TOLL);
    testIntegration<long double, 8>(TOLL);
    testIntegration<long double, 9>(TOLL);
    testIntegration<long double, 10>(TOLL);
  }
}

BOOST_AUTO_TEST_CASE(TestArithmetic) {
  constexpr double TOL = 1.0e-15;
  testArithmetic<double, 2>(TOL);
  testArithmetic<double, 3>(TOL);
  testArithmetic<double, 4>(TOL);
  testArithmetic<double, 5>(TOL);
  testArithmetic<double, 6>(TOL);
  testArithmetic<double, 7>(TOL);
  testArithmetic<double, 8>(TOL);
  testArithmetic<double, 9>(TOL);
  testArithmetic<double, 10>(TOL);

  if constexpr (sizeof(long double) != sizeof(double)) {
    constexpr long double TOLL = 1.0e-18l;
    testArithmetic<long double, 2>(TOLL);
    testArithmetic<long double, 3>(TOLL);
    testArithmetic<long double, 4>(TOLL);
    testArithmetic<long double, 5>(TOLL);
    testArithmetic<long double, 6>(TOLL);
    testArithmetic<long double, 7>(TOLL);
    testArithmetic<long double, 8>(TOLL);
    testArithmetic<long double, 9>(TOLL);
    testArithmetic<long double, 10>(TOLL);
  }
}

BOOST_AUTO_TEST_SUITE_END()
