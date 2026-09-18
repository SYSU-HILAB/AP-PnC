/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/BSplineGenerator.h>
#include <bspline/integration/BilinearForm.h>
#include <bspline/operators/Derivative.h>
#include <bspline/operators/ScalarOperators.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <type_traits>

using namespace bspline;
using namespace bspline::operators;

static_assert(std::is_nothrow_move_constructible_v<
                  ScalarMultiplication<double, IdentityOperator>> &&
                  std::is_nothrow_move_assignable_v<
                      ScalarMultiplication<double, IdentityOperator>>,
              "ScalarMultiplication is not nothrow moveable.");

static_assert(std::is_same_v<decltype(3.0 * IdentityOperator{}),
                             ScalarMultiplication<double, IdentityOperator>>,
              "Unexpected type.");

template <typename T, size_t order>
static T diffNorm(const Spline<T, order> &s1, const Spline<T, order> &s2) {
  const auto diff = s1 - s2;
  const integration::ScalarProduct sp;
  return std::sqrt(sp.evaluate(diff, diff));
}

template <typename T, size_t order>
static void testSplineMultiplication(T tol) {
  const T multiplicator = static_cast<T>(313) / 17;
  const auto op1 = multiplicator * IdentityOperator{};
  const auto op2 = multiplicator * Dx<1>{};
  std::vector<T> knots;
  for (size_t i = 0; i <= order + 4; i++) {
    knots.push_back(static_cast<T>(i));
  }
  const BSplineGenerator generator{knots};

  const auto splines = generator.template generateBSplines<order>();

  for (const auto &spline : splines) {
    BOOST_CHECK_SMALL(diffNorm(op1 * spline, multiplicator * spline), tol);
    BOOST_CHECK_SMALL(
        diffNorm(op2 * spline, Dx<1>{} * (multiplicator * spline)), tol);
  }
}

template <typename T, size_t size>
static std::array<T, size> multiplyArray(std::array<T, size> array, T val) {
  for (auto &x : array) {
    x *= val;
  }
  return array;
}

BOOST_AUTO_TEST_SUITE(ScalarMultiplicationTestSuite)
/**
 * Passes if an operator that has been rescaled correctly transforms a
 * coefficient array.
 */
BOOST_AUTO_TEST_CASE(TransformTest) {
  const double multiplicator = 313.0 / 17;
  const auto op = multiplicator * IdentityOperator{};
  const support::Grid<double> grid{{3.0, 4.0, 5.0, 6.0}};

  {
    std::array<double, 1> arr{3.0};
    BOOST_TEST(multiplyArray(arr, multiplicator) == op.transform(arr, grid, 1));
  }

  {
    std::array<double, 4> arr{3.0, 70.0, -33.7373, -178.89};
    BOOST_TEST(multiplyArray(arr, multiplicator) == op.transform(arr, grid, 0));
  }

  {
    std::array<double, 10> arr;
    arr.fill(3.14);
    BOOST_TEST(multiplyArray(arr, multiplicator) ==
               op.transform(arr, grid, 10));
  }
}

/**
 * Passes if the application of an operator which has been rescaled to a spline
 * yields the expected results.
 */
BOOST_AUTO_TEST_CASE(MultiplySplineTest) {
  constexpr double dtol = 1.0e-15;
  testSplineMultiplication<double, 3>(dtol);
  testSplineMultiplication<double, 5>(dtol);
  testSplineMultiplication<double, 10>(dtol);
  testSplineMultiplication<double, 15>(dtol);
  testSplineMultiplication<double, 20>(dtol);

  if constexpr (sizeof(long double) != sizeof(double)) {
    constexpr long double ldtol = 1.0e-18l;
    testSplineMultiplication<long double, 3>(ldtol);
    testSplineMultiplication<long double, 5>(ldtol);
    testSplineMultiplication<long double, 10>(ldtol);
    testSplineMultiplication<long double, 15>(ldtol);
    testSplineMultiplication<long double, 20>(ldtol);
  }
}

BOOST_AUTO_TEST_SUITE_END()
