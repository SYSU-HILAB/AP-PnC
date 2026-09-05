/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/BSplineGenerator.h>
#include <bspline/operators/GenericOperators.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <type_traits>

using namespace bspline;
using namespace bspline::operators;

static_assert(std::is_nothrow_move_constructible_v<Operator> &&
                  std::is_nothrow_move_assignable_v<Operator>,
              "Operator is not nothrow moveable.");

static_assert(std::is_nothrow_move_constructible_v<IdentityOperator> &&
                  std::is_nothrow_move_assignable_v<IdentityOperator>,
              "IdentityOperator is not nothrow moveable");

template <typename T, size_t order>
static void testSplineMultiplication() {
  const IdentityOperator op;
  std::vector<T> knots;
  for (size_t i = 0; i <= order + 4; i++) {
    knots.push_back(static_cast<T>(i));
  }
  const BSplineGenerator generator{knots};

  const auto splines = generator.template generateBSplines<order>();

  for (const auto &spline : splines) {
    const auto transformedSpline = op * spline;
    const bool splinesEqual = (spline == transformedSpline);
    BOOST_TEST(splinesEqual);
  }
}

BOOST_AUTO_TEST_SUITE(IdentityOperatorTestSuite)
/**
 * Passes if the IndentityOperator returns the coefficient array unchanged.
 */
BOOST_AUTO_TEST_CASE(TransformTest) {
  const IdentityOperator op;
  const support::Grid<double> grid{{3.0, 4.0, 5.0, 6.0}};

  {
    std::array<double, 1> arr{3.0};
    BOOST_TEST(arr == op.transform(arr, grid, 1));
  }

  {
    std::array<double, 4> arr{3.0, 70.0, -33.7373, -178.89};
    BOOST_TEST(arr == op.transform(arr, grid, 0));
  }

  {
    std::array<double, 10> arr;
    arr.fill(3.14);
    BOOST_TEST(arr == op.transform(arr, grid, 10));
  }
}

/**
 * Passes if the application of the IndentityOperator to a Spline returns the
 * unchanged Spline.
 */
BOOST_AUTO_TEST_CASE(MultiplySplineTest) {
  testSplineMultiplication<double, 3>();
  testSplineMultiplication<double, 5>();
  testSplineMultiplication<double, 10>();
  testSplineMultiplication<double, 15>();
  testSplineMultiplication<double, 20>();

  if constexpr (sizeof(long double) != sizeof(double)) {
    testSplineMultiplication<long double, 3>();
    testSplineMultiplication<long double, 5>();
    testSplineMultiplication<long double, 10>();
    testSplineMultiplication<long double, 15>();
    testSplineMultiplication<long double, 20>();
  }
}

BOOST_AUTO_TEST_SUITE_END()
