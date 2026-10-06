/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

/**
 * @file test_bspline_basis.cpp
 * @brief Golden test for the self-contained cubic B-spline basis.
 *
 * Reference values were produced with scipy:
 *   BSpline(knots, coefs, 3)(xs)
 * for the clamped knot vector below, so this pins the replacement of the
 * third-party B-spline library.
 */

#include <gtest/gtest.h>

#include <aerodynamics/bspline_basis.hpp>
#include <vector>

namespace
{
  const std::vector<double> kKnots = {0.0,  0.0, 0.0, 0.0, 0.25, 0.5,
                                      0.75, 1.0, 1.0, 1.0, 1.0};
  const std::vector<double> kCoefs = {1.0, 2.0, 0.5, -1.0, 3.0, 2.5, -0.5};
  const std::vector<double> kX     = {0.0,  0.1,  0.25, 0.33, 0.5,
                                      0.66, 0.75, 0.9,  1.0};
  const std::vector<double> kRef   = {
      1.0,
      1.4800000000000002,
      0.625,
      0.08934133333333322,
      -0.08333333333333329,
      1.3952373333333337,
      2.208333333333333,
      1.905333333333333,
      -0.5,
  };
}  // namespace

TEST(BsplineBasis, NumBasis)
{
  EXPECT_EQ(aerodynamics::bspline_num_basis(kKnots.size(), 3), 7u);
  EXPECT_EQ(aerodynamics::bspline_num_basis(3, 3), 0u);
}

TEST(BsplineBasis, PartitionOfUnity)
{
  for (double x : kX)
  {
    const std::vector<double> basis = aerodynamics::bspline_basis(3, kKnots, x);
    ASSERT_EQ(basis.size(), kCoefs.size());
    double sum = 0.0;
    for (double v : basis)
    {
      sum += v;
    }
    EXPECT_NEAR(sum, 1.0, 1e-12) << "at x=" << x;
  }
}

TEST(BsplineBasis, MatchesScipyReference)
{
  for (std::size_t i = 0; i < kX.size(); ++i)
  {
    EXPECT_NEAR(aerodynamics::evaluate_bspline(kCoefs, kKnots, 3, kX[i]),
                kRef[i], 1e-12)
        << "at x=" << kX[i];
  }
}
