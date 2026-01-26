/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#include <gtest/gtest.h>

#include <aerodynamics/aero_alpha_solver.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

using namespace aerodynamics;

class AeroAlphaSolverTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    aero_interface = std::make_shared<LyuAerodynamics>();
    solver = std::make_unique<AeroAlphaSolver>(aero_interface);
  }

  std::shared_ptr<AerodynamicsInterface> aero_interface;
  std::unique_ptr<AeroAlphaSolver> solver;
};

TEST_F(AeroAlphaSolverTest, ConvergesForTypicalConditions)
{
  // Typical flight conditions
  double V = 10.0;      // 10 m/s
  double S_ap_x = 0.0;  // Level flight
  double S_ap_z = 9.81; // Hover
  double mass = 2.0;    // 2 kg

  auto result = solver->solve(V, S_ap_x, S_ap_z, mass);

  // Should converge to a reasonable alpha value
  EXPECT_GT(result.alpha, -M_PI);
  EXPECT_LT(result.alpha, M_PI);
  EXPECT_LE(solver->getLastIterationCount(), 20);
  EXPECT_LT(result.residual, 1e-6);  // Residual should be small
}

TEST_F(AeroAlphaSolverTest, TemporalCoherence)
{
  double V = 10.0;
  double S_ap_x = 1.0;
  double S_ap_z = 9.81;
  double mass = 2.0;

  solver->resetAlpha(45.0);  // Start at 45 degrees

  auto result1 = solver->solve(V, S_ap_x, S_ap_z, mass);
  auto result2 = solver->solve(V, S_ap_x, S_ap_z, mass);

  // Second solve should give the same result
  EXPECT_DOUBLE_EQ(result1.alpha, result2.alpha);
}

TEST_F(AeroAlphaSolverTest, ResetAlphaWorks)
{
  solver->resetAlpha(30.0);
  solver->resetAlpha(60.0);

  // Should not throw, just verify it works
  SUCCEED();
}

TEST_F(AeroAlphaSolverTest, InvalidInputThrows)
{
  EXPECT_THROW(solver->solve(10.0, 0.0, 9.81, -1.0), std::invalid_argument);
  EXPECT_THROW(solver->solve(10.0, 0.0, 9.81, 0.0), std::invalid_argument);
}

TEST_F(AeroAlphaSolverTest, AngleUnwrappingMaintainsContinuity)
{
  // Test that angle unwrapping prevents discontinuities > π
  double V = 10.0;
  double mass = 2.0;

  solver->resetAlpha(170.0);  // Start near +π (170 degrees)

  std::vector<double> S_ap_x_values = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> alphas;

  for (double S_ap_x : S_ap_x_values)
  {
    auto result = solver->solve(V, S_ap_x, 9.81, mass);
    alphas.push_back(result.alpha);
  }

  // Check that consecutive alpha differences are small (< π)
  for (size_t i = 1; i < alphas.size(); ++i)
  {
    double delta = std::abs(alphas[i] - alphas[i - 1]);
    EXPECT_LT(delta, M_PI) << "Discontinuity detected between step "
                          << (i - 1) << " and " << i;
  }
}

TEST_F(AeroAlphaSolverTest, AngleUnwrappingAcrossMultipleWraps)
{
  // Test multiple wrap events in sequence
  double V = 10.0;
  double mass = 2.0;

  solver->resetAlpha(0.0);

  auto prev_result = solver->solve(V, 0.0, 9.81, mass);
  double prev_alpha = prev_result.alpha;

  // Simulate many iterations that would cause wrapping
  for (int i = 0; i < 100; ++i)
  {
    auto result = solver->solve(V, i * 0.1, 9.81, mass);
    double delta = std::abs(result.alpha - prev_alpha);

    // No jump should exceed π
    EXPECT_LT(delta, M_PI) << "Large jump at iteration " << i;
    prev_alpha = result.alpha;
  }
}

TEST_F(AeroAlphaSolverTest, ResetUnwrappingClearsAccumulatedOffset)
{
  double V = 10.0;
  double mass = 2.0;

  // Generate a sequence of alphas to accumulate offset
  solver->resetAlpha(0.0);
  auto result1 = solver->solve(V, 5.0, 9.81, mass);
  auto result2 = solver->solve(V, 5.0, 9.81, mass);
  double alpha1 = result1.alpha;
  double alpha2 = result2.alpha;

  // Now reset and verify the unwrapping state is cleared
  solver->resetUnwrapping(45.0);

  // Solve again - should use fresh initial guess without accumulated offset
  auto result3 = solver->solve(V, 5.0, 9.81, mass);
  auto result4 = solver->solve(V, 5.0, 9.81, mass);
  double alpha3 = result3.alpha;
  double alpha4 = result4.alpha;

  // The two consecutive solves after reset should be identical
  // (temporal coherence still works)
  EXPECT_DOUBLE_EQ(alpha3, alpha4);

  // The solves before and after reset should converge to the same physical value
  // (since they solve the same force balance equation)
  // They might differ by a constant offset due to wrapping, but the wrapped
  // values should be very close
  double wrapped_alpha2 = std::fmod(std::fmod(alpha2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  if (wrapped_alpha2 > M_PI) wrapped_alpha2 -= 2 * M_PI;

  double wrapped_alpha4 = std::fmod(std::fmod(alpha4, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  if (wrapped_alpha4 > M_PI) wrapped_alpha4 -= 2 * M_PI;

  EXPECT_NEAR(wrapped_alpha2, wrapped_alpha4, 0.01);
}
