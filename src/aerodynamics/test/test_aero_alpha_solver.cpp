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

  double alpha = solver->solve(V, S_ap_x, S_ap_z, mass);

  // Should converge to a reasonable alpha value
  EXPECT_GT(alpha, -M_PI);
  EXPECT_LT(alpha, M_PI);
  EXPECT_LE(solver->getLastIterationCount(), 20);
}

TEST_F(AeroAlphaSolverTest, TemporalCoherence)
{
  double V = 10.0;
  double S_ap_x = 1.0;
  double S_ap_z = 9.81;
  double mass = 2.0;

  solver->resetAlpha(45.0);  // Start at 45 degrees

  double alpha1 = solver->solve(V, S_ap_x, S_ap_z, mass);
  double alpha2 = solver->solve(V, S_ap_x, S_ap_z, mass);

  // Second solve should give the same result
  EXPECT_DOUBLE_EQ(alpha1, alpha2);
}

TEST_F(AeroAlphaSolverTest, ResetAlphaWorks)
{
  solver->resetAlpha(30.0);
  solver->resetAlpha(60.0);

  // Should not throw, just verify it works
  SUCCEED();
}

TEST_F(AeroAlphaSolverTest, ConfigMethodsWork)
{
  AeroAlphaSolver::Config config;
  config.max_iterations = 100;
  config.tolerance = 1e-10;

  solver->setConfig(config);

  auto retrieved_config = solver->getConfig();
  EXPECT_EQ(retrieved_config.max_iterations, 100);
  EXPECT_DOUBLE_EQ(retrieved_config.tolerance, 1e-10);
}

TEST_F(AeroAlphaSolverTest, InvalidInputThrows)
{
  EXPECT_THROW(solver->solve(10.0, 0.0, 9.81, -1.0), std::invalid_argument);
  EXPECT_THROW(solver->solve(10.0, 0.0, 9.81, 0.0), std::invalid_argument);
}
