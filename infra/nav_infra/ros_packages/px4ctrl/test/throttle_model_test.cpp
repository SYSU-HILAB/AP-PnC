/*
 * BSD 3-Clause License
 * Copyright (c) 2025, Sun Yat-sen University. All rights reserved.
 * Authors: Hanamy: rongerch@outlook.com
 *
 * Unit tests for the px4ctrl RLS throttle model and INDI thrust law
 * (see rls_throttle_model.h).
 */

#include <gtest/gtest.h>

#include "rls_throttle_model.h"

namespace
{
  /**
   * Configure a default estimator mirroring the sim defaults.
   *
   * @param[in,out] model Estimator under test [-]
   */
  void configureDefault(RlsThrottleModel &model)
  {
    model.setForgettingFactor(0.999);
    model.setEtaBounds(9.81, 40.0);
    model.setMinThrustForUpdate(0.10);
    model.setPInit(10.0);
    model.setVarValidThreshold(0.01);
    model.setInnovationGate(5.0);
    model.setMeasurementVariance(4.0);
  }
}  // namespace

TEST(RlsThrottleModel, ConvergesToTrueEffectiveness)
{
  RlsThrottleModel model;
  configureDefault(model);
  model.reset(RlsThrottleModel::kOneG / 0.5);  // 19.6 m/s^2

  const double eta_true = 15.0;
  for (int i = 0; i < 20000; ++i)
  {
    model.step(0.5, eta_true * 0.5);
  }
  EXPECT_NEAR(model.getEta(), eta_true, 0.5);
  EXPECT_TRUE(model.isEstimateValid());
}

TEST(RlsThrottleModel, RespectsBounds)
{
  RlsThrottleModel model;
  configureDefault(model);
  model.setEtaBounds(9.81, 25.0);
  model.reset(9.81);

  const double eta_true = 60.0;  // above eta_max
  for (int i = 0; i < 5000; ++i)
  {
    model.step(0.5, eta_true * 0.5);
  }
  EXPECT_DOUBLE_EQ(model.getEta(), 25.0);
}

TEST(RlsThrottleModel, NoUpdateBelowMinThrust)
{
  RlsThrottleModel model;
  configureDefault(model);
  model.reset(9.81);
  const double eta_before = model.getEta();

  for (int i = 0; i < 1000; ++i)
  {
    model.step(0.01, 20.0);  // below u_min
  }
  EXPECT_DOUBLE_EQ(model.getEta(), eta_before);
}

TEST(RlsThrottleModel, InnovationGateRejectsOutliers)
{
  RlsThrottleModel model;
  configureDefault(model);
  model.reset(15.0);

  for (int i = 0; i < 5000; ++i)
  {
    model.step(0.5, 15.0 * 0.5);  // settle
  }
  const double eta_before = model.getEta();

  model.step(0.5, 200.0);  // violent outlier
  EXPECT_NEAR(model.getEta(), eta_before, 1.0e-9);
}

TEST(RlsThrottleModel, ResetSeedsEstimate)
{
  RlsThrottleModel model;
  configureDefault(model);
  model.reset(22.0);
  EXPECT_DOUBLE_EQ(model.getEta(), 22.0);
}

TEST(IndiThrottle, PushesTowardsSetpointAndClamps)
{
  // Below setpoint: throttle must increase.
  const double u_up =
      computeIndiThrottle(0.4, 20.0, 12.0, 9.81, 2.0, 0.02, 0.05, 1.0);
  EXPECT_GT(u_up, 0.4);

  // Above setpoint: throttle must decrease.
  const double u_down =
      computeIndiThrottle(0.8, 20.0, 10.0, 15.0, 2.0, 0.02, 0.05, 1.0);
  EXPECT_LT(u_down, 0.8);

  // Large error saturates at the ceiling.
  const double u_sat =
      computeIndiThrottle(0.9, 20.0, 40.0, 0.0, 50.0, 0.02, 0.05, 1.0);
  EXPECT_DOUBLE_EQ(u_sat, 1.0);

  // Zero error holds the current throttle.
  const double u_hold =
      computeIndiThrottle(0.6, 20.0, 9.81, 9.81, 2.0, 0.02, 0.05, 1.0);
  EXPECT_DOUBLE_EQ(u_hold, 0.6);
}
