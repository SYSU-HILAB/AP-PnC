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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#pragma once

#include <gcopter/gcopter.hpp>
#include <string>
#include <vector>

namespace planner::core
{

  /**
   * @brief Safe Flight Corridor box dimensions [m]
   */
  struct BoxCfg
  {
    // Axis-aligned SFC half-extents [m]; left/right are -y/+y half-lengths.
    double left  = 0.1;
    double right = 1.0;
    double up    = 0.5;
    double down  = 0.1;
    double front = 1.5;
    double back  = 1.5;
  };

  /**
   * @brief Reference (basic) trajectory shape parameters
   */
  struct BasicTrajCfg
  {
    int    type   = 1;  // 1 Lemniscate / 2 Circle / 3 Sinusoidal / 4 Line
    double radius = 10.0;
    double height = 5.0;
    double rate   = 0.8;
    double curvature_factor = 0.0;
    double wave_len         = 10.0;
  };

  /**
   * @brief gcopter cost weights and dynamic limits (mirrored into the
   * gcopter::CostConfig singleton on load)
   */
  struct CostCfg
  {
    double v_max              = 10.0;
    double omg_x_max          = 3.0;
    double omg_y_max          = 3.0;
    double omg_z_max          = 3.0;
    double acc_max            = 1.0;
    double tangential_acc_min = 1.0;
    double tangential_acc_max = 20.0;
    // Collective specific thrust a_T [N/kg]; the real thrust, unlike the
    // frame-free tangential acceleration above.
    double thrust_min    = 2.0;
    double thrust_max    = 22.0;
    double thrust_weight = 1e4;

    double vel_weight            = 1e4;
    double acc_weight            = 1e4;
    double omg_x_weight          = 1e3;
    double omg_y_weight          = 1e3;
    double omg_z_weight          = 1e3;
    double tangential_acc_weight = 1e5;
    double time_weight           = 5e2;
    double omg_consistent_weight = 1e3;
  };

  /**
   * @brief MINCO trajectory optimization parameters
   */
  struct MincoCfg
  {
    double smoothing_eps    = 1e-2;
    int    integral_intervs = 16;
    double rel_cost_tol     = 1e-4;
  };

  /**
   * @brief Prior-free flatness model parameters (phi-theory linear aero map)
   */
  struct FlatnessCfg
  {
    std::vector<double> phi  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.09408};
    double              mass = 2.0;
    // ENU gravity used by the FLU flatness map, m/s^2.
    double gravity = 9.81;
  };

  /**
   * @brief The optimization problem itself: corridor, seed curve, cost,
   * trajectory parameterization, and vehicle flatness model
   */
  struct ProblemCfg
  {
    int piece_n = 10;

    BoxCfg       box;
    BasicTrajCfg basic_traj;
    CostCfg      cost;
    MincoCfg     minco;
    FlatnessCfg  flatness;
  };

  /**
   * @brief Aggregate OCP configuration, loaded from a sectioned yaml
   * (bringup/config/planning.yaml).
   */
  struct ProblemConfig
  {
    ProblemCfg problem;

    /// gcopter consumes its own singleton; load() mirrors problem.cost into it
    gcopter::CostConfig &cost_config;

    ProblemConfig() : cost_config(gcopter::CostConfig::getInstance()) {}

    /**
     * Parse the sectioned yaml and mirror cost settings into the singleton.
     *
     * @param[in] yaml_path Path to planning.yaml [path]
     */
    void load(const std::string &yaml_path);
  };

}  // namespace planner::core
