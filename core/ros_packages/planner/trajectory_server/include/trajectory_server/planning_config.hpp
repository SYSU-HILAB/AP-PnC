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

namespace trajectory_server
{

  /**
   * @brief Safe Flight Corridor box dimensions [m]
   */
  struct BoxCfg
  {
    double side_inside  = 0.1;
    double side_outside = 1.0;
    double up           = 0.5;
    double down         = 0.1;
    double front        = 1.5;
    double back         = 1.5;
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
    double v_max      = 10.0;
    double omg_x_max  = 3.0;
    double omg_y_max  = 3.0;
    double omg_z_max  = 3.0;
    double acc_max    = 1.0;
    double thrust_min = 1.0;
    double thrust_max = 20.0;

    double vel_weight            = 1e4;
    double acc_weight            = 1e4;
    double omg_x_weight          = 1e3;
    double omg_y_weight          = 1e3;
    double omg_z_weight          = 1e3;
    double thrust_weight         = 1e5;
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
  };

  /**
   * @brief The optimization problem itself: frame, scale, corridor, seed
   * curve, cost, trajectory parameterization, and vehicle flatness model
   */
  struct ProblemCfg
  {
    std::string frame     = "world";
    double      uav_scale = 1.0;
    int         piece_n   = 10;

    BoxCfg       box;
    BasicTrajCfg basic_traj;
    CostCfg      cost;
    MincoCfg     minco;
    FlatnessCfg  flatness;
  };

  /**
   * @brief Inbound/outbound topics and dump paths
   */
  struct IoCfg
  {
    std::string traj_topic    = "control_traj";
    double      traj_pub_rsl  = 0.02;
    std::string temp_data_dir = "artifacts/simulations/temp_data";
  };

  /**
   * @brief Aggregate planning configuration, loaded from
   * bringup/config/planning.yaml (see AP_PNC_PLANNING_YAML for override)
   */
  struct Config
  {
    ProblemCfg problem;
    IoCfg      io;

    /// gcopter consumes its own singleton; load() mirrors problem.cost into it
    gcopter::CostConfig &cost_config;

    Config() : cost_config(gcopter::CostConfig::getInstance()) {}

    /**
     * Parse the sectioned yaml and mirror cost settings into the singleton.
     *
     * @param[in] yaml_path Path to planning.yaml [path]
     */
    void load(const std::string &yaml_path);
  };

}  // namespace trajectory_server
