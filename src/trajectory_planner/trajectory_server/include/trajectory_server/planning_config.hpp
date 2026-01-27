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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <gcopter/gcopter.hpp>

namespace trajectory_server
{

/**
 * @brief Configuration for global trajectory planning
 *
 * Manages ROS2 parameters and cost configuration for the trajectory planner.
 * Loaded from ROS2 parameters and provides update methods for runtime reconfiguration.
 */
struct Config
{
  // Smoothing and optimization parameters
  double smoothingEps;
  int    integralIntervs;
  double relCostTol;
  double uavScale;

  // Trajectory publish time resolution
  double              traj_pub_rsl = 0.02;
  std::string         trajTopic    = "control_traj";
  std::vector<double> finVel;

  // Trajectory piece configuration
  int pieceN = 20;              // 20 pieces means 21 points including start and end
  int controlPoints = pieceN - 1;

  // Safe Flight Corridor (SFC) box dimensions
  double box_side_inside_length  = 1;
  double box_side_outside_length = 1;
  double box_up_length           = 1;
  double box_down_length         = 0.1;
  double box_front_length        = 1;
  double box_back_length         = 1;

  // Reference frame
  std::string frame = "world";

  // Basic trajectory parameters
  double basic_trajectory_rate             = 0.8;
  double basic_trajectory_radius           = 10;
  double basic_trajectory_height           = 5;
  int    basic_trajectory_type             = 1;
  double basic_trajectory_curvature_factor = 0.0;
  double basic_traj_wave_len               = 10.0;

  // Vehicle mass
  double mass = 2.0;  // Vehicle mass in kg (default from tailsitter_df config)

  // Reference to singleton cost configuration
  gcopter::CostConfig& cost_config;

  /**
   * @brief Constructor - declares and loads ROS2 parameters
   *
   * @param node ROS2 node pointer for parameter declaration
   */
  Config(rclcpp::Node* node);

  /**
   * @brief Update configuration from current node parameters
   *
   * @param node ROS2 node pointer for parameter retrieval
   */
  void UpdateConfig(const rclcpp::Node* node);
};

}  // namespace trajectory_server
