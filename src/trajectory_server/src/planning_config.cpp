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

#include <trajectory_server/planning_config.hpp>

namespace trajectory_server
{

Config::Config(rclcpp::Node* node) : cost_config(gcopter::CostConfig::getInstance())
{
  frame = node->declare_parameter("Frame", "world");

  mass = node->declare_parameter("mass", 2.0);

  uavScale = node->declare_parameter("UavScale", 1.0);

  smoothingEps    = node->declare_parameter("SmoothingEps", 1e-2);
  integralIntervs = node->declare_parameter("IntegralIntervs", 16);
  relCostTol      = node->declare_parameter("RelCostTol", 1e-4);
  pieceN          = node->declare_parameter("PieceN", 10);

  box_side_inside_length  = node->declare_parameter("BoxSideInsideLength", 0.1);
  box_side_outside_length = node->declare_parameter("BoxSideOutsideLength", 1.0);
  box_down_length         = node->declare_parameter("BoxDownLength", 0.1);
  box_up_length           = node->declare_parameter("BoxUpLength", 0.5);
  box_front_length        = node->declare_parameter("BoxFrontLength", 1.5);
  box_back_length         = node->declare_parameter("BoxBackLength", 1.5);

  basic_trajectory_rate = node->declare_parameter("BasicTrajectoryRate", 0.8);
  basic_trajectory_radius = node->declare_parameter("BasicTrajectoryRadius", 10.0);
  basic_trajectory_height = node->declare_parameter("BasicTrajectoryHeight", 5.0);
  basic_trajectory_type   = node->declare_parameter("BasicTrajectoryType", 1);
  basic_trajectory_curvature_factor =
      node->declare_parameter("BasicTrajectoryCurvatureFactor", 0.0);
  basic_traj_wave_len = node->declare_parameter("BasicTrajectoryWaveLen", 10.0);

  trajecotry_cirled = node->declare_parameter("BasicTrajectoryCirled", true);

  // pub config
  traj_pub_rsl = node->declare_parameter("traj_pub_rsl", 0.02);
  trajTopic    = node->declare_parameter("trajTopic", "control_traj");
  // cost_config
  cost_config.v_max     = node->declare_parameter("v_max", 10.0);
  cost_config.omg_x_max = node->declare_parameter("omg_x_max", 3.0);
  cost_config.omg_y_max = node->declare_parameter("omg_y_max", 3.0);
  cost_config.omg_z_max = node->declare_parameter("omg_z_max", 3.0);

  cost_config.thrust_max = node->declare_parameter("thrust_max", 20.0);
  cost_config.thrust_min = node->declare_parameter("thrust_min", 1.0);
  cost_config.acc_max    = node->declare_parameter("acc_max", 1.0);

  cost_config.thrust_weight      = node->declare_parameter("thrust_weight", 1e5);
  cost_config.vel_weight         = node->declare_parameter("vel_weight", 1e4);
  cost_config.acc_weight         = node->declare_parameter("acc_weight", 1e4);
  cost_config.omg_x_weight       = node->declare_parameter("omg_x_weight", 1e3);
  cost_config.omg_y_weight       = node->declare_parameter("omg_y_weight", 1e3);
  cost_config.omg_z_weight       = node->declare_parameter("omg_z_weight", 1e3);
  cost_config.time_weight        = node->declare_parameter("time_weight", 1e2 * 5);
  cost_config.omg_consistent_weight =
      node->declare_parameter("omg_consistent_weight", 1e3);
}

void Config::UpdateConfig(const rclcpp::Node* node)
{
  node->get_parameter("SmoothingEps", smoothingEps);
  node->get_parameter("IntegralIntervs", integralIntervs);
  node->get_parameter("RelCostTol", relCostTol);
  node->get_parameter("UavScale", uavScale);

  node->get_parameter("FinVel", finVel);
  node->get_parameter("PieceN", pieceN);

  node->get_parameter("BoxSideInsideLength", box_side_inside_length);
  node->get_parameter("BoxSideOutsideLength", box_side_outside_length);
  node->get_parameter("BoxUpLength", box_up_length);
  node->get_parameter("BoxDownLength", box_down_length);
  node->get_parameter("BoxFrontLength", box_front_length);
  node->get_parameter("BoxBackLength", box_back_length);

  node->get_parameter("Frame", frame);

  node->get_parameter("mass", mass);

  node->get_parameter("BasicTrajectoryRate", basic_trajectory_rate);
  node->get_parameter("BasicTrajectoryRadius", basic_trajectory_radius);
  node->get_parameter("BasicTrajectoryHeight", basic_trajectory_height);
  node->get_parameter("BasicTrajectoryType", basic_trajectory_type);
  node->get_parameter("BasicTrajectoryCurvatureFactor",
                      basic_trajectory_curvature_factor);
  node->get_parameter("BasicTrajectoryWaveLen", basic_traj_wave_len);
  node->get_parameter("TrajectoryCirled", trajecotry_cirled);
  // update cost_config
  node->get_parameter("v_max", cost_config.v_max);
  node->get_parameter("omg_x_max", cost_config.omg_x_max);
  node->get_parameter("omg_y_max", cost_config.omg_y_max);
  node->get_parameter("omg_z_max", cost_config.omg_z_max);
  node->get_parameter("thrust_max", cost_config.thrust_max);
  node->get_parameter("thrust_min", cost_config.thrust_min);
  node->get_parameter("acc_max", cost_config.acc_max);
  node->get_parameter("thrust_weight", cost_config.thrust_weight);
  node->get_parameter("vel_weight", cost_config.vel_weight);
  node->get_parameter("omg_x_weight", cost_config.omg_x_weight);
  node->get_parameter("omg_y_weight", cost_config.omg_y_weight);
  node->get_parameter("omg_z_weight", cost_config.omg_z_weight);
  node->get_parameter("time_weight", cost_config.time_weight);
  node->get_parameter("acc_weight", cost_config.acc_weight);
  node->get_parameter("omg_consistent_weight", cost_config.omg_consistent_weight);

  node->get_parameter("traj_pub_rsl", traj_pub_rsl);
  node->get_parameter("trajTopic", trajTopic);
}

}  // namespace trajectory_server
