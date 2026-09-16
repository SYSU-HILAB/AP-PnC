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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <trajectory_server/trajectory_planner.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto global_planner = std::make_shared<trajectory_server::GlobalPlanner>();
  rclcpp::spin(global_planner);
  rclcpp::shutdown();

  return 0;
}
