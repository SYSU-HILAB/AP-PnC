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

#include <vector>

#include <Eigen/Dense>

namespace planner::core
{

/**
 * @brief One sampled point of a planned reference trajectory.
 *
 * Pure data: no ROS message types. The ROS adapter maps this into the
 * flight message, and the Python binding exposes it as numpy arrays.
 */
struct TrajectorySample
{
  double          t = 0.0;  ///< time since trajectory start [s]
  Eigen::Vector3d p = Eigen::Vector3d::Zero();  ///< position, world [m]
  Eigen::Vector3d v = Eigen::Vector3d::Zero();  ///< velocity, world [m/s]
  Eigen::Vector3d a = Eigen::Vector3d::Zero();  ///< acceleration, world [m/s^2]
  Eigen::Vector3d yb =
      Eigen::Vector3d(0.0, 1.0, 0.0);            ///< body y-axis, quad FLU
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();  ///< body rate [rad/s]
  double          thrust = 0.0;  ///< flatness collective thrust [N]
};

/**
 * @brief A planned reference trajectory (ROS-free, time-ordered samples).
 */
struct Trajectory
{
  std::vector<TrajectorySample> samples;
  double                        duration = 0.0;  ///< total duration [s]
};

}  // namespace planner::core
