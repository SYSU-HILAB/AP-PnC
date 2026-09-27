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

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interface/msg/ref_trajectory.hpp>

#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>

#include <basic_trajectories/circle.hpp>
#include <basic_trajectories/lemniscate.hpp>
#include <basic_trajectories/line.hpp>
#include <basic_trajectories/sin.hpp>

#include <aerodynamics/aero_alpha_solver.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

#include <trajectory_server/planning_config.hpp>

using RefTrajectory   = interface::msg::RefTrajectory;
using TrajectoryPoint = interface::msg::TrajectoryPoint;

namespace trajectory_server
{

/**
 * @brief Basic trajectory type enumeration
 */
enum class BasicTrajectoryType
{
  LEMNISCATE = 1,
  CIRCLE     = 2,
  SIN        = 3,
  LINE       = 4
};

/**
 * @brief Global trajectory planner ROS2 node
 *
 * Generates optimized trajectories using GCOPTER with polytope SFC constraints.
 * Publishes reference trajectories for the MPC controller.
 */
class GlobalPlanner : public rclcpp::Node
{
 public:
  /**
   * @brief Constructor
   *
   * Initializes ROS2 node, loads configuration, and sets up publishers.
   */
  GlobalPlanner();

  /**
   * @brief Destructor
   */
  ~GlobalPlanner() override = default;

  // Disable copy and move
  GlobalPlanner(const GlobalPlanner&)            = delete;
  GlobalPlanner& operator=(const GlobalPlanner&) = delete;
  GlobalPlanner(GlobalPlanner&&)                 = delete;
  GlobalPlanner& operator=(GlobalPlanner&&)      = delete;

 private:
  /**
   * @brief Initialize basic trajectory type factory
   *
   * Creates the appropriate trajectory object based on configuration.
   *
   * @param type Trajectory type code (1=lemniscate, 2=circle, 3=sin, 4=line)
   */
  void init_basic_trajectory_type(const int type);

  /**
   * @brief Create Safe Flight Corridor (SFC) around basic trajectory waypoints
   *
   * Generates polytope constraints (boxes) around each waypoint for the optimizer.
   *
   * @param hPolys Output vector of half-space representation polytopes
   */
  void create_basic_trajecotry_sfc(std::vector<Eigen::MatrixX4d>& hPolys);

  /**
   * @brief Main planning method
   *
   * Sets up GCOPTER optimization, solves for trajectory, samples results,
   * and publishes reference trajectory message.
   */
  void plan();

  // Configuration
  Config config;

  // ROS2 interfaces
  rclcpp::Publisher<RefTrajectory>::SharedPtr trajPub;

  // Trajectory data
  RefTrajectory                                     control_traj_msg;
  Trajectory<5>                                     traj;
  double                                            trajStamp;
  std::unique_ptr<basic_trajectories::Trajectory>   basic_traj;
  rclcpp::TimerBase::SharedPtr                      timer;

  // State flag
  bool init_setup = false;
};

}  // namespace trajectory_server
