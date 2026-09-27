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

#include <cstddef>
#include <deque>

#include <Eigen/Dense>
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <planner_core/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace planner::node
{

using TrajecotryPoint  = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory       = interface::msg::RefTrajectory;

/**
 * @brief Reference buffer configuration.
 *
 * capacity == 1 reproduces the historic single-trajectory behaviour
 * (replanning disabled). capacity >= 2 keeps the previous segment(s) alive
 * so a replanned trajectory can be committed at a segment boundary.
 */
struct TrajectoryBufferConfig
{
  std::size_t capacity             = 1;
  int         down_sampling_factor = 5;
  double      traj_control_time    = 0.1;
  bool        circled              = false;
};

/**
 * @brief ROS-side segmented reference buffer.
 *
 * Converts the pure planner_core::Trajectory into the flight message, keeps a
 * bounded time-ordered FIFO of segments with a per-segment cursor, and emits
 * the next down-sampled reference bullet on every /ref_can_set tick.
 * Eviction is in-order retire, not LRU.
 */
class TrajectoryBuffer
{
 public:
  enum class RefTrajType
  {
    PLANNING,
    FIXED
  };

  /**
   * @brief Constructor
   *
   * @param node Node the buffer runs under (interfaces are created on it)
   * @param config Buffer configuration
   */
  explicit TrajectoryBuffer(rclcpp::Node&        node,
                            TrajectoryBufferConfig config = {});

  // Disable copy and move
  TrajectoryBuffer(const TrajectoryBuffer&)            = delete;
  TrajectoryBuffer& operator=(const TrajectoryBuffer&) = delete;
  TrajectoryBuffer(TrajectoryBuffer&&)                 = delete;
  TrajectoryBuffer& operator=(TrajectoryBuffer&&)      = delete;

  /**
   * @brief Replan entry point: enqueue a newly planned reference trajectory.
   *
   * @param traj Pure trajectory produced by planner_core::Planner
   */
  void push(const core::Trajectory& traj);

  /// Drop all buffered segments and fall back to the fixed hold point.
  void clear();

  /// Toggle between PLANNING (follow buffer) and FIXED (hold) reference.
  void toggleMode();

 private:
  /// A single buffered trajectory plus its read cursor.
  struct Segment
  {
    Trajectory  traj;
    std::size_t cursor = 0;
  };

  static Trajectory to_msg(const core::Trajectory& traj);

  void initInterfaces();
  void onRefCanSet(const std_msgs::msg::Bool::SharedPtr msg);
  void onMode(const std_msgs::msg::Bool::SharedPtr msg);
  void onState(const interface::msg::State::SharedPtr msg);
  bool checkTrajectoryTermination(Segment& segment);

  rclcpp::Node&          node_;
  TrajectoryBufferConfig config_;

  /// Time-ordered segments; front is the segment currently being streamed.
  std::deque<Segment> segments_;

  RefTrajType traj_type_     = RefTrajType::FIXED;
  bool        init_traj_set_ = false;

  Eigen::Vector3d init_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_yb_  = Eigen::Vector3d(0.0, 1.0, 0.0);

  TrajecotryBullet msg_planning_traj_;
  TrajecotryBullet msg_fixed_point_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   ref_can_set_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   mode_trigger_sub_;
  rclcpp::Subscription<interface::msg::State>::SharedPtr state_sub_;

  rclcpp::Publisher<TrajecotryBullet>::SharedPtr traj_bullet_pub_;

  const rclcpp::QoS reliabe_qos_keep_last =
      rclcpp::QoS(10)
          .reliable()
          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
          .keep_last(1);
};

}  // namespace planner::node
