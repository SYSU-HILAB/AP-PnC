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
#include <interface/msg/reference_horizon.hpp>
#include <interface/msg/reference_point.hpp>
#include <planner_core/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace planner::node
{

using ReferencePoint  = interface::msg::ReferencePoint;
using ReferenceHorizon = interface::msg::ReferenceHorizon;

/**
 * @brief Reference buffer configuration.
 *
 * capacity == 1 reproduces the historic single-trajectory behaviour
 * (replanning disabled). capacity >= 2 keeps the previous segment(s) alive
 * so a replanned trajectory can be committed at a segment boundary.
 *
 * The horizon convention (horizon_s, traj_res_s, ctrl_frq) is owned by nmpc
 * (see nmpc.yaml); the planner just builds the reference on that grid.
 */
struct ReferenceBufferConfig
{
  std::size_t capacity   = 1;
  double      horizon_s  = 1.0;   ///< NMPC horizon length [s]
  double      traj_res_s = 0.1;   ///< NMPC horizon node spacing [s]
  double      ctrl_frq   = 50.0;  ///< NMPC solve / request frequency [Hz]
  bool        circled    = false;
};

/**
 * @brief ROS-side reference buffer.
 *
 * Holds the continuous planner_core::ReferenceTrajectory and answers each
 * reference request with exactly N+1 points evaluated at
 *   t0 + k * traj_res_s,  k = 0..N,  N = round(horizon_s / traj_res_s)
 * (no downsampling, no interpolation). The playhead advances by 1/ctrl_frq
 * per request. Eviction is in-order retire, not LRU.
 */
class ReferenceBuffer
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
   * @param config Buffer + NMPC horizon convention
   */
  explicit ReferenceBuffer(rclcpp::Node& node, ReferenceBufferConfig config = {});

  // Disable copy and move
  ReferenceBuffer(const ReferenceBuffer&)            = delete;
  ReferenceBuffer& operator=(const ReferenceBuffer&) = delete;
  ReferenceBuffer(ReferenceBuffer&&)                 = delete;
  ReferenceBuffer& operator=(ReferenceBuffer&&)      = delete;

  /**
   * @brief Replan entry point: enqueue a newly planned reference trajectory.
   *
   * @param traj Continuous trajectory produced by planner_core::Planner
   */
  void push(const core::ReferenceTrajectory& traj);

  /// Drop all buffered segments and fall back to the fixed hold point.
  void clear();

  /// Toggle between PLANNING (follow buffer) and FIXED (hold) reference.
  void toggleMode();

 private:
  /// A buffered trajectory plus its time playhead.
  struct Segment
  {
    core::ReferenceTrajectory traj;
    double                    playhead = 0.0;  ///< current reference time [s]
  };

  void initInterfaces();
  void onReferenceRequest(const std_msgs::msg::Bool::SharedPtr msg);
  void onMode(const std_msgs::msg::Bool::SharedPtr msg);

  /// N+1 = round(horizon_s / traj_res_s) + 1
  std::size_t horizon_points() const;
  void        to_point(const core::ReferencePoint& rp, ReferencePoint& pt) const;
  void        build_fixed_horizon(ReferenceHorizon& msg) const;
  void        terminate(Segment& segment);

  rclcpp::Node&         node_;
  ReferenceBufferConfig config_;

  /// Time-ordered segments; front is the segment currently being streamed.
  std::deque<Segment> segments_;

  RefTrajType     traj_type_     = RefTrajType::FIXED;
  bool            init_traj_set_ = false;
  Eigen::Vector3d origin_        = Eigen::Vector3d::Zero();
  core::ReferencePoint fixed_point_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reference_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_trigger_sub_;
  rclcpp::Publisher<ReferenceHorizon>::SharedPtr       reference_pub_;

  const rclcpp::QoS reliabe_qos_keep_last =
      rclcpp::QoS(10).reliable().keep_last(1);
};

}  // namespace planner::node
