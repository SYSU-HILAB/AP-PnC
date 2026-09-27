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

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <planner_node/reference_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner::node
{

  ReferenceBuffer::ReferenceBuffer(rclcpp::Node &node,
                                   ReferenceBufferConfig config)
      : node_(node), config_(config)
  {
    initInterfaces();
  }

  void ReferenceBuffer::initInterfaces()
  {
    reference_request_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        "/nmpc/reference_request", reliabe_qos_keep_last,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        { onReferenceRequest(msg); });
    mode_trigger_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        "/planner/mode", reliabe_qos_keep_last,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onMode(msg); });

    reference_pub_ = node_.create_publisher<ReferenceHorizon>(
        "/nmpc/reference", reliabe_qos_keep_last);
  }

  std::size_t ReferenceBuffer::horizon_points() const
  {
    const double n = std::round(config_.horizon_s / config_.traj_res_s);
    return static_cast<std::size_t>(std::max(1.0, n)) + 1;
  }

  void ReferenceBuffer::to_point(const core::ReferencePoint &rp,
                                 ReferencePoint &pt) const
  {
    for (int i = 0; i < 3; i++)
    {
      pt.pos[i] = rp.p[i] - origin_[i];
      pt.vel[i] = rp.v[i];
      pt.yb[i]  = rp.yb[i];
    }
  }

  void ReferenceBuffer::build_fixed_horizon(ReferenceHorizon &msg) const
  {
    const std::size_t n = horizon_points();
    msg.points.resize(n);
    for (auto &pt : msg.points)
    {
      to_point(fixed_point_, pt);
    }
  }

  void ReferenceBuffer::push(const core::ReferenceTrajectory &traj)
  {
    if (traj.empty())
    {
      RCLCPP_WARN(node_.get_logger(), "Received empty trajectory, ignoring");
      return;
    }

    if (!init_traj_set_)
    {
      fixed_point_   = traj.sample(0.0);
      origin_        = fixed_point_.p;
      init_traj_set_ = true;
    }

    Segment segment;
    segment.traj     = traj;
    segment.playhead = 0.0;

    // In-order retire: keep at most capacity segments, oldest first.
    while (segments_.size() >= config_.capacity)
    {
      segments_.pop_front();
    }
    segments_.push_back(std::move(segment));

    RCLCPP_WARN(node_.get_logger(),
                "Reference received (%.2f s, %zu points/request), waiitng for "
                "nmpc/reference_request",
                traj.duration(), horizon_points());
  }

  void ReferenceBuffer::clear()
  {
    segments_.clear();
    init_traj_set_ = false;
    traj_type_     = RefTrajType::FIXED;
  }

  void ReferenceBuffer::toggleMode()
  {
    if (traj_type_ == RefTrajType::FIXED)
    {
      traj_type_ = RefTrajType::PLANNING;
      for (auto &segment : segments_)
      {
        segment.playhead = 0.0;
      }
      RCLCPP_WARN(node_.get_logger(), "Switch to Planning Trajectory");
    }
    else
    {
      traj_type_ = RefTrajType::FIXED;
      RCLCPP_WARN(node_.get_logger(), "Switch to Fixed Point Trajectory");
    }
  }

  void ReferenceBuffer::onMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      toggleMode();
    }
  }

  void ReferenceBuffer::terminate(Segment &segment)
  {
    fixed_point_ = segment.traj.sample(segment.traj.duration());
    RCLCPP_WARN(node_.get_logger(), "Reference trajectory is terminated");

    if (config_.circled)
    {
      traj_type_       = RefTrajType::PLANNING;
      segment.playhead = 0.0;
    }
    else
    {
      traj_type_ = RefTrajType::FIXED;
    }
  }

  void ReferenceBuffer::onReferenceRequest(
      const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || segments_.empty())
    {
      return;
    }

    ReferenceHorizon out;
    out.timestamp       = node_.now().nanoseconds() / 1000;  // [us]
    out.traj_resolution = config_.traj_res_s;

    if (traj_type_ == RefTrajType::PLANNING)
    {
      Segment &segment = segments_.front();
      const double duration = segment.traj.duration();

      if (segment.playhead > duration)
      {
        terminate(segment);
      }
      else
      {
        const std::size_t n = horizon_points();
        out.tracking_valid  = true;
        out.points.resize(n);
        for (std::size_t k = 0; k < n; k++)
        {
          const double t =
              std::min(segment.playhead + static_cast<double>(k) *
                                             config_.traj_res_s,
                       duration);
          to_point(segment.traj.sample(t), out.points[k]);
        }
        segment.playhead += 1.0 / config_.ctrl_frq;
        reference_pub_->publish(out);
        return;
      }
    }

    // FIXED / terminated: hold the last reference point for the whole horizon.
    out.tracking_valid = false;
    build_fixed_horizon(out);
    reference_pub_->publish(out);
  }

}  // namespace planner::node
