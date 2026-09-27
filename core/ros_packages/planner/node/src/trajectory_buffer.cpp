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

#include <array>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <planner_node/trajectory_buffer.hpp>

namespace planner::node
{

  namespace
  {
    constexpr int    kRefTrajBulletSize = TrajecotryBullet::SIZE;
    constexpr double kThrustPerRotor    = 9.8 * 1.0 / 4;
  }  // namespace

  TrajectoryBuffer::TrajectoryBuffer(rclcpp::Node &node,
                                     TrajectoryBufferConfig config)
      : node_(node), config_(config)
  {
    initInterfaces();
  }

  void TrajectoryBuffer::initInterfaces()
  {
    ref_can_set_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        "ref_can_set", reliabe_qos_keep_last,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onRefCanSet(msg); });
    mode_trigger_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        "mode", reliabe_qos_keep_last,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onMode(msg); });

    state_sub_ = node_.create_subscription<interface::msg::State>(
        "cur_state", reliabe_qos_keep_last,
        [this](const interface::msg::State::SharedPtr msg) { onState(msg); });

    traj_bullet_pub_ = node_.create_publisher<TrajecotryBullet>(
        "ref_trajectory_bullet", reliabe_qos_keep_last);
  }

  Trajectory TrajectoryBuffer::to_msg(const core::Trajectory &traj)
  {
    Trajectory msg;
    msg.points.reserve(traj.samples.size());
    for (const auto &s : traj.samples)
    {
      TrajecotryPoint point;
      for (int i = 0; i < 3; i++)
      {
        point.point[TrajecotryPoint::POS_START_INDEX + i]  = s.p[i];
        point.point[TrajecotryPoint::VEL_START_INDEX + i]  = s.v[i];
        point.point[TrajecotryPoint::RATE_START_INDEX + i] = s.omega[i];
        point.point[TrajecotryPoint::YB_START_INDEX + i]   = s.yb[i];
        point.point[TrajecotryPoint::THR_START_INDEX + i]  = kThrustPerRotor;
      }
      point.point[TrajecotryPoint::THR_START_INDEX + 3] = kThrustPerRotor;
      msg.points.push_back(point);
    }
    return msg;
  }

  void TrajectoryBuffer::push(const core::Trajectory &traj)
  {
    if (traj.samples.empty())
    {
      RCLCPP_WARN(node_.get_logger(), "Received empty trajectory, ignoring");
      return;
    }

    Segment segment;
    segment.traj = to_msg(traj);

    if (!init_traj_set_)
    {
      const auto &p0 = traj.samples.front().p;
      const auto &y0 = traj.samples.front().yb;
      init_pos_      = p0;
      init_yb_       = y0;

      // Seed the fixed hold point from the trajectory start.
      for (auto &point : msg_fixed_point_.points)
      {
        for (int i = 0; i < 3; i++)
        {
          point.point[TrajecotryPoint::POS_START_INDEX + i] = init_pos_[i];
          point.point[TrajecotryPoint::YB_START_INDEX + i]  = init_yb_[i];
        }
      }
      init_traj_set_ = true;
    }

    // Rebase positions to the planner origin.
    for (auto &point : segment.traj.points)
    {
      for (int i = 0; i < 3; i++)
      {
        point.point[TrajecotryPoint::POS_START_INDEX + i] -= init_pos_[i];
      }
    }

    // In-order retire: keep at most capacity segments, oldest first.
    while (segments_.size() >= config_.capacity)
    {
      segments_.pop_front();
    }
    segments_.push_back(std::move(segment));

    RCLCPP_WARN(node_.get_logger(), "Trajectory points size %zu",
                segments_.back().traj.points.size());
    RCLCPP_WARN(node_.get_logger(),
                "Trajectory is received, waiitng for /ref_can_set to be true");
  }

  void TrajectoryBuffer::clear()
  {
    segments_.clear();
    init_traj_set_ = false;
    traj_type_     = RefTrajType::FIXED;
  }

  void TrajectoryBuffer::toggleMode()
  {
    if (traj_type_ == RefTrajType::FIXED)
    {
      traj_type_ = RefTrajType::PLANNING;
      for (auto &segment : segments_)
      {
        segment.cursor = 0;
      }
      RCLCPP_WARN(node_.get_logger(), "Switch to Planning Trajectory");
    }
    else
    {
      traj_type_ = RefTrajType::FIXED;
      RCLCPP_WARN(node_.get_logger(), "Switch to Fixed Point Trajectory");
    }
  }

  void TrajectoryBuffer::onMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      toggleMode();
    }
  }

  bool TrajectoryBuffer::checkTrajectoryTermination(Segment &segment)
  {
    bool ifTerminated = (segment.cursor == segment.traj.points.size());
    if (ifTerminated)
    {
      RCLCPP_WARN(node_.get_logger(), "Trajectory is terminated");
      auto traj_last_point = segment.traj.points.back();
      for (auto &point : msg_fixed_point_.points)
      {
        for (int i = 0; i < 3; i++)
        {
          point.point[TrajecotryPoint::POS_START_INDEX + i] =
              traj_last_point.point[TrajecotryPoint::POS_START_INDEX + i];
          point.point[TrajecotryPoint::VEL_START_INDEX + i]  = 0;
          point.point[TrajecotryPoint::RATE_START_INDEX + i] = 0;
          point.point[TrajecotryPoint::YB_START_INDEX + i] =
              traj_last_point.point[TrajecotryPoint::YB_START_INDEX + i];
        }
      }

      if (config_.circled)
      {
        traj_type_ = RefTrajType::PLANNING;
        for (auto &s : segments_)
        {
          s.cursor = 0;
        }
      }
      else
      {
        traj_type_ = RefTrajType::FIXED;
      }
    }
    return ifTerminated;
  }

  // state callback is to update cz_predict
  void TrajectoryBuffer::onState(const interface::msg::State::SharedPtr msg)
  {
    auto state = msg->state;  // p,v,r,q
    auto v     = Eigen::Vector3d(state[3], state[4], state[5]);
    auto q     = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
    Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;
    (void)vb;
  }

  void TrajectoryBuffer::onRefCanSet(const std_msgs::msg::Bool::SharedPtr msg)
  {
    static bool first_time = true;

    if (!msg->data || segments_.empty())
    {
      return;
    }

    if (traj_type_ == RefTrajType::FIXED)
    {
      if (first_time)
      {
        RCLCPP_WARN(node_.get_logger(), "Fixed Point Publish");
        first_time = false;
      }
      msg_fixed_point_.tracking_valid = false;
      msg_fixed_point_.timestamp      = node_.now();

      traj_bullet_pub_->publish(msg_fixed_point_);
      return;
    }

    // traj_type_ == RefTrajType::PLANNING
    auto &segment = segments_.front();
    if (checkTrajectoryTermination(segment))
    {
      return;
    }
    msg_planning_traj_.tracking_valid = true;

    for (int k = 0; k < kRefTrajBulletSize; k++)
    {
      const std::size_t idx =
          segment.cursor +
          static_cast<std::size_t>(config_.down_sampling_factor) * k;
      if (idx < segment.traj.points.size())
      {
        msg_planning_traj_.points[k] = segment.traj.points[idx];
      }
      else
      {
        msg_planning_traj_.points[k] = segment.traj.points.back();
      }
    }

    msg_planning_traj_.timestamp = node_.now();
    segment.cursor += 1;
    traj_bullet_pub_->publish(msg_planning_traj_);
  }

}  // namespace planner::node
