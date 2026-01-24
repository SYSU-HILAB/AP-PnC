
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

#include <array>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <interface/msg/detail/trajectory_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_buffer/traj_buffer.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

void push_to_vis(visualization_msgs::msg::MarkerArray &vis, Trajectory &traj);

TrajectoryBuffer::TrajectoryBuffer() : Node("trajectory_buffer")
{
  initSubsPubs();
  declareParameters();
  updateParameters();
}

bool TrajectoryBuffer::checkTrajectoryTermination()
{
  bool ifTerminated = (index_ == trajectory_.points.size()) ? true : false;
  if (ifTerminated)
  {
    RCLCPP_WARN(this->get_logger(), "Trajectory is terminated");
    auto traj_last_point = trajectory_.points.back();
    for (auto &point : msg_fixed_point_.points)
    {
      for (int i = 0; i < 3; i++)
      {
        point.point[TrajecotryPoint::POS_START_INDEX + i] =
            traj_last_point.point[TrajecotryPoint::POS_START_INDEX + i];
        point.point[TrajecotryPoint::VEL_START_INDEX + i]  = 0;
        point.point[TrajecotryPoint::RATE_START_INDEX + i] = 0;
      }
      point.point[TrajecotryPoint::YB_START_INDEX] =
          traj_last_point.point[TrajecotryPoint::YB_START_INDEX];
      point.point[TrajecotryPoint::YB_START_INDEX + 1] =
          traj_last_point.point[TrajecotryPoint::YB_START_INDEX + 1];
      point.point[TrajecotryPoint::YB_START_INDEX] =
          traj_last_point.point[TrajecotryPoint::YB_START_INDEX + 2];
    }

    if (circled_)
    {
      traj_type_ = RefTrajType::PLANNING;
      index_     = 0;
    }
    else
    {
      traj_type_ = RefTrajType::FIXED;
    }
  }
  return ifTerminated;
}
// state callback is to update cz_predict
void TrajectoryBuffer::stateCallback(const interface::msg::State::SharedPtr msg)
{
  auto state = msg->state;  // p,v,r,q
  // get v and q to eigen
  auto v = Eigen::Vector3d(state[3], state[4], state[5]);
  auto q = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
  Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;
};

void TrajectoryBuffer::refTrajectoryBulletPublish(
    std_msgs::msg::Bool::SharedPtr msg)
{
  static bool first_time = true;

  if (msg->data)
  {
    if (traj_type_ == RefTrajType::FIXED)
    {
      if (first_time)
      {
        RCLCPP_WARN(this->get_logger(), "Fixed Point Publish");
        first_time = false;
      }
      tracking_valid_                 = false;
      msg_fixed_point_.tracking_valid = tracking_valid_;
      msg_fixed_point_.timestamp      = this->now();

      auto loaned_msg = traj_bullet_pub_->borrow_loaned_message();
      loaned_msg.get() = msg_fixed_point_;
      traj_bullet_pub_->publish(std::move(loaned_msg));
    }

    // traj_type_ == RefTrajType::PLANNING
    else
    {
      RCLCPP_INFO(this->get_logger(), "refTrajectoryBulletPublish");
      RCLCPP_INFO(this->get_logger(), "index_ %d", static_cast<int>(index_));
      if (checkTrajectoryTermination())
      {
        return;
      }
      tracking_valid_                   = true;
      msg_planning_traj_.tracking_valid = tracking_valid_;

      // Leaving all cz prediction to a seperate function
      for (size_t k = 0; k < kRefTrajBulletSize; k++)
      {
        if (index_ + down_sampling_factor * k < trajectory_.points.size())
        {
          msg_planning_traj_.points[k] =
              trajectory_.points[index_ + down_sampling_factor * k];
        }
        else
        {
          msg_planning_traj_.points[k] = trajectory_.points.back();
        }
      }

      auto v1 = Eigen::Vector3d(
          msg_planning_traj_.points[0]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX],
          msg_planning_traj_.points[0]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX + 1],
          msg_planning_traj_.points[0]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX + 2]);
      auto v2 = Eigen::Vector3d(
          msg_planning_traj_.points[1]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX],
          msg_planning_traj_.points[1]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX + 1],
          msg_planning_traj_.points[1]
              .point[interface::msg::TrajectoryPoint::VEL_START_INDEX + 2]);

      msg_planning_traj_.timestamp = this->now();
      index_ += 1;
      auto loaned_msg = traj_bullet_pub_->borrow_loaned_message();
      loaned_msg.get() = msg_planning_traj_;
      traj_bullet_pub_->publish(std::move(loaned_msg));
    }
  }
};

void TrajectoryBuffer::updateTrajCallback(const Trajectory::SharedPtr msg)
{
  static std::array<double, 3> init_pos = {0.0, 0.0, 0.0};
  static std::array<double, 3> init_yb  = {0.0, 1.0, 0.0};
  std::cerr << "updateTrajCallback" << std::endl;
  if (init_traj_set_ == false)
  {
    trajectory_.points.clear();
    // copy msg to trajectory_
    trajectory_.points = msg->points;
    init_pos[0]        = trajectory_.points[0]
                      .point[interface::msg::TrajectoryPoint::POS_START_INDEX];
    init_pos[1] =
        trajectory_.points[0]
            .point[interface::msg::TrajectoryPoint::POS_START_INDEX + 1];

    // change ref_thrust to gravity force.
    for (auto &point : trajectory_.points)
    {
      point.point[TrajecotryPoint::POS_START_INDEX] -= init_pos[0];
      point.point[TrajecotryPoint::POS_START_INDEX + 1] -= init_pos[1];
      point.point[TrajecotryPoint::POS_START_INDEX + 2] -= init_pos[2];
      point.point[TrajecotryPoint::THR_START_INDEX]     = kThrustPerRotor;
      point.point[TrajecotryPoint::THR_START_INDEX + 1] = kThrustPerRotor;
      point.point[TrajecotryPoint::THR_START_INDEX + 2] = kThrustPerRotor;
      point.point[TrajecotryPoint::THR_START_INDEX + 3] = kThrustPerRotor;
    }
    for (int i = 0; i < 3; i++)
    {
      init_yb[i] =
          trajectory_.points[0]
              .point[interface::msg::TrajectoryPoint::YB_START_INDEX + i];
    }
    for (auto &point : msg_fixed_point_.points)
    {
      point.point[TrajecotryPoint::POS_START_INDEX] =
          trajectory_.points[0].point[TrajecotryPoint::POS_START_INDEX];
      point.point[TrajecotryPoint::POS_START_INDEX + 1] =
          trajectory_.points[0].point[TrajecotryPoint::POS_START_INDEX + 1];
      point.point[TrajecotryPoint::POS_START_INDEX + 2] =
          trajectory_.points[0].point[TrajecotryPoint::POS_START_INDEX + 2];
      point.point[TrajecotryPoint::YB_START_INDEX]     = init_yb[0];
      point.point[TrajecotryPoint::YB_START_INDEX + 1] = init_yb[1];
      point.point[TrajecotryPoint::YB_START_INDEX + 2] = init_yb[2];
    }

    init_traj_set_ = true;
  }

  static auto vis_msg = visualization_msgs::msg::MarkerArray();
  push_to_vis(vis_msg, trajectory_);
  ref_vis_pub_->publish(vis_msg);

  RCLCPP_WARN(this->get_logger(), "Trajectory points size %zu",
              trajectory_.points.size());
  RCLCPP_WARN(this->get_logger(),
              "Trajectory is received, waiitng for /ref_can_set to be true");
};

void TrajectoryBuffer::declareParameters()
{
  this->declare_parameter<double>("traj_control_time", traj_control_time_);
  this->declare_parameter<int>("down_sampling_factor", down_sampling_factor);
  this->declare_parameter<bool>("circled", circled_);
};

void TrajectoryBuffer::updateParameters()
{
  this->get_parameter("traj_sampling_time", traj_sampling_time_);
  this->get_parameter("traj_control_time", traj_control_time_);
  this->get_parameter("down_sampling_factor", down_sampling_factor);
  this->get_parameter("circled", circled_);
}

void TrajectoryBuffer::modeTriggerCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
  {
    if (traj_type_ == RefTrajType::FIXED)
    {
      traj_type_ = RefTrajType::PLANNING;
      index_     = 0;
      RCLCPP_WARN(this->get_logger(), "Switch to Planning Trajectory");
    }
    else
    {
      traj_type_ = RefTrajType::FIXED;
      RCLCPP_WARN(this->get_logger(), "Switch to Fixed Point Trajectory");
    }
  }
};

void TrajectoryBuffer::initSubsPubs()
{
  ref_can_set_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "ref_can_set", reliabe_qos_keep_last,
      std::bind(&TrajectoryBuffer::refTrajectoryBulletPublish, this,
                std::placeholders::_1));
  mode_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "mode", reliabe_qos_keep_last,
      std::bind(&TrajectoryBuffer::modeTriggerCallback, this,
                std::placeholders::_1));
  traj_sub_ = this->create_subscription<Trajectory>(
      "control_traj", reliabe_qos_keep_last,
      std::bind(&TrajectoryBuffer::updateTrajCallback, this,
                std::placeholders::_1));

  state_sub_ = this->create_subscription<interface::msg::State>(
      "cur_state", reliabe_qos_keep_last,
      std::bind(&TrajectoryBuffer::stateCallback, this, std::placeholders::_1));

  ref_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ref_vis", reliabe_qos_keep_last);

  traj_bullet_pub_ = this->create_publisher<TrajecotryBullet>(
      "ref_trajectory_bullet", reliabe_qos_keep_last);
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryBuffer>());
  rclcpp::shutdown();
  return 0;
}

void push_to_vis(visualization_msgs::msg::MarkerArray &vis, Trajectory &traj)
{
  auto marker            = visualization_msgs::msg::Marker();
  marker.header.frame_id = "world";
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.type            = visualization_msgs::msg::Marker::ARROW;
  marker.scale.x         = 0.1;
  marker.scale.y         = 0.01;
  marker.scale.z         = 0.01;
  marker.color.a         = 0.2;
  marker.color.r         = 0.0;
  marker.color.g         = 0.0;
  marker.color.b         = 1.0;
  int index              = 0;
  for (auto point : traj.points)
  {
    marker.pose.position.x = point.point[TrajecotryPoint::POS_START_INDEX];
    marker.pose.position.y = point.point[TrajecotryPoint::POS_START_INDEX + 1];
    marker.pose.position.z = point.point[TrajecotryPoint::POS_START_INDEX + 2];
    vis.markers.push_back(marker);
    marker.id = index++;
    marker.ns = "ref_vis";
    auto yb   = Eigen::Vector3d();
    yb(0)     = point.point[TrajecotryPoint::YB_START_INDEX];
    yb(1)     = point.point[TrajecotryPoint::YB_START_INDEX + 1];
    yb(2)     = point.point[TrajecotryPoint::YB_START_INDEX + 2];

    auto rot_axis  = Eigen::Vector3d::UnitX().cross(yb);
    auto rot_angle = std::acos(Eigen::Vector3d::UnitX().dot(yb));
    // from rot axis and angle to construct quaternion
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(rot_angle, rot_axis.normalized());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    vis.markers.push_back(marker);
  }
}
