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

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <interface/msg/controls.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/tracking_info.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

#include "nmpc_solver.h"

using StateMsg        = interface::msg::State;
using ReferenceBullet = interface::msg::RefTrajectoryBullet;
using ControlMsg      = interface::msg::Controls;
using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrackingInfoMsg = interface::msg::TrackingInfo;
using AccelerationMsg = sensor_msgs::msg::Imu;

class NmpcCallbacks
{
 public:
  virtual void stateCallback(const StateMsg &msg)            = 0;
  virtual void referenceCallback(const ReferenceBullet &msg) = 0;
  virtual void accCallback(const AccelerationMsg &msg)       = 0;
  virtual void initSubsPubs()                                = 0;

 public:
  rclcpp::Subscription<StateMsg>::SharedPtr        state_sub_;
  rclcpp::Subscription<ReferenceBullet>::SharedPtr ref_sub_;
  rclcpp::Subscription<AccelerationMsg>::SharedPtr acc_sub_;

  rclcpp::Publisher<ControlMsg>::SharedPtr          ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<TrackingInfoMsg>::SharedPtr     tracking_info_pub_;
  const rclcpp::QoS                                 reliabe_qos_keep_last =
      rclcpp::QoS(10)
          .reliable()
          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
          .keep_last(1);
};

class NMPC : public rclcpp::Node, public NmpcCallbacks
{
 private:
  void initSubsPubs() override;
  void stateCallback(const StateMsg &msg) override;
  void referenceCallback(const ReferenceBullet &msg) override;
  void accCallback(const AccelerationMsg &msg) override;
  void solve();

  nmpc_solver_t *nmpc_;

  double mass_;
  double cz_rate_;

 public:
  NMPC();
  ~NMPC();

 private:
  std::vector<double>                        time_statistic;
  std::chrono::_V2::system_clock::time_point t_start_;
  std::chrono::_V2::system_clock::time_point t_end_;

  double               cur_vel_norm;
  constexpr static int state_size       = 13;
  constexpr static int controls_size    = 4;
  constexpr static int pos_start_index  = 0;
  constexpr static int vel_start_index  = 3;
  constexpr static int rate_start_index = 6;
  constexpr static int q_start_index    = 9;

  std::array<double, NMPC_NX>       x_cur_;
  std::array<double, controls_size> u_;
  std::array<double, controls_size> u_sol;
  std::array<double, NMPC_NX>       x_next;
  double                            cz_filtered_ = 0.0;
};

void NMPC::initSubsPubs()
{
  state_sub_ = this->create_subscription<StateMsg>(
      "/cur_state", reliabe_qos_keep_last,
      std::bind(&NMPC::stateCallback, this, std::placeholders::_1));

  ref_sub_ = this->create_subscription<ReferenceBullet>(
      "/ref_trajectory_bullet", reliabe_qos_keep_last,
      std::bind(&NMPC::referenceCallback, this, std::placeholders::_1));

  acc_sub_ = this->create_subscription<AccelerationMsg>(
      "/mavros/imu/data_raw", rclcpp::SensorDataQoS(),
      [this](const AccelerationMsg &msg) { accCallback(msg); });

  ctrl_pub_ = this->create_publisher<ControlMsg>("/mpc_controls",
                                                 reliabe_qos_keep_last);

  ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/ref_can_set", reliabe_qos_keep_last);

  tracking_info_pub_ = this->create_publisher<TrackingInfoMsg>(
      "/tracking_info", reliabe_qos_keep_last);
}

void NMPC::solve()
{
  int status = nmpc_solver_solve(nmpc_, u_sol.data(), x_next.data());

  switch (status)
  {
    case 0:
    case 2:
      break;
    default:
      RCLCPP_FATAL(this->get_logger(),
                   "nmpc_solver_solve returned %d. Exiting.\n", status);
      exit(1);
  }

  Eigen::Quaterniond q_next(x_next[q_start_index], x_next[q_start_index + 1],
                            x_next[q_start_index + 2],
                            x_next[q_start_index + 3]);
  q_next.normalize();

  double pitch = q_next.toRotationMatrix().eulerAngles(2, 1, 0)(1);
  if (pitch >= M_PI_2)
    pitch -= M_PI;
  if (pitch < -M_PI_2)
    pitch += M_PI;

  std::copy(x_next.data() + state_size,
            x_next.data() + state_size + controls_size, u_.begin());
  ControlMsg ctrl_msg;
  ctrl_msg.motors_u  = {u_[0], u_[1], u_[2], u_[3]};
  ctrl_msg.rates_sp  = {x_next[rate_start_index], x_next[rate_start_index + 1],
                        x_next[rate_start_index + 2]};
  ctrl_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  ctrl_msg.pitch_sp  = pitch;
  ctrl_msg.thrust_acc_sp = (u_[0] + u_[1] + u_[2] + u_[3]) / mass_;
  t_end_                 = std::chrono::high_resolution_clock::now();

  ctrl_msg.sol_time =
      std::chrono::duration<double, std::milli>(t_end_ - t_start_).count();
  ctrl_pub_->publish(ctrl_msg);
}

void NMPC::accCallback(const AccelerationMsg &msg)
{
  // mavros HIGHRES_IMU is FLU (body x forward); the legacy px4_msgs input
  // was FRD, hence the flipped sign of the drag-axis acceleration.
  cz_filtered_ = msg.linear_acceleration.x / (cur_vel_norm * cur_vel_norm + 0.1) * 1.0;
  if (cz_filtered_ > 0.4)
    cz_filtered_ = 0.4;
  if (cz_filtered_ < -0.4)
    cz_filtered_ = -0.4;
}

void NMPC::referenceCallback(const ReferenceBullet &msg)
{
  auto           &refs           = const_cast<ReferenceBullet &>(msg);
  bool            tracking_valid = refs.tracking_valid;
  TrackingInfoMsg tracking_info_msg;
  tracking_info_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  tracking_info_msg.ref_position = {
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::POS_START_INDEX]),
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::POS_START_INDEX + 1]),
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::POS_START_INDEX + 2])};
  tracking_info_msg.ref_velocity = {
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::VEL_START_INDEX]),
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::VEL_START_INDEX + 1]),
      static_cast<float>(
          refs.points[0].point[TrajecotryPoint::VEL_START_INDEX + 2])};
  tracking_info_msg.actual_position = {
      static_cast<float>(x_cur_[pos_start_index]),
      static_cast<float>(x_cur_[pos_start_index + 1]),
      static_cast<float>(x_cur_[pos_start_index + 2])};
  tracking_info_msg.actual_velocity = {
      static_cast<float>(x_cur_[vel_start_index]),
      static_cast<float>(x_cur_[vel_start_index + 1]),
      static_cast<float>(x_cur_[vel_start_index + 2])};
  tracking_info_msg.actual_vxy = tracking_info_msg.actual_velocity[0] *
                                     tracking_info_msg.actual_velocity[0] +
                                 tracking_info_msg.actual_velocity[1] *
                                     tracking_info_msg.actual_velocity[1];
  tracking_info_msg.ref_vxy =
      tracking_info_msg.ref_velocity[0] * tracking_info_msg.ref_velocity[0] +
      tracking_info_msg.ref_velocity[1] * tracking_info_msg.ref_velocity[1];
  tracking_info_msg.ref_yb = {
      static_cast<float>(msg.points[0].point[TrajecotryPoint::YB_START_INDEX]),
      static_cast<float>(
          msg.points[0].point[TrajecotryPoint::YB_START_INDEX + 1]),
      static_cast<float>(
          msg.points[0].point[TrajecotryPoint::YB_START_INDEX + 2])};
  Eigen::Quaterniond q_cur(x_cur_[q_start_index], x_cur_[q_start_index + 1],
                           x_cur_[q_start_index + 2],
                           x_cur_[q_start_index + 3]);
  q_cur.normalize();
  Eigen::Vector3d yb_double   = q_cur.toRotationMatrix().col(1);
  tracking_info_msg.actual_yb = {static_cast<float>(yb_double[0]),
                                 static_cast<float>(yb_double[1]),
                                 static_cast<float>(yb_double[2])};

  Eigen::Vector3d vb =
      q_cur.conjugate().toRotationMatrix() *
      Eigen::Vector3d(x_cur_[vel_start_index], x_cur_[vel_start_index + 1],
                      x_cur_[vel_start_index + 2]);

  tracking_info_msg.alpha = static_cast<float>(atan2(vb(0), vb(2)));
  tracking_info_msg.beta =
      static_cast<float>(atan2(vb(1), sqrt(vb(0) * vb(0) + vb(2) * vb(2))));
  tracking_info_msg.cz = static_cast<float>(cz_filtered_);
  if (tracking_valid)
    tracking_info_pub_->publish(tracking_info_msg);

  int    size = refs.points.size();
  double cz_rate;
  if (tracking_info_msg.alpha < M_PI_2 && tracking_info_msg.alpha > 0.0)
    cz_rate = cz_rate_;
  else if (tracking_info_msg.alpha >= M_PI_2 && tracking_info_msg.alpha < M_PI)
    cz_rate = -cz_rate_;
  else
    cz_rate = 0.0;

  double p[3] = {tracking_info_msg.cz, tracking_info_msg.alpha, cz_rate};
  int    N    = nmpc_solver_horizon(nmpc_);
  assert(size >= N + 1);

  for (int i = 0; i < N; i++)
  {
    nmpc_solver_set_ref_stage(nmpc_, i, refs.points[i].point.data(), p);
  }
  nmpc_solver_set_ref_terminal(nmpc_, refs.points[N].point.data());

  static bool first_solve = true;
  if (first_solve)
  {
    first_solve = false;
    RCLCPP_WARN(this->get_logger(), "First solve");
  }
  solve();
}

void NMPC::stateCallback(const StateMsg &msg)
{
  t_start_ = std::chrono::high_resolution_clock::now();
  std::copy(msg.state.data(), msg.state.data() + state_size, x_cur_.begin());
  std::copy(u_.data(), u_.data() + controls_size, x_cur_.begin() + state_size);
  x_cur_[state_size + controls_size] = cz_filtered_;
  cur_vel_norm =
      std::sqrt(x_cur_[vel_start_index] * x_cur_[vel_start_index] +
                x_cur_[vel_start_index + 1] * x_cur_[vel_start_index + 1] +
                x_cur_[vel_start_index + 2] * x_cur_[vel_start_index + 2]);
  nmpc_solver_set_state(nmpc_, x_cur_.data());

  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  ready_pub_->publish(ready_msg);
}

NMPC::NMPC() : Node("NMPC")
{
  // Config: direct yaml read (bringup), no ROS params.
  // Conventional cwd-relative path; containers run from /workspace.
  YAML::Node config = YAML::LoadFile("bringup/config/c_nmpc.yaml");
  mass_             = config["nmpc"]["mass"].as<double>(2.0);
  cz_rate_          = config["nmpc"]["cz_rate"].as<double>(0.10);
  RCLCPP_INFO(this->get_logger(), "mass: %f", mass_);
  initSubsPubs();
  u_.fill(9.8 / 4 * mass_);
  nmpc_ = nmpc_solver_create();
  if (!nmpc_)
  {
    RCLCPP_FATAL(this->get_logger(), "nmpc_solver_create failed");
    exit(1);
  }
}

NMPC::~NMPC()
{
  nmpc_solver_destroy(nmpc_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NMPC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
