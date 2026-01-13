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

#include <Eigen/Geometry>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <interface/msg/controls.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/tracking_info.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/tracking_info.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_acc_setpoint.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "helper.cpp"
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#include <stdexcept>
#include <vector>

static Eigen::Quaterniond q_flu_frd = Eigen::Quaterniond(0, 1, 0, 0);
class Transcribe
{
  Eigen::Quaterniond q_transform = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d    init_p;

 public:
  explicit Transcribe() { q_flu_frd.normalize(); };
  void setInitInertial(const Eigen::Vector3d    &pos,
                       const Eigen::Quaterniond &quat)
  {
    init_p      = pos;
    q_transform = quat * q_flu_frd;
    q_transform.normalize();
  }

  template <typename T>
  void toFLU(const std::array<T, 3> &pos, const std::array<T, 3> &vel,
             const std::array<T, 3> &rate, const std::array<T, 4> &quat,
             interface::msg::State &state)
  {
    Eigen::Vector3d    p{pos[0], pos[1], pos[2]};
    Eigen::Vector3d    v{vel[0], vel[1], vel[2]};
    Eigen::Vector3d    r{rate[0], rate[1], rate[2]};
    Eigen::Quaterniond q{quat[0], quat[1], quat[2], quat[3]};

    p -= init_p;
    p = q_transform * p;
    v = q_transform * v;
    r = q_flu_frd * r;
    q = q_transform * q * q_flu_frd;
    q.normalize();
    using interface::msg::State;
    std::copy(p.data(), p.data() + 3,
              state.state.begin() + State::POS_START_INDEX);
    std::copy(v.data(), v.data() + 3,
              state.state.begin() + State::VEL_START_INDEX);
    std::copy(r.data(), r.data() + 3,
              state.state.begin() + State::RATE_START_INDEX);

    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    std::copy(q_vec.data(), q_vec.data() + 4,
              state.state.begin() + State::QUAT_START_INDEX);
  }

  template <typename T>
  std::array<float, 3> &toFRD(const std::array<T, 3> &rate)
  {
    static std::array<float, 3> rate_frd;

    Eigen::Vector3d r{rate[0], rate[1], rate[2]};
    r = q_flu_frd * r;
    std::copy(r.data(), r.data() + 3, rate_frd.data());
    // q.coeffs
    return rate_frd;
  }
};

class PX4Interface : public rclcpp::Node
{
 public:
  PX4Interface() : Node("px4_interface")
  {
    trans_ = std::make_unique<Transcribe>();

    auto reliable_qos  = rclcpp::ParametersQoS().keep_last(1).transient_local();
    auto keep_last_qos = rclcpp::SensorDataQoS().keep_last(1);

    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
    px4_tracking_info_publisher_ =
        this->create_publisher<px4_msgs::msg::TrackingInfo>(
            "/fmu/in/tracking_info", 10);
    state_publisher_ = this->create_publisher<interface::msg::State>(
        "/cur_state", reliable_qos);
    vehicle_thrust_acc_setpoint_publisher_ =
        this->create_publisher<VehicleThrustAccSetpoint>(
            "/fmu/in/vehicle_thrust_acc_setpoint", 10);

    state_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/cur_state_odom", reliable_qos);
    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", keep_last_qos,
        std::bind(&PX4Interface::vehicle_odometry_callback, this,
                  std::placeholders::_1));
    controls_subscriber_ = this->create_subscription<interface::msg::Controls>(
        "/mpc_controls", rclcpp::SensorDataQoS(),
        std::bind(&PX4Interface::controls_callback, this,
                  std::placeholders::_1));
    tracking_info_subscriber_ =
        this->create_subscription<interface::msg::TrackingInfo>(
            "/tracking_info", keep_last_qos,
            std::bind(&PX4Interface::tracking_info_cb, this,
                      std::placeholders::_1));
    vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status", keep_last_qos,
        std::bind(&PX4Interface::init_status_check, this,
                  std::placeholders::_1));

    est_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/est_cmd", keep_last_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
        {
          if (msg->data)
          {
            is_takeoff_complete = true;
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Takeoff enter command received, please wait for "
                "takeoff complete");
          }
        });

    auto heartbeat_callback = [this]() -> void
    { publish_offboard_control_mode(); };
    heartbeat_timer_ = this->create_wall_timer(100ms, heartbeat_callback);
  }

 public:
  bool everything_is_ok      = false;
  bool is_init_check         = false;
  bool is_pos_orien_estimate = false;
  bool is_takeoff_complete   = false;

  std::unique_ptr<Transcribe> trans_;
  interface::msg::Controls    controls_;
  interface::msg::State       state_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;

  using MPC_STATE = interface::msg::State;

  static std::vector<Eigen::Vector3d>    pos_collecter_;
  static std::vector<Eigen::Quaterniond> quat_collector_;

  rclcpp::Publisher<VehicleThrustAccSetpoint>::SharedPtr
      vehicle_thrust_acc_setpoint_publisher_;
  rclcpp::Publisher<interface::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrackingInfo>::SharedPtr
      px4_tracking_info_publisher_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr
      offboard_control_mode_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_publisher_;

  rclcpp::Subscription<interface::msg::Controls>::SharedPtr
      controls_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      vehicle_odometry_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr est_cmd_subscriber_;
  rclcpp::Subscription<interface::msg::TrackingInfo>::SharedPtr
      tracking_info_subscriber_;

  void controls_callback(const interface::msg::Controls::SharedPtr msg);
  void vehicle_odometry_callback(
      const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void publish_offboard_control_mode();
  void publish_state();

  void tracking_info_cb(const interface::msg::TrackingInfo::SharedPtr msg);
  void init_status_check(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
};

void PX4Interface::init_status_check(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  // POS_CTRL == 2 in nav_state
  // ARM_STATE == 2 is armed in arming_state.

  static std::string WARN_START =
      "[PX4Interface:WARNING]===============================\n";

  static std::string WARN_END =
      "====================================================\n";

  if (is_init_check)
  {
    return;
  }

  bool is_armed =
      msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

  if (!is_armed)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       WARN_START +
                           "Vehicle is not armed! you can not start NMPC\n" +
                           WARN_END);
    return;
  }

  bool is_position_ctrl =
      msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;

  if (!is_position_ctrl)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       WARN_START +
                           "Vehicle is not in position control mode! you "
                           "can not start NMPC\n" +
                           WARN_END);
    return;
  }

  is_init_check = (is_armed && is_position_ctrl);
  if (is_init_check)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Vehicle is armed and in position control mode\n "
                       "Please give a estimation command");
    return;
  }
}

void PX4Interface::tracking_info_cb(
    const interface::msg::TrackingInfo::SharedPtr msg)
{
  static px4_msgs::msg::TrackingInfo tracking_info;
  tracking_info.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
  tracking_info.actual_position = msg->actual_position;
  tracking_info.actual_velocity = msg->actual_velocity;
  tracking_info.ref_position    = msg->ref_position;
  tracking_info.ref_velocity    = msg->ref_velocity;
  tracking_info.ref_yb          = msg->ref_yb;
  tracking_info.actual_yb       = msg->actual_yb;
  tracking_info.alpha           = msg->alpha;
  tracking_info.beta            = msg->beta;
  tracking_info.cz              = msg->cz;
  tracking_info.ref_vxy         = msg->ref_vxy;
  tracking_info.actual_vxy      = msg->actual_vxy;
  px4_tracking_info_publisher_->publish(tracking_info);
}

void PX4Interface::controls_callback(
    const interface::msg::Controls::SharedPtr msg)
{
  static px4_msgs::msg::VehicleThrustAccSetpoint px4_control;
  std::cout << "Controls received: " << msg->timestamp << std::endl;
  px4_control.timestamp     = this->get_clock()->now().nanoseconds() / 1000;
  px4_control.rates_sp      = trans_->toFRD(msg->rates_sp);
  px4_control.thrust_acc_sp = msg->thrust_acc_sp;
  px4_control.pitch_sp      = msg->pitch_sp;
  px4_control.sol_time      = msg->sol_time;

  if (everything_is_ok)
  {
    vehicle_thrust_acc_setpoint_publisher_->publish(px4_control);
  }
}

/**
 * @brief
 * Callback
 * on
 * vehicle_odometry
 */
void PX4Interface::vehicle_odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  static constexpr size_t   nums_to_to_collect = 100;
  static Eigen::Vector3d    pos_dec;
  static Eigen::Quaterniond quat_dec(Eigen::Quaterniond::Identity());
  static size_t             index = 0;

  if (!is_takeoff_complete || !is_init_check)
  {
    return;
  }

  if (index < nums_to_to_collect)
  {
    index++;
    Eigen::Vector3d    pos_temp(msg->position[0], msg->position[1],
                                msg->position[2]);
    Eigen::Quaterniond q_temp(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    // q_temp.normalize();
    q_temp = q_temp * q_flu_frd;
    pos_collecter_.push_back(pos_temp);
    quat_collector_.push_back(q_temp);
    return;
  }

  if (index == nums_to_to_collect)
  {
    index++;
    double yaw = YawCostFunctor::constructProblemAndSolve(quat_collector_);
    pos_dec    = pos_collecter_.back();
    std::cerr << "pos_dec: " << pos_dec.transpose() << std::endl;
    quat_dec =
        Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
    this->trans_->setInitInertial(pos_dec, quat_dec);
  }

  if (!is_pos_orien_estimate)
  {
    is_pos_orien_estimate = true;
    state_pub_timer_      = this->create_wall_timer(
        20ms, std::bind(&PX4Interface::publish_state, this));
    this->trans_->setInitInertial(pos_dec, quat_dec);
  }
  everything_is_ok = is_pos_orien_estimate && is_takeoff_complete;
  this->trans_->toFLU(msg->position, msg->velocity, msg->angular_velocity,
                      msg->q, state_);
}

void PX4Interface::publish_state()
{
  // TODO, 25.11.16, we should offload the cz estimation here, not in
  // c_nmpc.cpp
  state_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  // Add a logic that we should first Arm the vehicle and complte the position
  // and orientation estimate, and then we can publish the state otherwise,
  // the MPC should not start
  if (!everything_is_ok)
  {
    return;
  }
  state_publisher_->publish(state_);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = this->get_clock()->now();
  odom.header.frame_id = "world";
  using interface::msg::State;
  odom.pose.pose.position.x    = state_.state[State::POS_START_INDEX];
  odom.pose.pose.position.y    = state_.state[State::POS_START_INDEX + 1];
  odom.pose.pose.position.z    = state_.state[State::POS_START_INDEX + 2];
  odom.pose.pose.orientation.w = state_.state[State::QUAT_START_INDEX];
  odom.pose.pose.orientation.x = state_.state[State::QUAT_START_INDEX + 1];
  odom.pose.pose.orientation.y = state_.state[State::QUAT_START_INDEX + 2];
  odom.pose.pose.orientation.z = state_.state[State::QUAT_START_INDEX + 3];
  state_odom_publisher_->publish(odom);
}

void PX4Interface::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position     = false;
  msg.velocity     = false;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = true;
  msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}
std::vector<Eigen::Vector3d>    PX4Interface::pos_collecter_;
std::vector<Eigen::Quaterniond> PX4Interface::quat_collector_;

int main(int argc, char *argv[])
{
  std::cout << "Starting PX4 Interface Node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Interface>());

  rclcpp::shutdown();
  return 0;
}
