#ifndef __INPUT_H
#define __INPUT_H

#include <uav_utils/utils.h>

#include <Eigen/Dense>
#include <interface/msg/control.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/throttle_model_status.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "PX4CtrlParam.h"

class RcData
{
 public:
  double mode;
  double gear;
  double last_mode;
  double last_gear;
  bool   valid;
  bool   kill_switch_engaged;
  bool   have_init_last_mode{false};
  bool   have_init_last_gear{false};
  double ch[4];

  mavros_msgs::msg::RCIn msg;
  rclcpp::Time           rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;

  static constexpr double GEAR_SHIFT_VALUE            = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE    = 0.75;
  static constexpr double KILL_SWITCH_THRESHOLD_VALUE = 0.75;
  static constexpr double DEAD_ZONE                   = 0.25;
  static constexpr int    MODE_CHANNEL                = 4;
  static constexpr int    GEAR_CHANNEL                = 5;
  static constexpr int    KILL_SWITCH_CHANNEL         = 6;
  static constexpr int    MIN_CHANNELS                = 7;
  static constexpr int    RC_PWM_MIN                  = 800;
  static constexpr int    RC_PWM_MAX                  = 2200;

  RcData();
  bool check_validity();
  bool check_centered();
  void feed(mavros_msgs::msg::RCIn::ConstPtr pMsg);
  bool is_received(const rclcpp::Time &now_time);
};

class OdomData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d    p;
  Eigen::Vector3d    v;
  Eigen::Quaterniond q;
  Eigen::Vector3d    w;

  nav_msgs::msg::Odometry msg;
  rclcpp::Time            rcv_stamp;
  bool                    recv_new_msg;

  OdomData();
  /**
   * Cache one odometry sample.
   *
   * @param[in] pMsg Odometry message [-]
   * @param[in] twist_is_body True when twist is body-frame and needs the
   *            body->world rotation (mavros local_position in sim) [-]
   */
  void feed(nav_msgs::msg::Odometry::ConstPtr pMsg, bool twist_is_body = false);
};

class ImuData
{
 public:
  Eigen::Quaterniond q;
  Eigen::Vector3d    w;
  Eigen::Vector3d    a;

  sensor_msgs::msg::Imu msg;
  rclcpp::Time          rcv_stamp;

  ImuData();
  void feed(sensor_msgs::msg::Imu::ConstPtr pMsg);
};

class StateData
{
 public:
  mavros_msgs::msg::State current_state;
  mavros_msgs::msg::State state_before_offboard;

  StateData();
  void feed(mavros_msgs::msg::State::ConstPtr pMsg);
};

class ExtendedStateData
{
 public:
  mavros_msgs::msg::ExtendedState current_extended_state;

  ExtendedStateData();
  void feed(mavros_msgs::msg::ExtendedState::ConstPtr pMsg);
};

class CommandData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double          yaw;
  double          yaw_rate;
  uint8_t         yaw_control_mode;

  quadrotor_msgs::msg::PositionCommand msg;
  rclcpp::Time                         rcv_stamp;

  CommandData();
  void feed(quadrotor_msgs::msg::PositionCommand::ConstPtr pMsg);
};

class BatteryData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::msg::BatteryState msg;
  rclcpp::Time                   rcv_stamp;

  BatteryData();
  void feed(sensor_msgs::msg::BatteryState::ConstPtr pMsg);
};

class TakeoffLandData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool    triggered{false};
  uint8_t takeoff_land_cmd;  // see TakeoffLand.msg for its definition

  quadrotor_msgs::msg::TakeoffLand msg;
  rclcpp::Time                     rcv_stamp;

  TakeoffLandData();
  void feed(quadrotor_msgs::msg::TakeoffLand::ConstPtr pMsg);
};

/**
 * Latest NMPC control packet: source of the internal nmpc controller mode
 * (ctrl_mode=1) and of PASS_THROUGH when it is selected.
 */
class ControlsData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  interface::msg::Control msg;
  rclcpp::Time             rcv_stamp{0, 0, RCL_SYSTEM_TIME};
  bool                     msg_received{false};

  /**
   * Cache the newest NMPC control packet and stamp its arrival.
   *
   * @param[in] pMsg Controls (FLU rates_sp + specific_force_sp) [-]
   */
  void feed(interface::msg::Control::ConstPtr pMsg)
  {
    msg          = *pMsg;
    rcv_stamp    = rclcpp::Clock().now();
    msg_received = true;
  }

  /**
   * Check the controls stream against the pass-through timeout.
   *
   * @param[in] now_time Current FSM time [s]
   * @param[in] timeout_s Staleness threshold [s]
   * @return True when the latest packet is fresh enough [-]
   */
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const
  {
    return msg_received && (now_time - rcv_stamp).seconds() < timeout_s;
  }
};

/**
 * Latest external body-rate + thrust setpoint for the PASS_THROUGH state
 * (e.g. an external rates bridge running in the nav_infra container).
 */
class PassThroughData
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mavros_msgs::msg::AttitudeTarget msg;
  rclcpp::Time                     rcv_stamp{0, 0, RCL_SYSTEM_TIME};
  bool                             msg_received{false};

  /**
   * Cache the newest external setpoint and stamp its arrival.
   *
   * @param[in] pMsg External AttitudeTarget (FLU rates + thrust) [-]
   */
  void feed(mavros_msgs::msg::AttitudeTarget::ConstPtr pMsg)
  {
    msg          = *pMsg;
    rcv_stamp    = rclcpp::Clock().now();
    msg_received = true;
  }

  /**
   * Check the external stream against the pass-through timeout.
   *
   * @param[in] now_time Current FSM time [s]
   * @return True when the latest setpoint is fresh enough [-]
   */
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const
  {
    return msg_received && (now_time - rcv_stamp).seconds() < timeout_s;
  }
};

#endif
