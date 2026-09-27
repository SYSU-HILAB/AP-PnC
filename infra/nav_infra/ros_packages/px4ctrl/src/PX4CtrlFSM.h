#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <cstdint>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "input.h"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand
{
  bool         landed{true};
  rclcpp::Time toggle_takeoff_land_time;
  rclcpp::Time not_armed_since;  // start of the continuous unarmed window
                                 // during AUTO_TAKEOFF
  std::pair<bool, rclcpp::Time> delay_trigger{std::pair<bool, rclcpp::Time>(
      false, rclcpp::Time(0, 0, RCL_SYSTEM_TIME))};
  Eigen::Vector4d               start_pose;

  static constexpr double MOTORS_SPEEDUP_TIME =
      3.0;  // motors idle running for 3 seconds before takeoff
  static constexpr double DELAY_TRIGGER_TIME =
      2.0;  // Time to be delayed when reach at target height
  static constexpr double NOT_ARMED_DEADZONE_TIME =
      3.0;  // unarmed dead-zone before aborting AUTO_TAKEOFF
};

class PX4CtrlFSM
{
 public:
  Parameters &param;

  RcData            rc_data;
  StateData         state_data;
  ExtendedStateData extended_state_data;
  OdomData          odom_data;
  ImuData           imu_data;
  CommandData       cmd_data;
  BatteryData       bat_data;
  TakeoffLandData   takeoff_land_data;
  ControlsData      controls_data;
  PassThroughData   pass_through_data;

  Controller &controller;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      traj_start_trigger_pub;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_FCU_pub;
  rclcpp::Publisher<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr
      debug_pub;  // debug

  // State adapter outputs (/px4ctrl/state contract for nmpc + planner) and
  // throttle-model telemetry (RLS eta / INDI).
  rclcpp::Publisher<interface::msg::State>::SharedPtr   state_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_world_pub;
  rclcpp::Publisher<interface::msg::ThrottleModelStatus>::SharedPtr
                                                           throttle_status_pub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr     set_FCU_mode_srv;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr        planner_reset_srv;

  quadrotor_msgs::msg::Px4ctrlDebug debug_msg;  // debug

  Eigen::Vector4d hover_pose;
  rclcpp::Time    last_set_hover_pose_time;

  /* While px4ctrl is locked (MANUAL_CTRL), the drone is NOT under command
   * control. Keep the l3 planner FSM reset to WAIT_TARGET so it stops
   * streaming /setpoint_cmd (a stale planner trajectory trips the
   * cmd_is_received takeoff/hover guard). Throttled — a fresh reset is all
   * the planner needs, no need to spam the service. Disabled by default;
   * enable with planner_reset.enable for stacks that own that service. */
  rclcpp::Time            last_planner_reset_time;
  static constexpr double PLANNER_RESET_INTERVAL_S = 0.2;

  enum FsmState
  {
    MANUAL_CTRL = 1,  // px4ctrl is deactivated. FCU is controlled by the remote
                      // controller only
    AUTO_HOVER,  // px4ctrl is activated, it will keep the drone hovering from
                 // odom measurements while waiting for commands from
                 // PositionCommand topic.
    CMD_CTRL,    // px4ctrl is activated, and controlling the drone.
    AUTO_TAKEOFF,
    AUTO_LAND,
    PASS_THROUGH  // external body-rate + thrust setpoints (NMPC rates bridge)
                  // are forwarded verbatim to mavros at the ctrl rate
  };

  static const char *state_name(FsmState s)
  {
    switch (s)
    {
      case MANUAL_CTRL:
        return "MANUAL_CTRL";
      case AUTO_HOVER:
        return "AUTO_HOVER";
      case CMD_CTRL:
        return "CMD_CTRL";
      case AUTO_TAKEOFF:
        return "AUTO_TAKEOFF";
      case AUTO_LAND:
        return "AUTO_LAND";
      case PASS_THROUGH:
        return "PASS_THROUGH";
      default:
        return "UNKNOWN";
    }
  }

  // mmz change 因为ros2 中srv 好像必须使用node来判断，所有加了一个node成员
  rclcpp::Node::SharedPtr node;
  PX4CtrlFSM(Parameters &, Controller &, rclcpp::Node::SharedPtr &node);
  void process();
  bool rc_is_received(const rclcpp::Time &now_time);
  bool cmd_is_received(const rclcpp::Time &now_time);
  bool odom_is_received(const rclcpp::Time &now_time);
  bool imu_is_received(const rclcpp::Time &now_time);
  bool bat_is_received(const rclcpp::Time &now_time);
  bool controls_is_received(const rclcpp::Time &now_time, double timeout_s);
  bool recv_new_odom();

  /**
   * State adapter: lock the origin on the first mavros odometry sample,
   * convert body twist to world, and publish the world-frame odometry.
   *
   * @param[in] msg mavros local position odometry (ENU world, FLU body) [-]
   */
  void state_adapter_odom_callback(
      const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * Fixed-rate /px4ctrl/state + /px4ctrl/state_odom publication for nmpc and
   * planner.
   */
  void     publish_state_adapter();
  FsmState get_state() { return state; }
  bool     get_landed() { return takeoff_land.landed; }

  /**
   * Synchronous PASS_THROUGH toggle for the external rates bridge
   * (e.g. NMPC). Safe to mutate `state` here: the service callback runs
   * inside the same single-threaded spin_some as process(), so the FSM
   * stays strictly serialized.
   *
   * @param[in] enable true = AUTO_HOVER -> PASS_THROUGH, false = back [-]
   * @return True when the transition was applied this call [-]
   */
  bool toggle_pass_through(bool enable);

 private:
  FsmState state;  // Should only be changed in PX4CtrlFSM::process() function!
  FsmState last_state;  // Track previous state for debug message
  AutoTakeoffLand takeoff_land;

  // ---- nmpc controller mode (internal rates + INDI) ----
  void publish_nmpc_pass_through(const rclcpp::Time &now_time);
  void publish_throttle_status(const rclcpp::Time &now_time,
                               uint64_t            timestamp_sample_us = 0);

  // ---- state adapter (odometry origin lock + /px4ctrl/state contract) ----
  Eigen::Vector3d       adapter_init_pos_{Eigen::Vector3d::Zero()};
  bool                  adapter_origin_set_{false};
  bool                  adapter_odom_valid_{false};
  interface::msg::State adapter_state_;

  // ---- control loop timing ----
  rclcpp::Time last_process_time_{rclcpp::Time(0, 0, RCL_SYSTEM_TIME)};
  double       control_dt_{0.005};

  // ---- control related ----
  DesiredState get_hover_des();
  DesiredState get_cmd_des();

  // ---- auto takeoff/land ----
  void         motors_idling(const ImuData &imu, ControllerOutput &u);
  void         land_detector(const FsmState state, const DesiredState &des,
                             const OdomData &odom);  // Detect landing
  void         set_start_pose_for_takeoff_land(const OdomData &odom);
  DesiredState get_rotor_speed_up_des(const rclcpp::Time now);
  DesiredState get_takeoff_land_des(const double speed);

  // ---- tools ----
  void set_hov_with_odom();
  void set_hov_with_rc();
  void apply_rc_vel_bias(DesiredState &des);

  bool toggle_offboard_mode(
      bool on_off);  // It will only try to toggle once, so not blocked.
  bool toggle_arm_disarm(
      bool arm);  // It will only try to toggle once, so not blocked.

  void publish_bodyrate_ctrl(const ControllerOutput &u,
                             const rclcpp::Time     &stamp);
  void publish_attitude_ctrl(const ControllerOutput &u,
                             const rclcpp::Time     &stamp);
  void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);
  void reset_planner(const rclcpp::Time &now_time);
};

#endif
