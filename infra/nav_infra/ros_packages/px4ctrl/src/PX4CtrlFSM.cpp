#include "PX4CtrlFSM.h"

#include <uav_utils/converters.h>

#include <algorithm>

#include "px4ctrl_trace.h"

using namespace std;
using namespace uav_utils;

static rclcpp::Clock g_throttle_clock(RCL_STEADY_TIME);

PX4CtrlFSM::PX4CtrlFSM(Parameters &param_, Controller &controller_,
                       rclcpp::Node::SharedPtr &node_)
    : param(param_),
      controller(controller_),
      node(node_) /*, thrust_curve(thrust_curve_)*/
{
  state      = MANUAL_CTRL;
  last_state = MANUAL_CTRL;
  hover_pose.setZero();
  last_planner_reset_time = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

// clang-format off
/*
 * Finite State Machine
 *
 *   [start] --> MANUAL_CTRL
 *
 *   flight chain (all fallback edges return to MANUAL_CTRL unless noted):
 *     AUTO_TAKEOFF --> AUTO_HOVER <--> CMD_CTRL
 *                          |    ^
 *                          |    +-- PASS_THROUGH
 *                          v
 *                       AUTO_LAND --> MANUAL_CTRL
 *
 *   from            to              guard
 *   --------------  --------------  --------------------------------------------
 *   MANUAL_CTRL     AUTO_HOVER      RC hover mode on
 *   MANUAL_CTRL     AUTO_TAKEOFF    /px4ctrl/takeoff_land == TAKEOFF (only arm path)
 *   AUTO_TAKEOFF    AUTO_HOVER      takeoff height reached
 *   AUTO_TAKEOFF    MANUAL_CTRL     unarmed > 3 s (arm rejected/lost)
 *   AUTO_HOVER      CMD_CTRL        RC command mode + /setpoint_cmd fresh + OFFBOARD
 *   CMD_CTRL        AUTO_HOVER      RC command mode off / cmd timeout
 *   AUTO_HOVER      AUTO_LAND       /px4ctrl/takeoff_land == LAND
 *   AUTO_LAND       AUTO_HOVER      RC command mode off
 *   AUTO_LAND       MANUAL_CTRL     landed on ground + disarmed
 *   AUTO_HOVER      PASS_THROUGH    toggle_pass_through(true) [armed, stream fresh]
 *   PASS_THROUGH    AUTO_HOVER      toggle_pass_through(false) / stream stale /
 *                                   odom|imu timeout
 *   AUTO_HOVER      MANUAL_CTRL     disarm / RC hover off / odom timeout
 *   CMD_CTRL        MANUAL_CTRL     disarm / RC hover off / odom timeout
 *   AUTO_LAND       MANUAL_CTRL     RC hover off / odom timeout
 *   PASS_THROUGH    MANUAL_CTRL     disarm
 *
 *   PASS_THROUGH sources body-rate + thrust from the internal NMPC
 *   controller (ctrl_mode=1: /nmpc/control -> RLS eta + INDI) or from an
 *   external rates stream (ctrl_mode=0); px4ctrl stays the single mavros
 *   setpoint publisher. The RC-free profile (no_RC) reaches flight only
 *   through AUTO_TAKEOFF.
 */
// clang-format on

void PX4CtrlFSM::process()
{
  rclcpp::Time now_time = rclcpp::Clock().now();

  // Control period for the discrete INDI law and RLS/LPF filters.
  if (last_process_time_ != rclcpp::Time(0, 0, RCL_SYSTEM_TIME))
  {
    control_dt_ = (now_time - last_process_time_).seconds();
  }
  last_process_time_ = now_time;
  controller.setControlDt(control_dt_);

  // Freshness of the remote rates source for PASS_THROUGH: the internal
  // NMPC controls when ctrl_mode=1, the external stream otherwise.
  const bool remote_stream_fresh =
      param.ctrl_mode == 1
          ? controls_data.is_received(now_time, param.msg_timeout.pass_through)
          : pass_through_data.is_received(now_time,
                                          param.msg_timeout.pass_through);

  ControllerOutput u;
  DesiredState     des(odom_data);
  double           yaw_target                  = des.yaw;
  bool             yaw_target_mode             = false;
  bool             rotor_low_speed_during_land = false;

  // STEP1: state machine runs
  switch (state)
  {
    case MANUAL_CTRL:
    {
      reset_planner(now_time);
      if (rc_data.enter_hover_mode)  // Try to jump to AUTO_HOVER
      {
        if (!odom_is_received(now_time))
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_HOVER(L2). No odom!",
                       state_name(state));
          break;
        }
        if (cmd_is_received(now_time))
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_HOVER(L2). You are sending "
                       "commands before toggling into AUTO_HOVER, which is not "
                       "allowed. Stop sending commands now!",
                       state_name(state));
          break;
        }
        if (odom_data.v.norm() > 3.0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, "
                       "which seems that the localization module goes wrong!",
                       state_name(state), odom_data.v.norm());
          break;
        }

        state = AUTO_HOVER;
        controller.resetThrustMapping();
        set_hov_with_odom();
        toggle_offboard_mode(true);

        px4ctrl_trace::transition("MANUAL_CTRL", "AUTO_HOVER",
                                  "rc_hover_mode_on");
      }
      else if (param.takeoff_land.enable && takeoff_land_data.triggered &&
               takeoff_land_data.takeoff_land_cmd ==
                   quadrotor_msgs::msg::TakeoffLand::TAKEOFF)  // Try to jump to
                                                               // AUTO_TAKEOFF
      {
        if (!odom_is_received(now_time))
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_TAKEOFF. No odom!",
                       state_name(state));
          break;
        }
        if (cmd_is_received(now_time))
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_TAKEOFF. You are sending "
                       "commands before toggling into AUTO_TAKEOFF, which is "
                       "not allowed. Stop sending commands now!",
                       state_name(state));
          break;
        }
        if (odom_data.v.norm() > 0.1)
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, "
                       "non-static takeoff is not allowed!",
                       state_name(state), odom_data.v.norm());
          break;
        }
        if (!get_landed())
        {
          RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                       "[px4ctrl] [%s] Reject AUTO_TAKEOFF. Land detector says "
                       "that the drone is not landed now!",
                       state_name(state));
          break;
        }
        if (rc_is_received(now_time))  // Check this only if RC is connected.
        {
          if (!rc_data.is_hover_mode || !rc_data.is_command_mode ||
              !rc_data.check_centered())
          {
            RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                         "[px4ctrl] [%s] Reject AUTO_TAKEOFF. If you have your "
                         "RC connected, keep its switches at \"auto hover\" "
                         "and \"command control\" states, and all sticks at "
                         "the center, then takeoff again.",
                         state_name(state));
            while (rclcpp::ok())
            {
              rclcpp::Rate(100).sleep();
              rclcpp::spin_some(
                  node);  // mmz 在while中必须spin node, 不然会阻塞外部的订阅
              if (rc_data.is_hover_mode && rc_data.is_command_mode &&
                  rc_data.check_centered())
              {
                px4ctrl_trace::emit("takeoff_retry_ready");
                break;
              }
            }
            break;
          }
        }

        state = AUTO_TAKEOFF;
        controller.resetThrustMapping();
        set_start_pose_for_takeoff_land(odom_data);
        takeoff_land.not_armed_since = rclcpp::Time(
            0, 0, RCL_SYSTEM_TIME);  // start dead-zone fresh on every takeoff
        toggle_offboard_mode(true);  // toggle on offboard before arm
        for (int i = 0; i < 10 && rclcpp::ok();
             ++i)  // wait for 0.1 seconds to allow mode change by FMU // mark
        {
          rclcpp::Rate(100).sleep();
          rclcpp::spin_some(node);
        }
        toggle_arm_disarm(true);  // auto-arm is mandatory
        takeoff_land.toggle_takeoff_land_time = now_time;

        px4ctrl_trace::transition("MANUAL_CTRL", "AUTO_TAKEOFF",
                                  "takeoff_command");
      }

      break;
    }

    case AUTO_HOVER:
    {
      if (!state_data.current_state.armed)
      {
        // Not armed: AUTO_HOVER is only valid while flying. Fall back to
        // MANUAL_CTRL so the AUTO_TAKEOFF path (the only path that arms)
        // becomes reachable again.
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition("AUTO_HOVER", "MANUAL_CTRL", "not_armed");
      }
      else if (!rc_data.is_hover_mode || !odom_is_received(now_time))
      {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition(
            "AUTO_HOVER", "MANUAL_CTRL",
            !rc_data.is_hover_mode ? "rc_hover_mode_off" : "odom_timeout");
      }
      else if (rc_data.is_command_mode && cmd_is_received(now_time))
      {
        if (state_data.current_state.mode == "OFFBOARD")
        {
          state = CMD_CTRL;
          des   = get_cmd_des();
          px4ctrl_trace::transition("AUTO_HOVER", "CMD_CTRL",
                                    "cmd_stream_started");
        }
      }
      else if (takeoff_land_data.triggered &&
               takeoff_land_data.takeoff_land_cmd ==
                   quadrotor_msgs::msg::TakeoffLand::LAND)
      {
        state = AUTO_LAND;
        set_start_pose_for_takeoff_land(odom_data);

        px4ctrl_trace::transition("AUTO_HOVER", "AUTO_LAND", "land_command");
      }
      else
      {
        set_hov_with_rc();
        des = get_hover_des();
        if ((rc_data.enter_command_mode) ||
            (takeoff_land.delay_trigger.first &&
             now_time > takeoff_land.delay_trigger.second))
        {
          takeoff_land.delay_trigger.first = false;
          publish_trigger(odom_data.msg);
          px4ctrl_trace::emit("trigger_sent");
        }
      }

      break;
    }

    case CMD_CTRL:
    {
      if (!state_data.current_state.armed)
      {
        // Not armed: cannot command the FCU. Fall back to MANUAL_CTRL.
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition("CMD_CTRL", "MANUAL_CTRL", "not_armed");
      }
      else if (!rc_data.is_hover_mode || !odom_is_received(now_time))
      {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition(
            "CMD_CTRL", "MANUAL_CTRL",
            !rc_data.is_hover_mode ? "rc_hover_mode_off" : "odom_timeout");
      }
      else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
      {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        px4ctrl_trace::transition(
            "CMD_CTRL", "AUTO_HOVER",
            !rc_data.is_command_mode ? "rc_command_mode_off" : "cmd_timeout");
      }
      else
      {
        des = get_cmd_des();
        apply_rc_vel_bias(des);
      }

      if (takeoff_land_data.triggered &&
          takeoff_land_data.takeoff_land_cmd ==
              quadrotor_msgs::msg::TakeoffLand::LAND)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("px4ctrl"),
            "[px4ctrl] [%s] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
                Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
            state_name(state), param.msg_timeout.cmd);
      }

      break;
    }

    case AUTO_TAKEOFF:
    {
      // Dead-zone guard: if the vehicle stays unarmed (arm rejected or lost)
      // for NOT_ARMED_DEADZONE_TIME seconds, abort back to MANUAL_CTRL
      // instead of commanding an unarmed aircraft forever.
      if (!state_data.current_state.armed)
      {
        if (takeoff_land.not_armed_since == rclcpp::Time(0, 0, RCL_SYSTEM_TIME))
          takeoff_land.not_armed_since = now_time;
        else if ((now_time - takeoff_land.not_armed_since).seconds() >
                 AutoTakeoffLand::NOT_ARMED_DEADZONE_TIME)
        {
          state = MANUAL_CTRL;
          toggle_offboard_mode(false);
          takeoff_land.not_armed_since = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
          px4ctrl_trace::transition("AUTO_TAKEOFF", "MANUAL_CTRL",
                                    "not_armed_deadzone");
          break;
        }
      }
      else
      {
        takeoff_land.not_armed_since = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
      }

      if ((now_time - takeoff_land.toggle_takeoff_land_time).seconds() <
          AutoTakeoffLand::MOTORS_SPEEDUP_TIME)  // Wait for several seconds
                                                 // to warn people.
      {
        des = get_rotor_speed_up_des(now_time);
      }
      else if (odom_data.p(2) >=
               (takeoff_land.start_pose(2) +
                param.takeoff_land.height))  // reach the desired height
      {
        state = AUTO_HOVER;
        set_hov_with_odom();
        px4ctrl_trace::transition("AUTO_TAKEOFF", "AUTO_HOVER",
                                  "height_reached");
        px4ctrl_trace::emit("takeoff_completed");

        takeoff_land.delay_trigger.first = true;
        takeoff_land.delay_trigger.second =
            now_time +
            rclcpp::Duration::from_seconds(AutoTakeoffLand::DELAY_TRIGGER_TIME);
      }
      else
      {
        des = get_takeoff_land_des(param.takeoff_land.speed);
      }

      break;
    }

    case PASS_THROUGH:
    {
      if (!state_data.current_state.armed)
      {
        // Not armed: cannot command the FCU. Fall back to MANUAL_CTRL.
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition("PASS_THROUGH", "MANUAL_CTRL", "not_armed");
      }
      else if (!odom_is_received(now_time) || !imu_is_received(now_time))
      {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        controller.resetThrustMapping();
        px4ctrl_trace::transition(
            "PASS_THROUGH", "AUTO_HOVER",
            !odom_is_received(now_time) ? "odom_timeout" : "imu_timeout");
      }
      else if (!remote_stream_fresh)
      {
        // The rates stream went stale: fall back to hover on the latest
        // odometry so offboard continuity is preserved.
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        controller.resetThrustMapping();
        px4ctrl_trace::transition("PASS_THROUGH", "AUTO_HOVER",
                                  "remote_stream_timeout");
      }

      break;
    }

    case AUTO_LAND:
    {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time))
      {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);

        px4ctrl_trace::transition(
            "AUTO_LAND", "MANUAL_CTRL",
            !rc_data.is_hover_mode ? "rc_hover_mode_off" : "odom_timeout");
      }
      else if (!rc_data.is_command_mode)
      {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        px4ctrl_trace::transition("AUTO_LAND", "AUTO_HOVER",
                                  "rc_command_mode_off");
      }
      else if (!get_landed())
      {
        des = get_takeoff_land_des(-param.takeoff_land.speed);
      }
      else
      {
        rotor_low_speed_during_land = true;

        static bool print_once_flag = true;
        if (print_once_flag)
        {
          px4ctrl_trace::emit("land_ground_wait");
          print_once_flag = false;
        }

        if (extended_state_data.current_extended_state.landed_state ==
            mavros_msgs::msg::ExtendedState::
                LANDED_STATE_ON_GROUND)  // PX4 allows disarm after this
        {
          static double last_trial_time = 0;  // Avoid too frequent calls
          if (now_time.seconds() - last_trial_time > 1.0)
          {
            if (toggle_arm_disarm(false))  // disarm
            {
              print_once_flag = true;
              state           = MANUAL_CTRL;
              toggle_offboard_mode(false);  // toggle off offboard after disarm
              px4ctrl_trace::transition("AUTO_LAND", "MANUAL_CTRL",
                                        "landed_disarmed");
              px4ctrl_trace::emit("land_completed");
            }

            last_trial_time = now_time.seconds();
          }
        }
      }

      break;
    }

    default:
      break;
  }

  if (state == PASS_THROUGH)
  {
    if (param.ctrl_mode == 1)
    {
      // Internal NMPC controller: rates from /nmpc/control + INDI
      // throttle, produced by px4ctrl itself.
      publish_nmpc_pass_through(now_time);
    }
    else
    {
      // Forward the latest external body-rate + thrust setpoint verbatim
      // at the ctrl rate (repeat-forward keeps the offboard stream alive
      // even when the producer runs slower than this loop).
      mavros_msgs::msg::AttitudeTarget msg = pass_through_data.msg;
      msg.header.stamp                     = now_time;
      msg.header.frame_id                  = "FCU";
      ctrl_FCU_pub->publish(msg);
    }

    // STEP8 bookkeeping for the bypassed pipeline
    last_state = state;
    return;
  }

  // STEP2: solve and update new control commands
  yaw_target_mode = des.yaw_control_mode ==
                    quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TARGET;
  if (yaw_target_mode)
  {
    // (1.a) Preserve the absolute yaw target, but hide it from flatness so
    // position, roll/pitch, and thrust control run without yaw feedforward.
    yaw_target   = des.yaw;
    des.yaw      = get_yaw_from_quaternion(odom_data.q);
    des.yaw_rate = 0.0;
  }
  if (rotor_low_speed_during_land)  // used at the start of auto takeoff
  {
    motors_idling(imu_data, u);
  }
  else
  {
    try
    {
      debug_msg = controller.update(des, odom_data, imu_data, u, bat_data.volt);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"), "[px4ctrl] [%s] %s",
                   state_name(state), e.what());
      return;
    }
    debug_msg.header.stamp   = now_time;
    debug_msg.fsm_state      = static_cast<double>(state);
    debug_msg.last_fsm_state = static_cast<double>(last_state);
    debug_pub->publish(debug_msg);
  }
  if (yaw_target_mode)
  {
    // (1.b) After the position controller runs, replace only the yaw body
    // rate with the bounded shortest-error YAW_TARGET controller output.
    u.bodyrates.z() = controller.computeYawTargetBodyRate(
        yaw_target, get_yaw_from_quaternion(odom_data.q));
  }

  // STEP3: estimate thrust model
  if (state == AUTO_HOVER || state == CMD_CTRL)
  {
    if (param.throttle_estimator == 1)
    {
      controller.updateIndiThrottleModel(imu_data.a(2));
      publish_throttle_status(now_time);
    }
    else
    {
      controller.estimateThrustModel(imu_data.a, bat_data.volt, odom_data.v);
    }
  }

  // STEP4: publish control commands to mavros
  if (param.enable_body_rate_ctrl)
  {
    publish_bodyrate_ctrl(u, now_time);
  }
  else
  {
    publish_attitude_ctrl(u, now_time);
  }

  // STEP5: Detect if the drone has landed
  land_detector(state, des, odom_data);

  // STEP6: Periodic status print (same throttled pattern as input.cpp)
  {
    static rclcpp::Time last_status_print = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    if (bat_data.percentage > 0.05)
    {
      if ((now_time - last_status_print).seconds() > 5.0)
      {
        RCLCPP_INFO(rclcpp::get_logger("px4ctrl"),
                    "[px4ctrl] State: %s(%d), Voltage=%.3f, percentage=%.3f",
                    state_name(state), state, bat_data.volt,
                    bat_data.percentage);
        last_status_print = now_time;
      }
    }
    else
    {
      if ((now_time - last_status_print).seconds() > 1.0)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("px4ctrl"),
            "[px4ctrl] State: %s(%d), Dangerous! voltage=%.3f, percentage=%.3f",
            state_name(state), state, bat_data.volt, bat_data.percentage);
        last_status_print = now_time;
      }
    }
  }

  // STEP7: Clear flags beyond their lifetime
  rc_data.enter_hover_mode    = false;
  rc_data.enter_command_mode  = false;
  takeoff_land_data.triggered = false;

  // STEP8: Track state transition for debug
  last_state = state;
}

void PX4CtrlFSM::motors_idling(const ImuData &imu, ControllerOutput &u)
{
  u.q         = imu.q;
  u.bodyrates = Eigen::Vector3d::Zero();
  u.thrust    = 0.04;
}

void PX4CtrlFSM::land_detector(const FsmState state, const DesiredState &des,
                               const OdomData &odom)
{
  static FsmState last_state = FsmState::MANUAL_CTRL;
  if (last_state == FsmState::MANUAL_CTRL &&
      (state == FsmState::AUTO_HOVER || state == FsmState::AUTO_TAKEOFF))
  {
    takeoff_land.landed = false;  // Always holds
  }
  last_state = state;

  if (state == FsmState::MANUAL_CTRL && !state_data.current_state.armed)
  {
    takeoff_land.landed = true;
    return;  // No need of other decisions
  }

  // land_detector parameters
  constexpr double POSITION_DEVIATION_C =
      -0.5;  // Constraint 1: target position below real position for
             // POSITION_DEVIATION_C meters.
  constexpr double VELOCITY_THR_C =
      0.1;  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
  constexpr double TIME_KEEP_C =
      3.0;  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

  static rclcpp::Time time_C12_reached;  // time_Constraints12_reached
  static bool         is_last_C12_satisfy;
  if (takeoff_land.landed)
  {
    time_C12_reached    = rclcpp::Clock().now();
    is_last_C12_satisfy = false;
  }
  else
  {
    bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C &&
                       odom.v.norm() < VELOCITY_THR_C;
    if (C12_satisfy && !is_last_C12_satisfy)
    {
      time_C12_reached = rclcpp::Clock().now();
    }
    else if (C12_satisfy && is_last_C12_satisfy)
    {
      if ((rclcpp::Clock().now() - time_C12_reached).seconds() >
          TIME_KEEP_C)  // Constraint 3 reached
      {
        takeoff_land.landed = true;
      }
    }

    is_last_C12_satisfy = C12_satisfy;
  }
}

DesiredState PX4CtrlFSM::get_hover_des()
{
  DesiredState des;
  des.p        = hover_pose.head<3>();
  des.v        = Eigen::Vector3d::Zero();
  des.a        = Eigen::Vector3d::Zero();
  des.j        = Eigen::Vector3d::Zero();
  des.yaw      = hover_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

DesiredState PX4CtrlFSM::get_cmd_des()
{
  DesiredState des;
  des.p                = cmd_data.p;
  des.v                = cmd_data.v;
  des.a                = cmd_data.a;
  des.j                = cmd_data.j;
  des.yaw              = cmd_data.yaw;
  des.yaw_rate         = cmd_data.yaw_rate;
  des.yaw_control_mode = cmd_data.yaw_control_mode;

  return des;
}

DesiredState PX4CtrlFSM::get_rotor_speed_up_des(const rclcpp::Time now)
{
  double delta_t = (now - takeoff_land.toggle_takeoff_land_time).seconds();
  double des_a_z =
      exp((delta_t - AutoTakeoffLand::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 -
      7.0;  // Parameters 6.0 and 7.0 are just heuristic values which result in
            // a satisfactory curve.
  if (des_a_z > 0.1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                 "[px4ctrl] [%s] des_a_z > 0.1!, des_a_z=%f", state_name(state),
                 des_a_z);
    des_a_z = 0.0;
  }

  DesiredState des;
  des.p        = takeoff_land.start_pose.head<3>();
  des.v        = Eigen::Vector3d::Zero();
  des.a        = Eigen::Vector3d(0, 0, des_a_z);
  des.j        = Eigen::Vector3d::Zero();
  des.yaw      = takeoff_land.start_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

DesiredState PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
  rclcpp::Time now = rclcpp::Clock().now();
  double delta_t   = (now - takeoff_land.toggle_takeoff_land_time).seconds() -
                     (speed > 0 ? AutoTakeoffLand::MOTORS_SPEEDUP_TIME
                                : 0);  // speed > 0 means takeoff

  DesiredState des;
  des.p        = takeoff_land.start_pose.head<3>() +
                 Eigen::Vector3d(0, 0, speed * delta_t);
  des.v        = Eigen::Vector3d(0, 0, speed);
  des.a        = Eigen::Vector3d::Zero();
  des.j        = Eigen::Vector3d::Zero();
  des.yaw      = takeoff_land.start_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
  hover_pose.head<3>() = odom_data.p;
  hover_pose(3)        = get_yaw_from_quaternion(odom_data.q);

  last_set_hover_pose_time = rclcpp::Clock().now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
  rclcpp::Time now         = rclcpp::Clock().now();
  double       delta_t     = (now - last_set_hover_pose_time).seconds();
  last_set_hover_pose_time = now;

  // Move the hover point in the body frame (yaw-rotated), so the sticks
  // always push the drone forward/right regardless of its heading.
  double current_yaw = get_yaw_from_quaternion(odom_data.q);

  double vx_body =
      rc_data.ch[1] * param.max_manual_vel * (param.rc_reverse.pitch ? 1 : -1);
  double vy_body =
      rc_data.ch[0] * param.max_manual_vel * (param.rc_reverse.roll ? 1 : -1);

  Eigen::Vector3d new_hover_pos = hover_pose.head<3>();
  new_hover_pos(0) +=
      (cos(current_yaw) * vx_body - sin(current_yaw) * vy_body) * delta_t;
  new_hover_pos(1) +=
      (sin(current_yaw) * vx_body + cos(current_yaw) * vy_body) * delta_t;
  new_hover_pos(2) += rc_data.ch[2] * param.max_manual_vel * delta_t *
                      (param.rc_reverse.throttle ? 1 : -1);

  hover_pose.head<3>() = new_hover_pos;

  hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t *
                   (param.rc_reverse.yaw ? 1 : -1);
}

void PX4CtrlFSM::apply_rc_vel_bias(DesiredState &des)
{
  if (!rc_is_received(rclcpp::Clock().now()))
    return;

  double current_yaw = get_yaw_from_quaternion(odom_data.q);

  double vx_body = rc_data.ch[1] * param.max_manual_vel *
                   param.rc_cmd_vel_bias_scale *
                   (param.rc_reverse.pitch ? 1 : -1);
  double vy_body = rc_data.ch[0] * param.max_manual_vel *
                   param.rc_cmd_vel_bias_scale *
                   (param.rc_reverse.roll ? 1 : -1);

  des.v.x() += cos(current_yaw) * vx_body - sin(current_yaw) * vy_body;
  des.v.y() += sin(current_yaw) * vx_body + cos(current_yaw) * vy_body;
  des.v.z() += rc_data.ch[2] * param.max_manual_vel *
               param.rc_cmd_vel_bias_scale *
               (param.rc_reverse.throttle ? 1 : -1);
  des.yaw_rate += rc_data.ch[3] * param.max_manual_vel *
                  param.rc_cmd_vel_bias_scale * (param.rc_reverse.yaw ? 1 : -1);
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const OdomData &odom)
{
  takeoff_land.start_pose.head<3>() = odom_data.p;
  takeoff_land.start_pose(3)        = get_yaw_from_quaternion(odom_data.q);

  takeoff_land.toggle_takeoff_land_time = rclcpp::Clock().now();
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
  return rc_data.valid && !rc_data.kill_switch_engaged &&
         (now_time - rc_data.rcv_stamp).seconds() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time &now_time)
{
  return (now_time - cmd_data.rcv_stamp).seconds() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time &now_time)
{
  return (now_time - odom_data.rcv_stamp).seconds() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time &now_time)
{
  return (now_time - imu_data.rcv_stamp).seconds() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time)
{
  return (now_time - bat_data.rcv_stamp).seconds() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
  if (odom_data.recv_new_msg)
  {
    odom_data.recv_new_msg = false;
    return true;
  }

  return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const ControllerOutput &u,
                                       const rclcpp::Time     &stamp)
{
  auto msg = mavros_msgs::msg::AttitudeTarget();

  msg.header.stamp    = stamp;
  msg.header.frame_id = "FCU";

  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

  msg.body_rate.x = u.bodyrates.x();
  msg.body_rate.y = u.bodyrates.y();
  msg.body_rate.z = u.bodyrates.z();

  msg.thrust = u.thrust;

  ctrl_FCU_pub->publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const ControllerOutput &u,
                                       const rclcpp::Time     &stamp)
{
  auto msg = mavros_msgs::msg::AttitudeTarget();

  msg.header.stamp    = stamp;
  msg.header.frame_id = "FCU";

  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

  msg.orientation.x = u.q.x();
  msg.orientation.y = u.q.y();
  msg.orientation.z = u.q.z();
  msg.orientation.w = u.q.w();

  msg.thrust = u.thrust;

  ctrl_FCU_pub->publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::msg::Odometry &odom_msg)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp    = rclcpp::Clock().now();
  msg.pose            = odom_msg.pose.pose;

  traj_start_trigger_pub->publish(msg);
}

void PX4CtrlFSM::reset_planner(const rclcpp::Time &now_time)
{
  if (!param.planner_reset.enable)
    return;

  if ((now_time - last_planner_reset_time).seconds() < PLANNER_RESET_INTERVAL_S)
    return;
  last_planner_reset_time = now_time;

  if (!planner_reset_srv)
    return;

  if (!planner_reset_srv->service_is_ready())
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("px4ctrl"), g_throttle_clock, 5000,
                         "[px4ctrl] [%s] %s unavailable", state_name(state),
                         param.planner_reset.service.c_str());
    px4ctrl_trace::emit("planner_reset_failed");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  planner_reset_srv->async_send_request(req);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
  auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  if (on_off)
  {
    state_data.state_before_offboard = state_data.current_state;
    if (state_data.state_before_offboard.mode == "OFFBOARD")  // Not allowed
      state_data.state_before_offboard.mode = "MANUAL";

    offb_set_mode->custom_mode = "OFFBOARD";
    auto result = set_FCU_mode_srv->async_send_request(offb_set_mode);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(),
                                           result) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !result.get()->mode_sent)
    {
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] [%s] Enter OFFBOARD rejected by PX4!",
                   state_name(state));
      return false;
    }
  }
  else
  {
    offb_set_mode->custom_mode = state_data.state_before_offboard.mode;
    auto result = set_FCU_mode_srv->async_send_request(offb_set_mode);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(),
                                           result) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !result.get()->mode_sent)
    {
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] [%s] Exit OFFBOARD rejected by PX4!",
                   state_name(state));
      return false;
    }
  }

  return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
  auto arm_cmd   = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  arm_cmd->value = arm;

  auto result = arming_client_srv->async_send_request(arm_cmd);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(),
                                         result) !=
          rclcpp::FutureReturnCode::SUCCESS ||
      !result.get()->success)
  {
    if (arm)
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] [%s] ARM rejected by PX4! Kill-switch activated?",
                   state_name(state));
    else
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] [%s] DISARM rejected by PX4!", state_name(state));

    return false;
  }

  return true;
}

bool PX4CtrlFSM::toggle_pass_through(bool enable)
{
  const rclcpp::Time now_time = rclcpp::Clock().now();

  if (enable)
  {
    if (state != AUTO_HOVER)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("px4ctrl"),
          "[px4ctrl] [%s] Reject PASS_THROUGH: only AUTO_HOVER can switch over",
          state_name(state));
      return false;
    }
    if (!state_data.current_state.armed)
    {
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] Reject PASS_THROUGH: not armed");
      return false;
    }
    const bool stream_ready =
        param.ctrl_mode == 1 ? controls_data.is_received(
                                   now_time, param.msg_timeout.pass_through)
                             : pass_through_data.is_received(
                                   now_time, param.msg_timeout.pass_through);
    if (!stream_ready)
    {
      RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                   "[px4ctrl] Reject PASS_THROUGH: %s stream not received yet",
                   param.ctrl_mode == 1 ? "NMPC controls" : "external rates");
      return false;
    }

    state = PASS_THROUGH;
    px4ctrl_trace::transition("AUTO_HOVER", "PASS_THROUGH", "service_on");
    return true;
  }

  if (state != PASS_THROUGH)
  {
    RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                 "[px4ctrl] [%s] Reject PASS_THROUGH exit: not in PASS_THROUGH",
                 state_name(state));
    return false;
  }

  state = AUTO_HOVER;
  set_hov_with_odom();
  controller.resetThrustMapping();
  px4ctrl_trace::transition("PASS_THROUGH", "AUTO_HOVER", "service_off");
  return true;
}

bool PX4CtrlFSM::controls_is_received(const rclcpp::Time &now_time,
                                      double              timeout_s)
{
  return controls_data.is_received(now_time, timeout_s);
}

void PX4CtrlFSM::publish_nmpc_pass_through(const rclcpp::Time &now_time)
{
  const interface::msg::Control &controls = controls_data.msg;

  ControllerOutput u;
  u.bodyrates = Eigen::Vector3d(controls.rates_sp[0], controls.rates_sp[1],
                                controls.rates_sp[2]);
  u.thrust =
      controller.computeNmpcThrottle(imu_data.a(2), controls.specific_force_sp);

  publish_bodyrate_ctrl(u, now_time);
  publish_throttle_status(now_time, controls.timestamp);
}

void PX4CtrlFSM::publish_throttle_status(const rclcpp::Time &now_time,
                                         uint64_t timestamp_sample_us)
{
  interface::msg::ThrottleModelStatus status;
  status.timestamp        = now_time.nanoseconds() / 1000;
  status.timestamp_sample = timestamp_sample_us;
  controller.fillThrottleStatus(status);
  throttle_status_pub->publish(status);
}

void PX4CtrlFSM::state_adapter_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!adapter_origin_set_)
  {
    adapter_init_pos_ =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    adapter_origin_set_ = true;
    RCLCPP_INFO(rclcpp::get_logger("px4ctrl"),
                "[px4ctrl] State origin locked at (%.2f, %.2f, %.2f)",
                adapter_init_pos_.x(), adapter_init_pos_.y(),
                adapter_init_pos_.z());
  }

  const Eigen::Vector3d    p(msg->pose.pose.position.x - adapter_init_pos_.x(),
                             msg->pose.pose.position.y - adapter_init_pos_.y(),
                             msg->pose.pose.position.z - adapter_init_pos_.z());
  const Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  // mavros local_position/odom twist is body-frame (FLU); the /px4ctrl/state
  // contract (px4ctrl / nmpc / ekf_quat lineage) is world-frame.
  const Eigen::Vector3d v =
      q * Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);
  const Eigen::Vector3d w(msg->twist.twist.angular.x,
                          msg->twist.twist.angular.y,
                          msg->twist.twist.angular.z);

  // World-frame corrected odometry for external observers.
  nav_msgs::msg::Odometry odom_world = *msg;
  odom_world.header.frame_id         = "world";
  odom_world.child_frame_id.clear();
  odom_world.twist.twist.linear.x = v.x();
  odom_world.twist.twist.linear.y = v.y();
  odom_world.twist.twist.linear.z = v.z();
  odom_world_pub->publish(odom_world);

  using State = interface::msg::State;
  std::copy(p.data(), p.data() + 3,
            adapter_state_.state.begin() + State::POS_START_INDEX);
  std::copy(v.data(), v.data() + 3,
            adapter_state_.state.begin() + State::VEL_START_INDEX);
  std::copy(w.data(), w.data() + 3,
            adapter_state_.state.begin() + State::RATE_START_INDEX);
  const Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
  std::copy(q_vec.data(), q_vec.data() + 4,
            adapter_state_.state.begin() + State::QUAT_START_INDEX);
  adapter_state_.cur_cz = 0.0;
  adapter_odom_valid_   = true;
}

void PX4CtrlFSM::publish_state_adapter()
{
  if (!adapter_origin_set_ || !adapter_odom_valid_)
  {
    return;
  }

  const rclcpp::Time now_time = rclcpp::Clock().now();
  adapter_state_.timestamp    = now_time.nanoseconds() / 1000;
  state_pub->publish(adapter_state_);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = now_time;
  odom.header.frame_id = "world";
  using State          = interface::msg::State;

  odom.pose.pose.position.x = adapter_state_.state[State::POS_START_INDEX];
  odom.pose.pose.position.y = adapter_state_.state[State::POS_START_INDEX + 1];
  odom.pose.pose.position.z = adapter_state_.state[State::POS_START_INDEX + 2];
  odom.pose.pose.orientation.w = adapter_state_.state[State::QUAT_START_INDEX];
  odom.pose.pose.orientation.x =
      adapter_state_.state[State::QUAT_START_INDEX + 1];
  odom.pose.pose.orientation.y =
      adapter_state_.state[State::QUAT_START_INDEX + 2];
  odom.pose.pose.orientation.z =
      adapter_state_.state[State::QUAT_START_INDEX + 3];
  state_odom_pub->publish(odom);
}
