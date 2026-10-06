#include "PX4CtrlParam.h"

Parameters::Parameters() {}

void Parameters::config_from_ros_handle(const rclcpp::Node::SharedPtr &node)
{
  read_essential_param(node, "gain.Kp0", gain.Kp0);
  read_essential_param(node, "gain.Kp1", gain.Kp1);
  read_essential_param(node, "gain.Kp2", gain.Kp2);
  read_essential_param(node, "gain.Kv0", gain.Kv0);
  read_essential_param(node, "gain.Kv1", gain.Kv1);
  read_essential_param(node, "gain.Kv2", gain.Kv2);
  read_essential_param(node, "gain.Kvi0", gain.Kvi0);
  read_essential_param(node, "gain.Kvi1", gain.Kvi1);
  read_essential_param(node, "gain.Kvi2", gain.Kvi2);
  read_essential_param(node, "gain.KAngR", gain.KAngR);
  read_essential_param(node, "gain.KAngP", gain.KAngP);
  read_essential_param(node, "gain.KAngY", gain.KAngY);

  read_essential_param(node, "rotor_drag.x", rt_drag.x);
  read_essential_param(node, "rotor_drag.y", rt_drag.y);
  read_essential_param(node, "rotor_drag.z", rt_drag.z);
  read_essential_param(node, "rotor_drag.k_thrust_horz", rt_drag.k_thrust_horz);

  read_essential_param(node, "msg_timeout.odom", msg_timeout.odom);
  read_essential_param(node, "msg_timeout.rc", msg_timeout.rc);
  read_essential_param(node, "msg_timeout.cmd", msg_timeout.cmd);
  read_essential_param(node, "msg_timeout.imu", msg_timeout.imu);
  read_essential_param(node, "msg_timeout.bat", msg_timeout.bat);
  read_essential_param(node, "msg_timeout.pass_through",
                       msg_timeout.pass_through);

  read_essential_param(node, "pose_solver", pose_solver);
  read_essential_param(node, "enable_body_rate_ctrl", enable_body_rate_ctrl);
  read_essential_param(node, "mass", mass);
  read_essential_param(node, "gra", gra);
  read_essential_param(node, "ctrl_freq_max", ctrl_freq_max);
  read_essential_param(node, "max_manual_vel", max_manual_vel);
  read_essential_param(node, "max_angle", max_angle);
  read_essential_param(node, "low_voltage", low_voltage);

  max_yaw_target_mode_rate_max = 1.0;
  yaw_target_kp                = 1.0;
  yaw_target_deadband          = 2.0 / 180.0 * M_PI;
  rc_cmd_vel_bias_scale        = 0.5;
  read_optional_param(node, "max_yaw_target_mode_rate_max",
                      max_yaw_target_mode_rate_max);
  read_optional_param(node, "yaw_target_kp", yaw_target_kp);
  read_optional_param(node, "yaw_target_deadband", yaw_target_deadband);
  read_optional_param(node, "rc_cmd_vel_bias_scale", rc_cmd_vel_bias_scale);

  read_essential_param(node, "rc_reverse.roll", rc_reverse.roll);
  read_essential_param(node, "rc_reverse.pitch", rc_reverse.pitch);
  read_essential_param(node, "rc_reverse.yaw", rc_reverse.yaw);
  read_essential_param(node, "rc_reverse.throttle", rc_reverse.throttle);

  read_essential_param(node, "auto_takeoff_land.enable", takeoff_land.enable);
  read_essential_param(node, "auto_takeoff_land.no_RC", takeoff_land.no_RC);
  read_essential_param(node, "auto_takeoff_land.takeoff_height",
                       takeoff_land.height);
  read_essential_param(node, "auto_takeoff_land.takeoff_land_speed",
                       takeoff_land.speed);

  planner_reset.enable  = false;
  planner_reset.service = "/planner/reset";
  read_optional_param(node, "planner_reset.enable", planner_reset.enable);
  read_optional_param(node, "planner_reset.service", planner_reset.service);

  read_essential_param(node, "thrust_model.print_value", thr_map.print_val);
  read_essential_param(node, "thrust_model.K1", thr_map.K1);
  read_essential_param(node, "thrust_model.K2", thr_map.K2);
  read_essential_param(node, "thrust_model.K3", thr_map.K3);
  read_essential_param(node, "thrust_model.accurate_thrust_model",
                       thr_map.accurate_thrust_model);
  read_essential_param(node, "thrust_model.hover_percentage",
                       thr_map.hover_percentage);
  read_essential_param(node, "thrust_model.noisy_imu", thr_map.noisy_imu);

  read_essential_param(node, "ctrl_mode", ctrl_mode);
  read_essential_param(node, "throttle_estimator", throttle_estimator);
  read_essential_param(node, "odom_twist_is_body", odom_twist_is_body);

  if (ctrl_mode != 0 && ctrl_mode != 1)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "ctrl_mode must be 0 (tracking) or 1 (nmpc), got %d",
                 ctrl_mode);
    throw std::runtime_error("invalid ctrl_mode");
  }
  if (throttle_estimator != 0 && throttle_estimator != 1)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "throttle_estimator must be 0 (mapping) or 1 (indi), got %d",
                 throttle_estimator);
    throw std::runtime_error("invalid throttle_estimator");
  }

  read_essential_param(node, "rls.lambda", rls.lambda);
  read_essential_param(node, "rls.eta_min", rls.eta_min);
  read_essential_param(node, "rls.eta_max", rls.eta_max);
  read_essential_param(node, "rls.u_min", rls.u_min);
  read_essential_param(node, "rls.p_init", rls.p_init);
  read_essential_param(node, "rls.var_valid_thr", rls.var_valid_thr);
  read_essential_param(node, "rls.innovation_gate", rls.innovation_gate);

  read_essential_param(node, "indi.specific_force_p", indi.specific_force_p);
  read_essential_param(node, "indi.throttle_min", indi.throttle_min);
  read_essential_param(node, "indi.throttle_max", indi.throttle_max);
  read_essential_param(node, "indi.throttle_lpf_cutoff_hz",
                       indi.throttle_lpf_cutoff_hz);
  read_essential_param(node, "indi.setpoint_lpf_cutoff_hz",
                       indi.setpoint_lpf_cutoff_hz);
  read_essential_param(node, "indi.meas_lpf_cutoff_hz",
                       indi.meas_lpf_cutoff_hz);

  max_angle /= (180.0 / M_PI);

  if (takeoff_land.no_RC && !takeoff_land.enable)
  {
    takeoff_land.no_RC = false;
    RCLCPP_ERROR(
        node->get_logger(),
        "\"no_RC\" is only allowed with \"auto_takeoff_land\" enabled.");
  }

  if (thr_map.print_val)
  {
    RCLCPP_WARN(
        node->get_logger(),
        "You should disable \"print_value\" if you are in regular usage.");
  }
}
