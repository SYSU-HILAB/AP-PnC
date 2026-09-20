#include "input.h"

static rclcpp::Clock g_throttle_clock(RCL_STEADY_TIME);

RcData::RcData()
{
  rcv_stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  last_mode           = -1.0;
  last_gear           = -1.0;
  valid               = false;
  kill_switch_engaged = true;

  // Parameter initialization is very important in RC-Free usage!
  is_hover_mode      = true;
  enter_hover_mode   = false;
  is_command_mode    = true;
  enter_command_mode = false;
  for (int i = 0; i < 4; ++i)
  {
    ch[i] = 0.0;
  }
}

void RcData::feed(mavros_msgs::msg::RCIn::ConstPtr pMsg)
{
  msg       = *pMsg;
  rcv_stamp = rclcpp::Clock().now();

  valid = false;
  if (msg.channels.size() < MIN_CHANNELS)
  {
    kill_switch_engaged = true;
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RcData"), g_throttle_clock, 1000,
        "[px4ctrl] RC input has %zu channels, need at least %d.",
        msg.channels.size(), MIN_CHANNELS);
    return;
  }

  for (int i = 0; i < 4; i++)
  {
    ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
    if (ch[i] > DEAD_ZONE)
      ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
    else if (ch[i] < -DEAD_ZONE)
      ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
    else
      ch[i] = 0.0;
  }

  mode                = ((double)msg.channels[MODE_CHANNEL] - 1000.0) / 1000.0;
  gear                = ((double)msg.channels[GEAR_CHANNEL] - 1000.0) / 1000.0;
  kill_switch_engaged = (((double)msg.channels[KILL_SWITCH_CHANNEL] - 1000.0) /
                         1000.0) > KILL_SWITCH_THRESHOLD_VALUE;

  valid = check_validity();
  if (!valid)
    return;
  if (kill_switch_engaged)
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RcData"), g_throttle_clock, 10000,
                         "[px4ctrl] Kill switch engaged on RC channel %d.",
                         KILL_SWITCH_CHANNEL + 1);
    return;
  }

  if (!have_init_last_mode)
  {
    have_init_last_mode = true;
    last_mode           = mode;
  }
  if (!have_init_last_gear)
  {
    have_init_last_gear = true;
    last_gear           = gear;
  }

  // 1
  if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
    enter_hover_mode = true;
  else
    enter_hover_mode = false;

  if (mode > API_MODE_THRESHOLD_VALUE)
    is_hover_mode = true;
  else
    is_hover_mode = false;

  // 2
  if (is_hover_mode)
  {
    if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
      enter_command_mode = true;
    else if (gear < GEAR_SHIFT_VALUE)
      enter_command_mode = false;

    if (gear > GEAR_SHIFT_VALUE)
      is_command_mode = true;
    else
      is_command_mode = false;
  }

  last_mode = mode;
  last_gear = gear;
}

bool RcData::check_validity()
{
  for (int idx : {0, 1, 2, 3, MODE_CHANNEL, GEAR_CHANNEL, KILL_SWITCH_CHANNEL})
  {
    if (msg.channels[idx] < RC_PWM_MIN || msg.channels[idx] > RC_PWM_MAX)
    {
      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("RcData"), g_throttle_clock, 1000,
          "RC data validity check fail. channel[%d]=%u outside [%d,%d]", idx,
          static_cast<unsigned>(msg.channels[idx]), RC_PWM_MIN, RC_PWM_MAX);
      return false;
    }
  }

  if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1)
  {
    return true;
  }

  RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RcData"), g_throttle_clock, 1000,
                        "RC data validity check fail. mode=%f, gear=%f", mode,
                        gear);
  return false;
}

bool RcData::check_centered()
{
  bool centered = fabs(ch[0]) < 1e-5 && fabs(ch[1]) < 1e-5 &&
                  fabs(ch[2]) < 1e-5 && fabs(ch[3]) < 1e-5;
  return centered;
}

OdomData::OdomData()
{
  rcv_stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  q.setIdentity();
  recv_new_msg = false;
}

void OdomData::feed(nav_msgs::msg::Odometry::ConstPtr pMsg, bool twist_is_body)
{
  rclcpp::Time now = rclcpp::Clock().now();

  msg          = *pMsg;
  rcv_stamp    = now;
  recv_new_msg = true;

  uav_utils::extract_odometry(pMsg, p, v, q, w);

  // mavros local_position/odom reports twist in the body frame; the
  // controller contract (and /ekf_quat/ekf_odom) is world-frame.
  if (twist_is_body)
  {
    v = q * v;
  }

  // check the frequency
  static int          one_min_count = 9999;
  static rclcpp::Time last_clear_count_time =
      rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  if ((now - last_clear_count_time).seconds() > 1.0)
  {
    if (one_min_count < 100)
    {
      RCLCPP_WARN(rclcpp::get_logger("OdomData"),
                  "ODOM frequency seems lower than 100Hz, which is too low!");
    }
    one_min_count         = 0;
    last_clear_count_time = now;
  }
  one_min_count++;
}

ImuData::ImuData()
{
  rcv_stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void ImuData::feed(sensor_msgs::msg::Imu::ConstPtr pMsg)
{
  rclcpp::Time now = rclcpp::Clock().now();

  msg       = *pMsg;
  rcv_stamp = now;

  w(0) = msg.angular_velocity.x;
  w(1) = msg.angular_velocity.y;
  w(2) = msg.angular_velocity.z;

  a(0) = msg.linear_acceleration.x;
  a(1) = msg.linear_acceleration.y;
  a(2) = msg.linear_acceleration.z;

  q.x() = msg.orientation.x;
  q.y() = msg.orientation.y;
  q.z() = msg.orientation.z;
  q.w() = msg.orientation.w;

  // check the frequency
  static int          one_min_count = 9999;
  static rclcpp::Time last_clear_count_time =
      rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  if ((now - last_clear_count_time).seconds() > 1.0)
  {
    if (one_min_count < 100)
    {
      RCLCPP_WARN(rclcpp::get_logger("ImuData"),
                  "IMU frequency seems lower than 100Hz, which is too low!");
    }
    one_min_count         = 0;
    last_clear_count_time = now;
  }
  one_min_count++;
}

StateData::StateData() {}

void StateData::feed(mavros_msgs::msg::State::ConstPtr pMsg)
{
  current_state = *pMsg;
}

ExtendedStateData::ExtendedStateData() {}

void ExtendedStateData::feed(mavros_msgs::msg::ExtendedState::ConstPtr pMsg)
{
  current_extended_state = *pMsg;
}

CommandData::CommandData()
{
  rcv_stamp        = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  yaw_control_mode = quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TRACK;
}

void CommandData::feed(quadrotor_msgs::msg::PositionCommand::ConstPtr pMsg)
{
  msg       = *pMsg;
  rcv_stamp = rclcpp::Clock().now();

  p(0) = msg.position.x;
  p(1) = msg.position.y;
  p(2) = msg.position.z;

  v(0) = msg.velocity.x;
  v(1) = msg.velocity.y;
  v(2) = msg.velocity.z;

  a(0) = msg.acceleration.x;
  a(1) = msg.acceleration.y;
  a(2) = msg.acceleration.z;

  j(0) = msg.jerk.x;
  j(1) = msg.jerk.y;
  j(2) = msg.jerk.z;

  yaw              = uav_utils::normalize_angle(msg.yaw);
  yaw_rate         = msg.yaw_dot;
  yaw_control_mode = msg.yaw_control_mode;
}

BatteryData::BatteryData()
{
  rcv_stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void BatteryData::feed(sensor_msgs::msg::BatteryState::ConstPtr pMsg)
{
  msg       = *pMsg;
  rcv_stamp = rclcpp::Clock().now();

  double voltage = 0;
  for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
  {
    voltage += pMsg->cell_voltage[i];
  }
  volt = 0.8 * volt +
         0.2 * voltage;  // Naive LPF, cell_voltage has a higher frequency

  percentage = pMsg->percentage;
}

TakeoffLandData::TakeoffLandData()
{
  rcv_stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void TakeoffLandData::feed(quadrotor_msgs::msg::TakeoffLand::ConstPtr pMsg)
{
  msg       = *pMsg;
  rcv_stamp = rclcpp::Clock().now();

  triggered        = true;
  takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
