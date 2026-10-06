#include "pid_position.h"

PidPosition::PidPosition(const Parameters &param)
{
  Kp_(0) = param.gain.Kp0;
  Kp_(1) = param.gain.Kp1;
  Kp_(2) = param.gain.Kp2;
  Kv_(0) = param.gain.Kv0;
  Kv_(1) = param.gain.Kv1;
  Kv_(2) = param.gain.Kv2;
}

Eigen::Vector3d PidPosition::compute(
    const OdomData &odom, const DesiredState &des,
    quadrotor_msgs::msg::Px4ctrlDebug &dbg) const
{
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error =
      std::isnan(des.p(0))
          ? 0.0
          : std::max(std::min(des.p(0) - odom.p(0), 1.0), -1.0);
  double x_vel_error = std::max(
      std::min((des.v(0) + Kp_(0) * x_pos_error) - odom.v(0), 1.0), -1.0);
  acc_error(0) = Kv_(0) * x_vel_error;

  // y acceleration
  double y_pos_error =
      std::isnan(des.p(1))
          ? 0.0
          : std::max(std::min(des.p(1) - odom.p(1), 1.0), -1.0);
  double y_vel_error = std::max(
      std::min((des.v(1) + Kp_(1) * y_pos_error) - odom.v(1), 1.0), -1.0);
  acc_error(1) = Kv_(1) * y_vel_error;

  // z acceleration
  double z_pos_error =
      std::isnan(des.p(2))
          ? 0.0
          : std::max(std::min(des.p(2) - odom.p(2), 1.0), -1.0);
  double z_vel_error = std::max(
      std::min((des.v(2) + Kp_(2) * z_pos_error) - odom.v(2), 1.0), -1.0);
  acc_error(2) = Kv_(2) * z_vel_error;

  dbg.des_v_x = (des.v(0) + Kp_(0) * x_pos_error);
  dbg.des_v_y = (des.v(1) + Kp_(1) * y_pos_error);
  dbg.des_v_z = (des.v(2) + Kp_(2) * z_pos_error);

  return acc_error;
}
