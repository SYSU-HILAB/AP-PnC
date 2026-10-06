#ifndef __CONTROLLER_TYPES_H
#define __CONTROLLER_TYPES_H

#include <Eigen/Dense>
#include <quadrotor_msgs/msg/position_command.hpp>

#include "input.h"

/** Minimum normalized collective acceleration, avoids free-fall singularity
 * [m/s^2]. */
inline constexpr double kMinNormalizedCollectiveAcc = 3.0;

struct DesiredState
{
  Eigen::Vector3d    p;
  Eigen::Vector3d    v;
  Eigen::Vector3d    a;
  Eigen::Vector3d    j;
  Eigen::Quaterniond q;
  double             yaw;
  double             yaw_rate;
  uint8_t            yaw_control_mode{
      quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TRACK};

  DesiredState() {};

  DesiredState(OdomData &odom)
      : p(odom.p),
        v(Eigen::Vector3d::Zero()),
        a(Eigen::Vector3d::Zero()),
        j(Eigen::Vector3d::Zero()),
        q(odom.q),
        yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
        yaw_rate(0),
        yaw_control_mode(
            quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TRACK) {};
};

struct ControllerOutput
{
  // Orientation of the body frame with respect to the world frame
  Eigen::Quaterniond q;

  // Body rates in body frame
  Eigen::Vector3d bodyrates;  // [rad/s]

  // Collective mass normalized thrust
  double thrust;

  // Eigen::Vector3d des_v_real;
};

#endif
