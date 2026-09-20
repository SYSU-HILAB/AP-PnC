#include "thrust_limiter.h"

#include "types.h"

ThrustLimiter::ThrustLimiter(double max_angle_rad, double gra)
    : max_angle_(max_angle_rad), gravity_(0.0, 0.0, -gra)
{
}

Eigen::Vector3d ThrustLimiter::limitFromThrustForce(
    const Eigen::Vector3d &thrustforce, double mass) const
{
  return limitMagnitudeAndAngle(thrustforce / mass);
}

Eigen::Vector3d ThrustLimiter::limitTotalAcc(
    const Eigen::Vector3d &pid_acc, const Eigen::Vector3d &ref_acc,
    const Eigen::Vector3d &drag_acc) const
{
  Eigen::Vector3d total_acc = pid_acc + ref_acc - gravity_ - drag_acc;
  return limitMagnitudeAndAngle(total_acc);
}

Eigen::Vector3d ThrustLimiter::limitMagnitudeAndAngle(
    Eigen::Vector3d total_acc) const
{
  // Limit magnitude
  if (total_acc.norm() < kMinNormalizedCollectiveAcc)
  {
    total_acc = total_acc.normalized() * kMinNormalizedCollectiveAcc;
  }

  // Limit angle
  if (max_angle_ > 0)
  {
    double          z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d z_B   = total_acc.normalized();
    if (z_acc < kMinNormalizedCollectiveAcc)
    {
      z_acc = kMinNormalizedCollectiveAcc;  // Not allow too small z-force when
                                            // angle limit is enabled.
    }
    Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
    double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
    if (rot_ang > max_angle_)  // Exceed the angle limit
    {
      Eigen::Vector3d limited_z_B =
          Eigen::AngleAxisd(max_angle_, rot_axis) * Eigen::Vector3d::UnitZ();
      total_acc = z_acc / std::cos(max_angle_) * limited_z_B;
    }
  }

  return total_acc;
}

Eigen::Vector3d ThrustLimiter::limitAngularAcc(
    const Eigen::Vector3d &candidate_rate)
{
  rclcpp::Time t_now = rclcpp::Clock().now();
  if (last_stamp_ != rclcpp::Time(0, 0, RCL_SYSTEM_TIME))
  {
    double          dura           = (t_now - last_stamp_).seconds();
    double          max_delta_rate = kMaxAngularAcc_ * dura;
    Eigen::Vector3d rate_out;

    if ((candidate_rate(0) - last_rate_(0)) > max_delta_rate)
    {
      rate_out(0) = last_rate_(0) + max_delta_rate;
    }
    else if ((candidate_rate(0) - last_rate_(0)) < -max_delta_rate)
    {
      rate_out(0) = last_rate_(0) - max_delta_rate;
    }
    else
    {
      rate_out(0) = candidate_rate(0);
    }

    if ((candidate_rate(1) - last_rate_(1)) > max_delta_rate)
    {
      rate_out(1) = last_rate_(1) + max_delta_rate;
    }
    else if ((candidate_rate(1) - last_rate_(1)) < -max_delta_rate)
    {
      rate_out(1) = last_rate_(1) - max_delta_rate;
    }
    else
    {
      rate_out(1) = candidate_rate(1);
    }

    if ((candidate_rate(2) - last_rate_(2)) > max_delta_rate)
    {
      rate_out(2) = last_rate_(2) + max_delta_rate;
    }
    else if ((candidate_rate(2) - last_rate_(2)) < -max_delta_rate)
    {
      rate_out(2) = last_rate_(2) - max_delta_rate;
    }
    else
    {
      rate_out(2) = candidate_rate(2);
    }

    last_stamp_ = t_now;
    last_rate_  = rate_out;

    return rate_out;
  }
  else
  {
    last_stamp_ = t_now;
    last_rate_  = candidate_rate;
    return candidate_rate;
  }
}

void ThrustLimiter::reset()
{
  last_stamp_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  last_rate_.setZero();
}
