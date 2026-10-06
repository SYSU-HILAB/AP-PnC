#include "yaw_target.h"

#include <algorithm>
#include <cmath>

YawTarget::YawTarget(const Parameters &param)
    : kp_(param.yaw_target_kp),
      max_rate_(param.max_yaw_target_mode_rate_max),
      deadband_(param.yaw_target_deadband)
{
}

double YawTarget::compute(double target_yaw, double current_yaw) const
{
  const double yaw_error = std::atan2(std::sin(target_yaw - current_yaw),
                                      std::cos(target_yaw - current_yaw));
  if (std::abs(yaw_error) <= deadband_)
    return 0.0;
  return std::max(-max_rate_, std::min(max_rate_, kp_ * yaw_error));
}
