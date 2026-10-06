#ifndef __CONTROLLER_YAW_TARGET_H
#define __CONTROLLER_YAW_TARGET_H

#include "PX4CtrlParam.h"

/**
 * Bounded shortest-error yaw regulator for YAW_CONTROL_TARGET mode.
 *
 * Holds its own gain/deadband/rate-limit copies from Parameters.
 */
class YawTarget
{
 public:
  explicit YawTarget(const Parameters &param);

  /**
   * Yaw body-rate command with deadband and magnitude saturation.
   *
   * @param[in] target_yaw Absolute yaw target [rad]
   * @param[in] current_yaw Estimated yaw [rad]
   * @return Yaw body rate [rad/s]
   */
  double compute(double target_yaw, double current_yaw) const;

 private:
  double kp_;
  double max_rate_;
  double deadband_;
};

#endif
