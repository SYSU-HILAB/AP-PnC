#ifndef __CONTROLLER_TRACKING_BASE_H
#define __CONTROLLER_TRACKING_BASE_H

#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include "types.h"

/**
 * Position-tracking solver interface: one implementation per
 * pose_solver mode, each owning independent gains and filter state.
 */
class TrackingControllerBase
{
 public:
  virtual ~TrackingControllerBase() = default;

  /**
   * One position-control step producing body-rate + thrust commands.
   *
   * @param[in] des Desired state (position/velocity/accel/jerk/yaw) [-]
   * @param[in] odom Current odometry (world frame) [-]
   * @param[in] imu Current IMU (attitude/gyro/accel) [-]
   * @param[out] u Body-rate + thrust command (FCU frame conv. by caller) [-]
   * @param[in] voltage Battery voltage [V]
   * @param[out] dbg Debug telemetry accumulator; fields the solver does
   *            not write (e.g. hover_percentage) are preserved [-]
   */
  virtual void update(const DesiredState &des, const OdomData &odom,
                      const ImuData &imu, ControllerOutput &u, double voltage,
                      quadrotor_msgs::msg::Px4ctrlDebug &dbg) = 0;

  /** Drop integrator/filter memory (takeoff / mode switch). */
  virtual void reset() = 0;

  /** Solver name for logs [-] */
  virtual const char *name() const = 0;
};

#endif
