#ifndef __CONTROLLER_ALG2_H
#define __CONTROLLER_ALG2_H

#include <Eigen/Dense>

#include "PX4CtrlParam.h"
#include "attitude_feedback.h"
#include "pid_position.h"
#include "throttle_manager.h"
#include "thrust_limiter.h"
#include "tracking_controller_base.h"
#include "types.h"

/**
 * Rotor-drag-compensated tracking (rotor-drag paper). Stateless today;
 * keeps the solver interface so drag state has a home if added later.
 */
class Alg2RotorDrag : public TrackingControllerBase
{
 public:
  Alg2RotorDrag(Parameters &param, PidPosition &pid, AttitudeFeedback &att_fb,
                ThrustLimiter &limiter, ThrottleManager &throttle);

  void update(const DesiredState &des, const OdomData &odom, const ImuData &imu,
              ControllerOutput &u, double voltage,
              quadrotor_msgs::msg::Px4ctrlDebug &dbg) override;

  void        reset() override;
  const char *name() const override { return "alg2_rotor_drag"; }

 private:
  /**
   * Drag-compensated reference attitude/rates plus drag acceleration.
   *
   * @param[in] des Desired state (accel/vel/jerk/yaw feedforward) [-]
   * @param[in] odom Current odometry (world frame) [-]
   * @param[out] u Reference attitude/thrust/rates (body frame conv.) [-]
   * @param[out] drag_acc Rotor-drag acceleration in world [m/s^2]
   */
  void computeAeroCompensatedReferenceInputs(const DesiredState &des,
                                             const OdomData     &odom,
                                             ControllerOutput   *u,
                                             Eigen::Vector3d *drag_acc) const;

  /**
   * Desired attitude aligning body-z with the desired acceleration.
   *
   * @param[in] des_acc Desired collective acceleration in world [m/s^2]
   * @param[in] reference_heading Desired yaw [rad]
   * @param[in] est_q Estimated attitude (world from body) [-]
   * @return Desired attitude (world from body) [-]
   */
  Eigen::Quaterniond computeDesiredAttitude(
      const Eigen::Vector3d &des_acc, double reference_heading,
      const Eigen::Quaterniond &est_q) const;

  Parameters       &param_;
  PidPosition      &pid_;
  AttitudeFeedback &att_fb_;
  ThrustLimiter    &limiter_;
  ThrottleManager  &throttle_;
  Eigen::Vector3d   gravity_;
};

#endif
