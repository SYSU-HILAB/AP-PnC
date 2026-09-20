#ifndef __CONTROLLER_ALG1_H
#define __CONTROLLER_ALG1_H

#include <Eigen/Dense>

#include "PX4CtrlParam.h"
#include "attitude_feedback.h"
#include "pid_position.h"
#include "throttle_manager.h"
#include "thrust_limiter.h"
#include "tracking_controller_base.h"
#include "types.h"

/**
 * Geometric tracking on SE(3) (Zhepei Wang alg1). Owns its flatness
 * fallback memory (omg) independently of the other solvers.
 */
class Alg1Geometric : public TrackingControllerBase
{
 public:
  Alg1Geometric(Parameters &param, PidPosition &pid, AttitudeFeedback &att_fb,
                ThrustLimiter &limiter, ThrottleManager &throttle);

  void update(const DesiredState &des, const OdomData &odom, const ImuData &imu,
              ControllerOutput &u, double voltage,
              quadrotor_msgs::msg::Px4ctrlDebug &dbg) override;

  void        reset() override;
  const char *name() const override { return "alg1_geometric"; }

 private:
  /**
   * Flatness attitude/rates from the collective-acceleration vector.
   *
   * @param[in] thr_acc Desired collective acceleration in world [m/s^2]
   * @param[in] jer Desired jerk in world [m/s^3]
   * @param[in] yaw Desired yaw [rad]
   * @param[in] yawd Desired yaw rate [rad/s]
   * @param[in] att_est Estimated attitude (world from body) [-]
   * @param[out] att Desired attitude (world from body) [-]
   * @param[out] omg Feedforward body rates [rad/s]
   */
  void computeFlatInput(const Eigen::Vector3d &thr_acc,
                        const Eigen::Vector3d &jer, double yaw, double yawd,
                        const Eigen::Quaterniond &att_est,
                        Eigen::Quaterniond &att, Eigen::Vector3d &omg);

  Parameters       &param_;
  PidPosition      &pid_;
  AttitudeFeedback &att_fb_;
  ThrustLimiter    &limiter_;
  ThrottleManager  &throttle_;

  // Flatness fallback memory, replaces the former function-static state.
  Eigen::Vector3d omg_old_{Eigen::Vector3d::Zero()};
};

#endif
