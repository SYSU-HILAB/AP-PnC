#ifndef __CONTROLLER_ALG0_H
#define __CONTROLLER_ALG0_H

#include <Eigen/Dense>

#include "PX4CtrlParam.h"
#include "attitude_feedback.h"
#include "pid_position.h"
#include "throttle_manager.h"
#include "thrust_limiter.h"
#include "tracking_controller_base.h"
#include "types.h"

/**
 * Differential-flatness tracking with rotor-drag feedforward (Zhepei Wang
 * alg0). Owns its flatness fallback memory (omg/thrust) independently of
 * the other solvers.
 */
class Alg0DifferentialFlatness : public TrackingControllerBase
{
 public:
  Alg0DifferentialFlatness(Parameters &param, PidPosition &pid,
                           AttitudeFeedback &att_fb, ThrustLimiter &limiter,
                           ThrottleManager &throttle);

  void update(const DesiredState &des, const OdomData &odom, const ImuData &imu,
              ControllerOutput &u, double voltage,
              quadrotor_msgs::msg::Px4ctrlDebug &dbg) override;

  void        reset() override;
  const char *name() const override { return "alg0_differential_flatness"; }

 private:
  /**
   * Drag-aware flatness map from vel/acc/jerk/yaw to thrust/attitude/rates.
   *
   * @param[in] vel Desired velocity in world [m/s]
   * @param[in] acc Desired acceleration in world [m/s^2]
   * @param[in] jer Desired jerk in world [m/s^3]
   * @param[in] psi Desired yaw [rad]
   * @param[in] dpsi Desired yaw rate [rad/s]
   * @param[out] thr Collective thrust [N]
   * @param[out] quat Desired attitude quaternion (w,x,y,z) [-]
   * @param[out] omg Feedforward body rates [rad/s]
   * @param[in] dh Horizontal rotor-drag coefficient [1/s]
   * @param[in] dv Vertical rotor-drag coefficient [1/s]
   * @param[in] cp Second-order drag coefficient [s/m]
   * @param[in] veps Velocity smoothing constant [m/s]
   * @return False on the inverted-flight / free-fall singularity [-]
   */
  bool flatnessWithDrag(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &jer, double psi, double dpsi,
                        double &thr, Eigen::Vector4d &quat,
                        Eigen::Vector3d &omg, double dh, double dv, double cp,
                        double veps) const;

  /**
   * Singularity-guarded wrapper falling back to the last good solution.
   *
   * @param[in] vel Desired velocity in world [m/s]
   * @param[in] acc Desired acceleration in world [m/s^2]
   * @param[in] jer Desired jerk in world [m/s^3]
   * @param[in] yaw Desired yaw [rad]
   * @param[in] yawd Desired yaw rate [rad/s]
   * @param[in] att_est Estimated attitude (world from body) [-]
   * @param[out] att Desired attitude (world from body) [-]
   * @param[out] omg Feedforward body rates [rad/s]
   * @param[out] thrust Collective thrust [N]
   */
  void minimumSingularityFlatWithDrag(const Eigen::Vector3d &vel,
                                      const Eigen::Vector3d &acc,
                                      const Eigen::Vector3d &jer, double yaw,
                                      double                    yawd,
                                      const Eigen::Quaterniond &att_est,
                                      Eigen::Quaterniond       &att,
                                      Eigen::Vector3d &omg, double &thrust);

  Parameters       &param_;
  PidPosition      &pid_;
  AttitudeFeedback &att_fb_;
  ThrustLimiter    &limiter_;
  ThrottleManager  &throttle_;
  Eigen::Vector3d   gravity_;

  // Flatness fallback memory, replaces the former function-static state.
  Eigen::Vector3d omg_old_{Eigen::Vector3d::Zero()};
  double          thrust_old_{0.0};
  bool            have_last_solution_{false};
};

#endif
