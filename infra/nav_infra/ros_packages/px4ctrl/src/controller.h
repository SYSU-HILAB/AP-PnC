/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <Eigen/Dense>
#include <interface/msg/throttle_model_status.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <memory>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include "controller/attitude_feedback.h"
#include "controller/pid_position.h"
#include "controller/throttle_manager.h"
#include "controller/thrust_limiter.h"
#include "controller/tracking_controller_base.h"
#include "controller/types.h"
#include "controller/yaw_target.h"

/**
 * Tracking facade: owns one pose_solver strategy plus the shared
 * PID/attitude/limiter/throttle modules. FSM calls update() only.
 */
class Controller
{
 public:
  explicit Controller(Parameters &param);

  /**
   * One position-control step via the configured pose_solver strategy.
   *
   * @param[in] des Desired state (position/velocity/accel/jerk/yaw) [-]
   * @param[in] odom Current odometry (world frame) [-]
   * @param[in] imu Current IMU (attitude/gyro/accel) [-]
   * @param[out] u Body-rate + thrust command (FCU frame conv. by solver) [-]
   * @param[in] voltage Battery voltage [V]
   * @return Debug telemetry to publish [-]
   */
  quadrotor_msgs::msg::Px4ctrlDebug update(const DesiredState &des,
                                           const OdomData     &odom,
                                           const ImuData      &imu,
                                           ControllerOutput &u, double voltage);

  /**
   * Yaw body-rate command for YAW_CONTROL_TARGET mode [rad/s].
   *
   * @param[in] target_yaw Absolute yaw target [rad]
   * @param[in] current_yaw Estimated yaw [rad]
   * @return Yaw body rate [rad/s]
   */
  double computeYawTargetBodyRate(double target_yaw, double current_yaw) const;

  /**
   * Attitude-error to body-rate feedback for tests [rad/s].
   *
   * @param[in] des_q Desired attitude (world from body) [-]
   * @param[in] est_q Estimated attitude (world from body) [-]
   * @return Feedback body rates in body [rad/s]
   */
  Eigen::Vector3d computeFeedBackControlBodyrates(
      const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q);

  /** Reset thrust estimators and solver memory (takeoff / mode switch). */
  void resetThrustMapping(void);

  /**
   * Publish the control period for the discrete INDI law and RLS LPF.
   *
   * @param[in] dt Control period [s]
   */
  void setControlDt(double dt);

  /**
   * Step the RLS eta estimator on the thrust-axis specific force.
   *
   * @param[in] specific_force Measured body-z specific force [m/s^2]
   * @return True once the eta variance has converged [-]
   */
  bool updateIndiThrottleModel(double specific_force);

  /**
   * NMPC path: estimate then produce the INDI throttle command.
   *
   * @param[in] specific_force Measured body-z specific force [m/s^2]
   * @param[in] specific_force_sp NMPC thrust-axis setpoint [m/s^2]
   * @return Normalized throttle command [0,1]
   */
  double computeNmpcThrottle(double specific_force, double specific_force_sp);

  /**
   * Fill the RLS/INDI telemetry message.
   *
   * @param[out] status Throttle model status to populate [-]
   */
  void fillThrottleStatus(interface::msg::ThrottleModelStatus &status) const;

  /**
   * Legacy RLS thrust-model update on the delayed thrust queue.
   *
   * @param[in] est_a Estimated acceleration in world [m/s^2]
   * @param[in] voltage Battery voltage [V]
   * @param[in] est_v Estimated velocity in world [m/s]
   * @return True when a delayed sample was consumed [-]
   */
  bool estimateThrustModel(const Eigen::Vector3d &est_a, double voltage,
                           const Eigen::Vector3d &est_v);

 private:
  /**
   * Build the solver matching param.pose_solver (0/1/2).
   *
   * @return Owned tracking strategy [-]
   */
  std::unique_ptr<TrackingControllerBase> makeSolver();

  Parameters                             &param_;
  PidPosition                             pid_;
  AttitudeFeedback                        att_fb_;
  ThrustLimiter                           limiter_;
  ThrottleManager                         throttle_;
  YawTarget                               yaw_target_;
  quadrotor_msgs::msg::Px4ctrlDebug       debug_;  // scratch telemetry buffer
  std::unique_ptr<TrackingControllerBase> solver_;
  int                                     solver_id_{-1};
};

#endif
