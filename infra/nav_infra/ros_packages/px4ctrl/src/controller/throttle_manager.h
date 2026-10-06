#ifndef __CONTROLLER_THROTTLE_MANAGER_H
#define __CONTROLLER_THROTTLE_MANAGER_H

#include <Eigen/Dense>
#include <interface/msg/throttle_model_status.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "PX4CtrlParam.h"
#include "rls_throttle_model.h"
#include "types.h"

/**
 * Collective-throttle estimation and mapping (legacy thr2acc / accurate
 * curve / RLS eta + INDI). Owns the estimator state shared by all
 * tracking algorithms and the NMPC rates path.
 */
class ThrottleManager
{
 public:
  explicit ThrottleManager(Parameters &param);

  /**
   * Normalized collective thrust from the desired acceleration.
   *
   * @param[in] est_q Estimated attitude (world from body) [-]
   * @param[in] est_v Estimated velocity in world [m/s]
   * @param[in] des_acc Desired collective acceleration in world [m/s^2]
   * @param[in] voltage Battery voltage [V]
   * @param[out] dbg Telemetry (des_thr) to publish [-]
   * @return Normalized throttle [0,1]
   */
  double computeDesiredCollectiveThrustSignal(
      const Eigen::Quaterniond &est_q, const Eigen::Vector3d &est_v,
      const Eigen::Vector3d &des_acc, double voltage,
      quadrotor_msgs::msg::Px4ctrlDebug &dbg);

  /**
   * Voltage-compensated quadratic thrust-curve inversion.
   *
   * @param[in] des_acc_z Desired thrust-axis acceleration [m/s^2]
   * @param[in] voltage Battery voltage [V]
   * @return Normalized throttle [0,1]
   */
  double accurateThrustAccMapping(double des_acc_z, double voltage) const;

  /**
   * Legacy RLS thrust-model update on the delayed thrust queue.
   *
   * @param[in] est_a Estimated acceleration in world [m/s^2]
   * @param[in] voltage Battery voltage [V]
   * @param[in] est_v Estimated velocity in world [m/s]
   * @param[out] dbg Telemetry (hover_percentage, ...) to publish [-]
   * @return True when a delayed sample was consumed [-]
   */
  bool estimateThrustModel(const Eigen::Vector3d &est_a, double voltage,
                           const Eigen::Vector3d             &est_v,
                           quadrotor_msgs::msg::Px4ctrlDebug &dbg);

  /** Reset estimators to the hover seed (takeoff / mode switch). */
  void resetThrustMapping();

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

  /** @return Current throttle-effectiveness estimate [m/s^2] */
  double getEta() const { return rls_.getEta(); }

  /**
   * Record a commanded throttle for the delayed RLS update queue.
   *
   * @param[in] thrust Normalized throttle just sent to the mixer [0,1]
   */
  void pushTimedThrust(double thrust);

 private:
  /**
   * INDI incremental throttle from the current estimator state.
   *
   * @param[in] specific_force_sp Thrust-axis setpoint [m/s^2]
   * @return Normalized throttle command [0,1]
   */
  double computeIndiThrustSignal(double specific_force_sp);

  Parameters &param_;

  // Legacy thrust-accel mapping state
  double                  thr_scale_compensate;
  double                  thr2acc;
  double                  P;
  static constexpr double rho2 = 0.998;  // do not change

  // RLS eta model + INDI incremental thrust state
  RlsThrottleModel rls_;
  FirstOrderLpf    setpoint_lpf_, meas_lpf_, throttle_lpf_;
  double           throttle_memory_{0.0};  // last commanded throttle [0,1]
  double           control_dt_{0.005};     // control period from FSM [s]

  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
};

#endif
