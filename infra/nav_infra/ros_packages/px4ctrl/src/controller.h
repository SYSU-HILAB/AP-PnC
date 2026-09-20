/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <Eigen/Dense>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <queue>

#include "input.h"
#include "rls_throttle_model.h"

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

class Controller
{
 public:
  Parameters &param;

  Eigen::Vector3d                             Kp;
  Eigen::Vector3d                             Kv;
  Eigen::Vector3d                             Kvi;
  Eigen::Vector3d                             Kvd;
  Eigen::Vector3d                             KAng;
  Eigen::Vector3d                             int_e_v;
  Eigen::Vector3d                             Gravity;
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust;
  rclcpp::Time    last_ctrl_timestamp_{rclcpp::Time(0, 0, RCL_SYSTEM_TIME)};
  Eigen::Vector3d last_bodyrate_{Eigen::Vector3d(0, 0, 0)};
  quadrotor_msgs::msg::Px4ctrlDebug debug;  // debug

  // Thrust-accel mapping params
  double       thr_scale_compensate;
  const double rho2 = 0.998;  // do not change
  double       thr2acc;
  double       P;

  // Throttle estimator: RLS eta model + INDI incremental thrust state
  RlsThrottleModel rls_;
  FirstOrderLpf    setpoint_lpf_, meas_lpf_, throttle_lpf_;
  double           throttle_memory_{0.0};  // last commanded normalized throttle
  double           control_dt_{0.005};     // control period fed by the FSM [s]

  Controller(Parameters &);
  double computeYawTargetBodyRate(double target_yaw, double current_yaw) const;

  /* Algorithm0 from  Zhepei Wang*/
  quadrotor_msgs::msg::Px4ctrlDebug update_alg0(const DesiredState &des,
                                                const OdomData     &odom,
                                                const ImuData      &imu,
                                                ControllerOutput   &u,
                                                double              voltage);

  Eigen::Vector3d computeLimitedTotalAccFromThrustForce(
      const Eigen::Vector3d &thrustforce, const double &mass) const;

  bool flatnessWithDrag(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &jer, const double &psi,
                        const double &dpsi, double &thr, Eigen::Vector4d &quat,
                        Eigen::Vector3d &omg, const double &mass,
                        const double &grav, const double &dh, const double &dv,
                        const double &cp, const double &veps) const;

  void minimumSingularityFlatWithDrag(
      const double mass, const double grav, const Eigen::Vector3d &vel,
      const Eigen::Vector3d &acc, const Eigen::Vector3d &jer, const double &yaw,
      const double &yawd, const Eigen::Quaterniond &att_est,
      Eigen::Quaterniond &att, Eigen::Vector3d &omg, double &thrust) const;

  /* Algorithm1 from  Zhepei Wang*/
  quadrotor_msgs::msg::Px4ctrlDebug update_alg1(const DesiredState &des,
                                                const OdomData     &odom,
                                                const ImuData      &imu,
                                                ControllerOutput   &u,
                                                double              voltage);

  void normalizeWithGrad(const Eigen::Vector3d &x, const Eigen::Vector3d &xd,
                         Eigen::Vector3d &xNor, Eigen::Vector3d &xNord) const;

  void computeFlatInput(const Eigen::Vector3d &thr_acc,
                        const Eigen::Vector3d &jer, const double &yaw,
                        const double &yawd, const Eigen::Quaterniond &att_est,
                        Eigen::Quaterniond &att, Eigen::Vector3d &omg) const;

  /*Algorithm from the rotor-drag paper*/
  void update_alg2(const DesiredState &des, const OdomData &odom,
                   const ImuData &imu, ControllerOutput &u, double voltage);

  void computeAeroCompensatedReferenceInputs(const DesiredState &des,
                                             const OdomData     &odom,
                                             const Parameters   &param,
                                             ControllerOutput   *u,
                                             Eigen::Vector3d *drag_acc) const;

  Eigen::Quaterniond computeDesiredAttitude(
      const Eigen::Vector3d &des_acc, const double reference_heading,
      const Eigen::Quaterniond &est_q) const;

  Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype,
                                         const Eigen::Vector3d &x_C,
                                         const Eigen::Vector3d &y_C,
                                         const Eigen::Quaterniond &est_q) const;

  /* Shared functions*/
  Eigen::Vector3d computePIDErrorAcc(const OdomData     &odom,
                                     const DesiredState &des,
                                     const Parameters   &param);

  Eigen::Vector3d computeLimitedTotalAcc(
      const Eigen::Vector3d &PIDErrorAcc, const Eigen::Vector3d &ref_acc,
      const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

  Eigen::Vector3d computeLimitedAngularAcc(
      const Eigen::Vector3d candirate_bodyrate);

  double computeDesiredCollectiveThrustSignal(const Eigen::Quaterniond &est_q,
                                              const Eigen::Vector3d    &est_v,
                                              const Eigen::Vector3d    &des_acc,
                                              const Parameters         &param,
                                              double voltage);

  double AccurateThrustAccMapping(const double des_acc_z, double voltage,
                                  const Parameters &param) const;

  Eigen::Vector3d computeFeedBackControlBodyrates(
      const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q,
      const Parameters &param);

  bool estimateThrustModel(const Eigen::Vector3d &est_a, const double voltage,
                           const Eigen::Vector3d &est_v,
                           const Parameters      &param);

  bool almostZero(const double value) const;

  bool almostZeroThrust(const double thrust_value) const;

  void resetThrustMapping(void);

  /**
   * Publish the control period for the discrete INDI law and RLS LPF.
   *
   * @param[in] dt Control period [s]
   */
  void setControlDt(double dt);

  /**
   * Step the RLS eta estimator on the latest thrust-axis specific force
   * (tracking `throttle_estimator=1` path).
   *
   * @param[in] specific_force Measured body-z specific force [m/s^2]
   * @return True once the eta variance has converged [-]
   */
  bool updateIndiThrottleModel(double specific_force);

  /**
   * NMPC controller mode: step the estimator and produce the INDI
   * throttle command for one NMPC control packet.
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

 private:
  /**
   * INDI incremental throttle from the current estimator state.
   *
   * @param[in] specific_force_sp Thrust-axis setpoint [m/s^2]
   * @return Normalized throttle command [0,1]
   */
  double computeIndiThrustSignal(double specific_force_sp);

  static constexpr double kMinNormalizedCollectiveAcc_ = 3;
  static constexpr double kAlmostZeroValueThreshold_   = 0.001;
  static constexpr double kAlmostZeroThrustThreshold_  = 0.01;
  static constexpr double kMaxBodyratesFeedback_       = 4;
  static constexpr double kMaxAngularAcc_              = 60;
};

#endif
