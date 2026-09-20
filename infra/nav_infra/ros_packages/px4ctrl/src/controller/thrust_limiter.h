#ifndef __CONTROLLER_THRUST_LIMITER_H
#define __CONTROLLER_THRUST_LIMITER_H

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

/**
 * Collective-acceleration magnitude/tilt limiter plus body-rate
 * angular-acceleration limiter. Owns the rate-limit memory so each
 * tracking algorithm instance carries independent limiter state.
 */
class ThrustLimiter
{
 public:
  /**
   * @param[in] max_angle_rad Tilt limit, <=0 disables the angle gate [rad]
   * @param[in] gra Gravitational acceleration [m/s^2]
   */
  ThrustLimiter(double max_angle_rad, double gra);

  /**
   * Limit a thrust-force vector to feasible collective acceleration.
   *
   * @param[in] thrustforce Thrust force in world [N]
   * @param[in] mass Vehicle mass [kg]
   * @return Limited collective acceleration in world [m/s^2]
   */
  Eigen::Vector3d limitFromThrustForce(const Eigen::Vector3d &thrustforce,
                                       double                 mass) const;

  /**
   * Limit the PID + feedforward + drag collective acceleration.
   *
   * @param[in] pid_acc PID feedback acceleration in world [m/s^2]
   * @param[in] ref_acc Feedforward acceleration in world [m/s^2]
   * @param[in] drag_acc Rotor-drag compensation in world [m/s^2]
   * @return Limited collective acceleration in world [m/s^2]
   */
  Eigen::Vector3d limitTotalAcc(
      const Eigen::Vector3d &pid_acc, const Eigen::Vector3d &ref_acc,
      const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

  /**
   * Rate-limit a body-rate command to the angular-acceleration gate.
   *
   * @param[in] candidate_rate Unsaturated body-rate command [rad/s]
   * @return Rate-limited body rates in body [rad/s]
   */
  Eigen::Vector3d limitAngularAcc(const Eigen::Vector3d &candidate_rate);

  /** Drop the rate-limit memory (takeoff / mode switch). */
  void reset();

 private:
  Eigen::Vector3d limitMagnitudeAndAngle(Eigen::Vector3d total_acc) const;

  double          max_angle_;
  Eigen::Vector3d gravity_;
  rclcpp::Time    last_stamp_{rclcpp::Time(0, 0, RCL_SYSTEM_TIME)};
  Eigen::Vector3d last_rate_{Eigen::Vector3d(0, 0, 0)};

  static constexpr double kMaxAngularAcc_ = 60;
};

#endif
