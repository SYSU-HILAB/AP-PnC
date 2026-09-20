#ifndef __CONTROLLER_ATTITUDE_FEEDBACK_H
#define __CONTROLLER_ATTITUDE_FEEDBACK_H

#include <Eigen/Dense>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include "PX4CtrlParam.h"

/**
 * Quaternion-error to body-rate feedback (2*KAng*q_e_xyz, shortest path).
 *
 * Holds its own KAng copy; output saturates at 4 rad/s per axis.
 */
class AttitudeFeedback
{
 public:
  explicit AttitudeFeedback(const Parameters &param);

  /**
   * Body-rate feedback from the attitude error [rad/s].
   *
   * @param[in] des_q Desired attitude (world from body) [-]
   * @param[in] est_q Estimated attitude (world from body) [-]
   * @param[out] dbg Telemetry (exec_err_*, fb_rate_*) to publish [-]
   * @return Feedback body rates in body [rad/s]
   */
  Eigen::Vector3d compute(const Eigen::Quaterniond          &des_q,
                          const Eigen::Quaterniond          &est_q,
                          quadrotor_msgs::msg::Px4ctrlDebug &dbg) const;

 private:
  Eigen::Vector3d KAng_;

  static constexpr double kMaxBodyratesFeedback_ = 4;
};

#endif
