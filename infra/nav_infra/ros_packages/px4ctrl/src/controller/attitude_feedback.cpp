#include "attitude_feedback.h"

AttitudeFeedback::AttitudeFeedback(const Parameters &param)
{
  KAng_(0) = param.gain.KAngR;
  KAng_(1) = param.gain.KAngP;
  KAng_(2) = param.gain.KAngY;
}

Eigen::Vector3d AttitudeFeedback::compute(
    const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q,
    quadrotor_msgs::msg::Px4ctrlDebug &dbg) const
{
  // Compute the error quaternion
  const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

  Eigen::AngleAxisd rotation_vector(q_e);
  Eigen::Vector3d   axis   = rotation_vector.axis();
  dbg.exec_err_axisang_x   = axis(0);
  dbg.exec_err_axisang_y   = axis(1);
  dbg.exec_err_axisang_z   = axis(2);
  dbg.exec_err_axisang_ang = rotation_vector.angle();

  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;

  if (q_e.w() >= 0)
  {
    bodyrates.x() = 2.0 * KAng_(0) * q_e.x();
    bodyrates.y() = 2.0 * KAng_(1) * q_e.y();
    bodyrates.z() = 2.0 * KAng_(2) * q_e.z();
  }
  else
  {
    bodyrates.x() = -2.0 * KAng_(0) * q_e.x();
    bodyrates.y() = -2.0 * KAng_(1) * q_e.y();
    bodyrates.z() = -2.0 * KAng_(2) * q_e.z();
  }

  if (bodyrates.x() > kMaxBodyratesFeedback_)
    bodyrates.x() = kMaxBodyratesFeedback_;
  if (bodyrates.x() < -kMaxBodyratesFeedback_)
    bodyrates.x() = -kMaxBodyratesFeedback_;
  if (bodyrates.y() > kMaxBodyratesFeedback_)
    bodyrates.y() = kMaxBodyratesFeedback_;
  if (bodyrates.y() < -kMaxBodyratesFeedback_)
    bodyrates.y() = -kMaxBodyratesFeedback_;
  if (bodyrates.z() > kMaxBodyratesFeedback_)
    bodyrates.z() = kMaxBodyratesFeedback_;
  if (bodyrates.z() < -kMaxBodyratesFeedback_)
    bodyrates.z() = -kMaxBodyratesFeedback_;

  dbg.fb_rate_x = bodyrates.x();
  dbg.fb_rate_y = bodyrates.y();
  dbg.fb_rate_z = bodyrates.z();

  return bodyrates;
}
