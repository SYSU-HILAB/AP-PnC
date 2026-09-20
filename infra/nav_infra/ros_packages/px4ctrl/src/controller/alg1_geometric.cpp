#include "alg1_geometric.h"

#include <rclcpp/rclcpp.hpp>

#include "flatness_math.h"
#include "types.h"

Alg1Geometric::Alg1Geometric(Parameters &param, PidPosition &pid,
                             AttitudeFeedback &att_fb, ThrustLimiter &limiter,
                             ThrottleManager &throttle)
    : param_(param),
      pid_(pid),
      att_fb_(att_fb),
      limiter_(limiter),
      throttle_(throttle)
{
}

void Alg1Geometric::update(const DesiredState &des, const OdomData &odom,
                           const ImuData &imu, ControllerOutput &u,
                           double                             voltage,
                           quadrotor_msgs::msg::Px4ctrlDebug &dbg)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    RCLCPP_WARN(rclcpp::get_logger("px4ctrl"),
                "Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous "
                "since the drone will be unstable!",
                des.v(2));

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = pid_.compute(odom, des, dbg);
  Eigen::Vector3d       total_des_acc =
      limiter_.limitTotalAcc(pid_error_accelerations, des.a);

  dbg.fb_a_x  = pid_error_accelerations(0);
  dbg.fb_a_y  = pid_error_accelerations(1);
  dbg.fb_a_z  = pid_error_accelerations(2);
  dbg.des_a_x = total_des_acc(0);
  dbg.des_a_y = total_des_acc(1);
  dbg.des_a_z = total_des_acc(2);

  u.thrust = throttle_.computeDesiredCollectiveThrustSignal(
      odom.q, odom.v, total_des_acc, voltage, dbg);

  Eigen::Quaterniond desired_attitude;
  computeFlatInput(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q,
                   desired_attitude, u.bodyrates);

  const Eigen::Vector3d feedback_bodyrates =
      att_fb_.compute(desired_attitude, odom.q, dbg);

  dbg.des_q_w = desired_attitude.w();
  dbg.des_q_x = desired_attitude.x();
  dbg.des_q_y = desired_attitude.y();
  dbg.des_q_z = desired_attitude.z();

  u.q = imu.q * odom.q.inverse() * desired_attitude;  // Align with FCU frame
  u.bodyrates += feedback_bodyrates;
}

void Alg1Geometric::computeFlatInput(const Eigen::Vector3d &thr_acc,
                                     const Eigen::Vector3d &jer, double yaw,
                                     double                    yawd,
                                     const Eigen::Quaterniond &att_est,
                                     Eigen::Quaterniond       &att,
                                     Eigen::Vector3d          &omg)
{
  if (thr_acc.norm() < kMinNormalizedCollectiveAcc)
  {
    att = att_est;
    omg.setConstant(0.0);
    return;
  }
  else
  {
    Eigen::Vector3d zb, zbd;
    normalizeWithGrad(thr_acc, jer, zb, zbd);
    double          syaw = sin(yaw);
    double          cyaw = cos(yaw);
    Eigen::Vector3d xc(cyaw, syaw, 0.0);
    Eigen::Vector3d xcd(-syaw * yawd, cyaw * yawd, 0.0);
    Eigen::Vector3d yc = zb.cross(xc);
    if (almostZero(yc.norm()))
    {
      RCLCPP_WARN(rclcpp::get_logger("px4ctrl"),
                  "Conor case, pitch is close to 90 deg");
      att = att_est;
      omg = omg_old_;
    }
    else
    {
      Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
      Eigen::Vector3d yb, ybd;
      normalizeWithGrad(yc, ycd, yb, ybd);
      Eigen::Vector3d xb  = yb.cross(zb);
      Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
      omg(0)              = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
      omg(1)              = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
      omg(2)              = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;
      Eigen::Matrix3d rotM;
      rotM << xb, yb, zb;
      att      = Eigen::Quaterniond(rotM);
      omg_old_ = omg;
    }
  }
  return;
}

void Alg1Geometric::reset()
{
  omg_old_.setZero();
}
