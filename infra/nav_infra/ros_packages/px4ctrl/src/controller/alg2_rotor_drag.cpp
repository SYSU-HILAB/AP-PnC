#include "alg2_rotor_drag.h"

#include <rclcpp/rclcpp.hpp>

#include "flatness_math.h"

Alg2RotorDrag::Alg2RotorDrag(Parameters &param, PidPosition &pid,
                             AttitudeFeedback &att_fb, ThrustLimiter &limiter,
                             ThrottleManager &throttle)
    : param_(param),
      pid_(pid),
      att_fb_(att_fb),
      limiter_(limiter),
      throttle_(throttle),
      gravity_(0.0, 0.0, -param.gra)
{
}

void Alg2RotorDrag::update(const DesiredState &des, const OdomData &odom,
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

  // Compute reference inputs that compensate for aerodynamic drag
  Eigen::Vector3d drag_acc = Eigen::Vector3d::Zero();
  computeAeroCompensatedReferenceInputs(des, odom, &u, &drag_acc);

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = pid_.compute(odom, des, dbg);
  Eigen::Vector3d       total_des_acc =
      limiter_.limitTotalAcc(pid_error_accelerations, des.a, drag_acc);

  u.thrust = throttle_.computeDesiredCollectiveThrustSignal(
      odom.q, odom.v, total_des_acc, voltage, dbg);

  const Eigen::Quaterniond desired_attitude =
      computeDesiredAttitude(total_des_acc, des.yaw, odom.q);
  const Eigen::Vector3d feedback_bodyrates =
      att_fb_.compute(desired_attitude, odom.q, dbg);

  if (param_.enable_body_rate_ctrl)
  {
    u.bodyrates += feedback_bodyrates;
  }
  else
  {
    u.q = imu.q * odom.q.inverse() * desired_attitude;  // Align with FCU frame
  }
}

void Alg2RotorDrag::computeAeroCompensatedReferenceInputs(
    const DesiredState &des, const OdomData &odom, ControllerOutput *outputs,
    Eigen::Vector3d *drag_acc) const
{
  const double dx = param_.rt_drag.x;
  const double dy = param_.rt_drag.y;
  const double dz = param_.rt_drag.z;

  const Eigen::Quaterniond q_heading =
      Eigen::Quaterniond(Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d alpha = des.a - gravity_ + dx * des.v;
  const Eigen::Vector3d beta  = des.a - gravity_ + dy * des.v;
  const Eigen::Vector3d gamma = des.a - gravity_ + dz * des.v;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, odom.q);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm()))
  {
    const Eigen::Vector3d z_B_estimated = odom.q * Eigen::Vector3d::UnitZ();
    y_B                                 = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm()))
    {
      y_B = y_C;
    }
    else
    {
      y_B.normalize();
    }
  }
  else
  {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  outputs->q = Eigen::Quaterniond(R_W_B_ref);

  // Reference thrust
  outputs->thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates
  const double B1 = outputs->thrust - (dz - dx) * z_B.dot(des.v);
  const double C1 = -(dx - dy) * y_B.dot(des.v);
  const double D1 = x_B.dot(des.j) + dx * x_B.dot(des.a);
  const double A2 = outputs->thrust + (dy - dz) * z_B.dot(des.v);
  const double C2 = (dx - dy) * x_B.dot(des.v);
  const double D2 = -y_B.dot(des.j) - dy * y_B.dot(des.a);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = des.yaw_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator))
  {
    outputs->bodyrates = Eigen::Vector3d::Zero();
  }
  else
  {
    // Compute body rates
    if (almostZero(A2))
    {
      outputs->bodyrates.x() = 0.0;
    }
    else
    {
      outputs->bodyrates.x() =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
    outputs->bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
    outputs->bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;
  }

  // Transform reference rates and derivatives into estimated body frame
  const Eigen::Matrix3d R_trans =
      odom.q.toRotationMatrix().transpose() * R_W_B_ref;
  const Eigen::Vector3d bodyrates_ref = outputs->bodyrates;

  outputs->bodyrates = R_trans * bodyrates_ref;

  // Drag accelerations
  *drag_acc = -1.0 * (R_W_B_ref * (D * (R_W_B_ref.transpose() * des.v)));
}

Eigen::Quaterniond Alg2RotorDrag::computeDesiredAttitude(
    const Eigen::Vector3d &des_acc, double reference_heading,
    const Eigen::Quaterniond &est_q) const
{
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  Eigen::Vector3d z_B;
  if (almostZero(des_acc.norm()))
  {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = est_q * Eigen::Vector3d::UnitZ();
  }
  else
  {
    z_B = des_acc.normalized();
  }

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, est_q);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

void Alg2RotorDrag::reset()
{
  // Stateless: nothing to clear.
}
