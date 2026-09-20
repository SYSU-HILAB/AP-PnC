#include "alg0_differential_flatness.h"

#include <rclcpp/rclcpp.hpp>

#include "types.h"

Alg0DifferentialFlatness::Alg0DifferentialFlatness(Parameters       &param,
                                                   PidPosition      &pid,
                                                   AttitudeFeedback &att_fb,
                                                   ThrustLimiter    &limiter,
                                                   ThrottleManager  &throttle)
    : param_(param),
      pid_(pid),
      att_fb_(att_fb),
      limiter_(limiter),
      throttle_(throttle),
      gravity_(0.0, 0.0, -param.gra)
{
}

void Alg0DifferentialFlatness::update(const DesiredState &des,
                                      const OdomData &odom, const ImuData &imu,
                                      ControllerOutput &u, double voltage,
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
  Eigen::Vector3d       translational_acc = pid_error_accelerations + des.a;
  Eigen::Quaterniond    desired_attitude, idel_att;
  Eigen::Vector3d       omega;
  double                thrust, debug_thrust;
  translational_acc = (gravity_ + limiter_.limitFromThrustForce(
                                      translational_acc - gravity_, 1.0))
                          .eval();

  // wmywmy
  minimumSingularityFlatWithDrag(des.v, des.a, des.j, des.yaw, des.yaw_rate,
                                 odom.q, idel_att, omega, debug_thrust);
  // wmywmy

  minimumSingularityFlatWithDrag(des.v, translational_acc, des.j, des.yaw,
                                 des.yaw_rate, odom.q, desired_attitude,
                                 u.bodyrates, thrust);

  Eigen::Vector3d thrustforce =
      desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d total_des_acc =
      limiter_.limitFromThrustForce(thrustforce, param_.mass);

  u.thrust = throttle_.computeDesiredCollectiveThrustSignal(
      odom.q, odom.v, total_des_acc, voltage, dbg);

  const Eigen::Vector3d feedback_bodyrates =
      att_fb_.compute(desired_attitude, odom.q, dbg);

  // Compute the error quaternion wmywmy
  const Eigen::Quaterniond q_e = idel_att.inverse() * desired_attitude;

  Eigen::AngleAxisd rotation_vector(q_e);
  Eigen::Vector3d   axis = rotation_vector.axis();
  dbg.fb_axisang_x       = axis(0);
  dbg.fb_axisang_y       = axis(1);
  dbg.fb_axisang_z       = axis(2);
  dbg.fb_axisang_ang     = rotation_vector.angle();
  // wmywmy

  dbg.fb_a_x  = pid_error_accelerations(0);
  dbg.fb_a_y  = pid_error_accelerations(1);
  dbg.fb_a_z  = pid_error_accelerations(2);
  dbg.des_a_x = total_des_acc(0);
  dbg.des_a_y = total_des_acc(1);
  dbg.des_a_z = total_des_acc(2);
  dbg.des_q_w = desired_attitude.w();
  dbg.des_q_x = desired_attitude.x();
  dbg.des_q_y = desired_attitude.y();
  dbg.des_q_z = desired_attitude.z();

  u.q = imu.q * odom.q.inverse() * desired_attitude;  // Align with FCU frame
  const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

  // limit the angular acceleration
  u.bodyrates = limiter_.limitAngularAcc(bodyrate_candidate);
}

bool Alg0DifferentialFlatness::flatnessWithDrag(
    const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
    const Eigen::Vector3d &jer, double psi, double dpsi, double &thr,
    Eigen::Vector4d &quat, Eigen::Vector3d &omg, double dh, double dv,
    double cp, double veps) const
{
  const double almost_zero = 1.0e-6;

  double w0, w1, w2, dw0, dw1, dw2;
  double v0, v1, v2, a0, a1, a2, v_dot_a;
  double z0, z1, z2, dz0, dz1, dz2;
  double cp_term, w_term, dh_over_m;
  double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
  double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
  double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
  double dw_term, dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
  double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
  double c_psi, s_psi, omg_den, omg_term;

  v0          = vel(0);
  v1          = vel(1);
  v2          = vel(2);
  a0          = acc(0);
  a1          = acc(1);
  a2          = acc(2);
  cp_term     = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
  w_term      = 1.0 + cp * cp_term;
  w0          = w_term * v0;
  w1          = w_term * v1;
  w2          = w_term * v2;
  dh_over_m   = dh / param_.mass;
  zu0         = a0 + dh_over_m * w0;
  zu1         = a1 + dh_over_m * w1;
  zu2         = a2 + dh_over_m * w2 + param_.gra;
  zu_sqr0     = zu0 * zu0;
  zu_sqr1     = zu1 * zu1;
  zu_sqr2     = zu2 * zu2;
  zu01        = zu0 * zu1;
  zu12        = zu1 * zu2;
  zu02        = zu0 * zu2;
  zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
  zu_norm     = sqrt(zu_sqr_norm);
  if (zu_norm < almost_zero)
  {
    return false;
  }
  z0       = zu0 / zu_norm;
  z1       = zu1 / zu_norm;
  z2       = zu2 / zu_norm;
  ng_den   = zu_sqr_norm * zu_norm;
  ng00     = (zu_sqr1 + zu_sqr2) / ng_den;
  ng01     = -zu01 / ng_den;
  ng02     = -zu02 / ng_den;
  ng11     = (zu_sqr0 + zu_sqr2) / ng_den;
  ng12     = -zu12 / ng_den;
  ng22     = (zu_sqr0 + zu_sqr1) / ng_den;
  v_dot_a  = v0 * a0 + v1 * a1 + v2 * a2;
  dw_term  = cp * v_dot_a / cp_term;
  dw0      = w_term * a0 + dw_term * v0;
  dw1      = w_term * a1 + dw_term * v1;
  dw2      = w_term * a2 + dw_term * v2;
  dz_term0 = jer(0) + dh_over_m * dw0;
  dz_term1 = jer(1) + dh_over_m * dw1;
  dz_term2 = jer(2) + dh_over_m * dw2;
  dz0      = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
  dz1      = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
  dz2      = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
  f_term0  = param_.mass * a0 + dv * w0;
  f_term1  = param_.mass * a1 + dv * w1;
  f_term2  = param_.mass * (a2 + param_.gra) + dv * w2;
  thr      = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
  if (1.0 + z2 < almost_zero)
  {
    return false;
  }
  tilt_den   = sqrt(2.0 * (1.0 + z2));
  tilt0      = 0.5 * tilt_den;
  tilt1      = -z1 / tilt_den;
  tilt2      = z0 / tilt_den;
  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  quat(0)    = tilt0 * c_half_psi;
  quat(1)    = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat(2)    = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat(3)    = tilt0 * s_half_psi;
  c_psi      = cos(psi);
  s_psi      = sin(psi);
  omg_den    = z2 + 1.0;
  omg_term   = dz2 / omg_den;
  omg(0)     = dz0 * s_psi - dz1 * c_psi - (z0 * s_psi - z1 * c_psi) * omg_term;
  omg(1)     = dz0 * c_psi + dz1 * s_psi - (z0 * c_psi + z1 * s_psi) * omg_term;
  omg(2)     = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

  return true;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void Alg0DifferentialFlatness::minimumSingularityFlatWithDrag(
    const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
    const Eigen::Vector3d &jer, double yaw, double yawd,
    const Eigen::Quaterniond &att_est, Eigen::Quaterniond &att,
    Eigen::Vector3d &omg, double &thrust)
{
  // Drag effect parameters (Drag may cause larger tracking error in aggressive
  // flight during our tests) dv >= dh is required dv is the rotor drag effect
  // in vertical direction, typical value is 0.35 dh is the rotor drag effect in
  // horizontal direction, typical value is 0.25 cp is the second-order drag
  // effect, typical valye is 0.01
  const double dh = 0.00;
  const double dv = 0.00;
  const double cp = 0.00;

  // veps is a smnoothing constant, do not change it
  const double veps = 0.02;  // ms^-s

  Eigen::Vector4d quat;
  if (flatnessWithDrag(vel, acc, jer, yaw, yawd, thrust, quat, omg, dh, dv, cp,
                       veps))
  {
    att         = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3));
    omg_old_    = omg;
    thrust_old_ = thrust;
    have_last_solution_ = true;
  }
  else
  {
    RCLCPP_WARN(
        rclcpp::get_logger("px4ctrl"),
        "Conor case: 1. Exactly inverted flight or 2. Unactuated falling");
    att = att_est;
    if (have_last_solution_)
    {
      omg    = omg_old_;
      thrust = thrust_old_;
    }
    else
    {
      // Deterministic cold-start fallback: hover thrust, zero rates.
      // (Old code read uninitialized function-static memory here.)
      omg = Eigen::Vector3d::Zero();
      thrust =
          param_.mass * (acc + param_.gra * Eigen::Vector3d::UnitZ()).norm();
    }
  }

  return;
}

void Alg0DifferentialFlatness::reset()
{
  omg_old_.setZero();
  thrust_old_         = 0.0;
  have_last_solution_ = false;
}
