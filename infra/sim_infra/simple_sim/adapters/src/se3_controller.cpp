#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "simple_sim/adapters/controllers.hpp"

namespace simple_sim
{
  namespace
  {
    Eigen::Matrix3d desired_attitude(const Eigen::Vector3d &force,
                                     const Eigen::Vector3d &yb)
    {
      if (!force.allFinite() || force.norm() < 1e-9 || !yb.allFinite())
        throw std::runtime_error("degenerate SE3 reference");
      const Eigen::Vector3d z = force.normalized();
      Eigen::Vector3d       y = yb - z * yb.dot(z);
      if (y.norm() < 1e-9)
        throw std::runtime_error("reference yb parallel to thrust axis");
      y.normalize();
      Eigen::Matrix3d r;
      r.col(0) = y.cross(z);
      r.col(1) = y;
      r.col(2) = z;
      return r;
    }
    // Thrust axis of the flatness map with a numerically solved angle of
    // attack.
    //
    // With the simulator convention alpha = atan2(v_Bx, v_Bz) (thrust along
    // +Z_B, wing normal along +X_B) and the stability frame x_s = v/|v|, z_s =
    // lateral(eta), the body axes are
    //     x_B =  sin(a) x_s + cos(a) z_s
    //     z_B = -cos(a) x_s + sin(a) z_s
    // (check: a=90deg gives v along x_B, a=0 gives z_B along -v). The balance
    // is the body-X projection of the desired specific force against the wing
    // force, solved by Newton with a numerical derivative; no phi-theory closed
    // form and no lookup table is involved.
    bool flatness_thrust_axis(const Eigen::Vector3d &eta,
                              const Eigen::Vector3d &velocity, double V,
                              aerodynamics::AerodynamicsInterface &model,
                              double mass, bool zero_thrust_axis_aero_,
                              double &alpha_rad, bool &alpha_initialized,
                              const AeroConfig &aero_config_in,
                              Eigen::Vector3d  &thrust_axis)
    {
      const AeroConfig *const aero_config = &aero_config_in;
      const Eigen::Vector3d   x_s         = velocity / V;
      Eigen::Vector3d         lateral     = eta - x_s * x_s.dot(eta);
      if (lateral.norm() < 1e-6)
        return false;
      const Eigen::Vector3d z_s      = lateral.normalized();
      const Eigen::Vector3d y_s      = z_s.cross(x_s);
      const auto            residual = [&](double a)
      {
        const Eigen::Vector3d x_b = std::sin(a) * x_s + std::cos(a) * z_s;
        const Eigen::Vector3d z_b = -std::cos(a) * x_s + std::sin(a) * z_s;
        Eigen::Vector3d       force;
        Eigen::Vector3d       moment;
        double                aero_alpha = 0.0;
        double                aero_beta  = 0.0;
        // The model is evaluated exactly as the plant adapter does: it takes
        // the WING-frame airspeed and returns the wrench in the WING frame.
        const Eigen::Vector3d body_speed =
            V * Eigen::Vector3d(std::sin(a), 0.0, std::cos(a));
        Eigen::Vector3d force_wing;
        model.getAeroWrench(wing_from_flu(body_speed), force_wing, moment,
                            aero_alpha, aero_beta);
        force = aero_config->scale * flu_from_wing(force_wing);
        if (zero_thrust_axis_aero_)
          force.z() = 0.0;
        // eta and x_b live in the world frame, the model returns the wrench in
        // the body frame; rotate before projecting or the components are mixed.
        Eigen::Matrix3d r_wb;
        r_wb.col(0) = x_b;
        r_wb.col(1) = y_s;
        r_wb.col(2) = z_b;
        return eta.dot(x_b) - (r_wb * force).dot(x_b) / mass;
      };
      // Scan for sign changes and bisect. A bare Newton step is unusable here:
      // the operating band (40-60 deg) sits on the shoulder of the lift polar,
      // where the residual derivative approaches zero and Newton jumps to a
      // distant branch. Among the bracketed roots prefer the one closest to the
      // previous solution, and reject the whole feedforward when that root
      // moved too far, so the caller can fall back to the plain SE3 attitude.
      const double lo       = -0.5 * M_PI;
      const double hi       = M_PI;
      const int    n        = 96;
      bool         found    = false;
      double       best     = alpha_rad;
      double       best_err = std::numeric_limits<double>::max();
      double       a_prev   = lo;
      double       r_prev   = residual(lo);
      for (int i = 1; i <= n; ++i)
      {
        const double a_cur = lo + (hi - lo) * static_cast<double>(i) / n;
        const double r_cur = residual(a_cur);
        if (r_prev == 0.0 || r_prev * r_cur < 0.0)
        {
          double a  = a_prev;
          double b  = a_cur;
          double ra = r_prev;
          for (int k = 0; k < 60; ++k)
          {
            const double m  = 0.5 * (a + b);
            const double rm = residual(m);
            if (ra * rm <= 0.0)
            {
              b = m;
            }
            else
            {
              a  = m;
              ra = rm;
            }
          }
          const double root = 0.5 * (a + b);
          const double err  = std::abs(root - alpha_rad);
          if (err < best_err)
          {
            best_err = err;
            best     = root;
            found    = true;
          }
        }
        a_prev = a_cur;
        r_prev = r_cur;
      }
      // On the first engaged solve the warm start is only a guess, so accept
      // the closest bracketed root; afterwards require continuity, otherwise a
      // branch jump would be latched in forever because a rejected solve does
      // not update the previous angle.
      const double max_jump = alpha_initialized ? 0.5 : 1.5;
      if (!found || best_err > max_jump)
        return false;
      alpha_initialized = true;
      alpha_rad         = best;
      thrust_axis       = -std::cos(best) * x_s + std::sin(best) * z_s;
      return true;
    }
  }  // namespace
  State initial_state(const planner::core::ReferencePoint &ref, double gravity)
  {
    State s;
    s.position = ref.p;
    s.velocity = ref.v;
    s.attitude = Eigen::Quaterniond(
        desired_attitude(ref.a + gravity * Eigen::Vector3d::UnitZ(), ref.yb));
    s.angular_velocity = ref.omega;
    validate_state(s);
    return s;
  }
  Se3Controller::Se3Controller(
      std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
      double gravity, Se3Gains gains,
      std::shared_ptr<aerodynamics::AerodynamicsInterface> aero_model,
      double mass, bool zero_thrust_axis_aero, AeroConfig aero_config)
      : trajectory_(std::move(trajectory)),
        gravity_(gravity),
        gains_(std::move(gains)),
        aero_model_(std::move(aero_model)),
        mass_(mass),
        alpha_prev_(M_PI / 2.0),
        zero_thrust_axis_aero_(zero_thrust_axis_aero),
        alpha_initialized_(false),
        aero_config_(std::move(aero_config))
  {
    if (!trajectory_ || trajectory_->empty() || !std::isfinite(gravity_) ||
        gravity_ <= 0.0)
      throw std::invalid_argument("invalid SE3 trajectory or gravity");
    for (const auto *v : {&gains_.position, &gains_.velocity, &gains_.attitude})
      if (!v->allFinite() || (v->array() <= 0.0).any())
        throw std::invalid_argument("SE3 gains must be finite and positive");
  }
  void Se3Controller::reset(const State &initial)
  {
    validate_state(initial);
    alpha_prev_        = M_PI / 2.0;
    alpha_initialized_ = false;
  }
  ControlResult Se3Controller::compute(const StepContext &ctx, const State &s,
                                       const Feedback &)
  {
    if (ctx.time_s() < 0.0)
      throw std::runtime_error("SE3 reference time outside trajectory");
    // Past the end the trajectory holds its last point (ReferenceTrajectory
    // clamps), so the controller can keep tracking through the terminal dwell
    // instead of aborting the run.
    const auto            ref = trajectory_->sample(ctx.time_s());
    const Eigen::Vector3d force =
        ref.a + gravity_ * Eigen::Vector3d::UnitZ() +
        gains_.position.cwiseProduct(ref.p - s.position) +
        gains_.velocity.cwiseProduct(ref.v - s.velocity);
    Eigen::Matrix3d desired;
    double          thrust = 0.0;
    if (gains_.aero_flatness_feedforward && aero_model_ && mass_ > 0.0 &&
        s.velocity.norm() > 0.5)
    {
      // Flatness feedforward: solve alpha from the true aerodynamic force
      // balance, rebuild the thrust axis, and keep the planned yb.
      Eigen::Vector3d z_b;
      if (flatness_thrust_axis(force, s.velocity, s.velocity.norm(),
                               *aero_model_, mass_, zero_thrust_axis_aero_,
                               alpha_prev_, alpha_initialized_, aero_config_,
                               z_b))
      {
        desired = desired_attitude(z_b, ref.yb);
        thrust  = force.dot(z_b);
      }
      else
      {
        desired = desired_attitude(force, ref.yb);
        thrust  = force.dot(s.attitude.toRotationMatrix().col(2));
      }
    }
    else
    {
      desired = desired_attitude(force, ref.yb);
      thrust  = force.dot(s.attitude.toRotationMatrix().col(2));
    }
    const Eigen::Matrix3d   current = s.attitude.toRotationMatrix();
    const Eigen::AngleAxisd error(desired * current.transpose());
    ControlResult           result;
    result.command.specific_force = thrust;
    result.command.rates =
        gains_.attitude.cwiseProduct(current.transpose() *
                                     (error.axis() * error.angle())) +
        current.transpose() * desired * ref.omega;
    // Per-axis output limit, the same one the NMPC carries as command bounds: a
    // demand the airframe cannot realise must not leave the controller,
    // whatever the reference asks for.
    result.command.rates = result.command.rates.cwiseMax(-gains_.max_rate)
                               .cwiseMin(gains_.max_rate);
    return result;
  }
}  // namespace simple_sim
