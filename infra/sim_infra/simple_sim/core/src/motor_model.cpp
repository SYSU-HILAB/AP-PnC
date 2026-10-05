#include "simple_sim/motor_model.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace simple_sim
{
  void MotorModelConfig::validate() const
  {
    const double values[] = {tau, diameter, ct, cq, jm, pwm_min, pwm_max};
    for (double v : values)
      if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument("invalid motor parameter");
    if (pwm_max <= pwm_min || inner_dt_ns <= 0 || !arms.allFinite() ||
        (arms.array() <= 0.0).any() || !arm_angles.allFinite() ||
        !tilts.allFinite() || !mixer.allFinite() || !thrust_poly.allFinite() ||
        !torque_poly.allFinite() || !esc_poly.allFinite() ||
        !damping_wing.allFinite() || !kp.allFinite() || !ki.allFinite() ||
        !kd.allFinite() || !wind_world.allFinite() ||
        (kp.array() < 0.0).any() || (ki.array() < 0.0).any() ||
        (kd.array() < 0.0).any() ||
        !mixer.col(0).isApprox(Eigen::Vector4d::Ones()) ||
        (tilts.array().abs() >= 1.5707963267948966).any())
      throw std::invalid_argument("invalid motor geometry/mixer/PID");
  }

  MotorModel::MotorModel(MotorModelConfig config, double mass, double max_force,
                         Eigen::Vector3d max_rate, std::int64_t physics_dt_ns)
      : cfg_(std::move(config)),
        mass_(mass),
        max_force_(max_force),
        max_rate_(std::move(max_rate))
  {
    cfg_.validate();
    if (physics_dt_ns <= 0 || cfg_.inner_dt_ns % physics_dt_ns != 0 ||
        !std::isfinite(mass_) || mass_ <= 0.0 || !std::isfinite(max_force_) ||
        max_force_ <= 0.0 || !max_rate_.allFinite() ||
        (max_rate_.array() <= 0.0).any())
      throw std::invalid_argument("invalid motor sampling/command limits");
    inner_stride_ = cfg_.inner_dt_ns / physics_dt_ns;
    for (int i = 0; i < 4; ++i)
      rotations_[i] =
          (Eigen::AngleAxisd(cfg_.arm_angles[i], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(cfg_.tilts[i], Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
  }

  Eigen::Matrix3d MotorModel::wing_from_flu()
  {
    Eigen::Matrix3d c;
    c << 0, 0, 1, 0, -1, 0, 1, 0, 0;
    return c;
  }

  MixerResult MotorModel::mix(const Eigen::Vector4d &input) const
  {
    if (!input.allFinite())
      throw std::invalid_argument("nonfinite mixer input");
    const double          thrust = std::clamp(input[0], 0.0, 1.0);
    const Eigen::Vector4d rp     = cfg_.mixer.rightCols<3>() * input.tail<3>();
    Eigen::Vector4d       output = rp + thrust * cfg_.mixer.col(0);
    // MATLAB <=/>= scans pick the LAST tied index. This affects desaturation.
    int imin = 0, imax = 0;
    for (int i = 0; i < 4; ++i)
    {
      if (output[i] <= output[imin])
        imin = i;
      if (output[i] >= output[imax])
        imax = i;
    }
    const double lo = output[imin], hi = output[imax], span = hi - lo;
    double       increase = 0.3 * thrust, decrease = 0.3 * thrust;
    double       boost = 0.0, scale = 1.0;
    auto         both = [&]()
    {
      const double d = (0.0 - (thrust + boost)) / rp[imin];
      const double u = (1.0 - (thrust + boost)) / rp[imax];
      scale          = std::min(d, u);
    };
    if (lo < 0.0 && hi < 1.0 && span <= 1.0)
    {
      if (increase >= -lo)
        boost = -lo;
      else
      {
        boost = increase;
        scale = -(thrust + boost) / rp[imin];
      }
    }
    else if (hi > 1.0 && lo > 0.0 && span <= 1.0)
    {
      if (decrease >= hi - 1.0)
        boost = -(hi - 1.0);
      else
      {
        boost = -decrease;
        scale = (1.0 - thrust - boost) / rp[imax];
      }
    }
    else if (lo < 0.0 && hi < 1.0 && span > 1.0)
    {
      increase = std::clamp(increase, 0.0, 0.5 * (1.0 - thrust));
      boost    = std::clamp((hi - 1.0 - lo) / 2.0, 0.0, increase);
      both();
    }
    else if (hi > 1.0 && lo > 0.0 && span > 1.0)
    {
      decrease = std::clamp(decrease, 0.0, 0.5 * thrust);
      boost    = std::clamp(-(hi - 1.0 - lo) / 2.0, -decrease, 0.0);
      both();
    }
    else if (lo < 0.0 && hi > 1.0)
    {
      if (hi - 1.0 >= -lo)
      {
        decrease = std::clamp(decrease, 0.0, 0.5 * thrust);
        boost    = std::clamp(-(hi - 1.0 - lo) / 2.0, -decrease, increase);
      }
      else
      {
        increase = std::clamp(increase, 0.0, 0.5 * (1.0 - thrust));
        boost    = std::clamp((hi - 1.0 - lo) / 2.0, -decrease, increase);
      }
      both();
    }
    MixerResult result;
    result.fraction = rp * scale + cfg_.mixer.col(0) * thrust +
                      Eigen::Vector4d::Constant(boost);
    result.lower = lo < 0.0;
    result.upper = hi > 1.0;
    if (!result.fraction.allFinite())
      throw std::runtime_error("nonfinite mixer result");
    return result;
  }

  double MotorModel::esc_rpm(double pwm) const
  {
    return (cfg_.esc_poly[0] * pwm + cfg_.esc_poly[1]) * pwm + cfg_.esc_poly[2];
  }

  Eigen::Vector2d MotorModel::propeller(double rpm, double axial) const
  {
    // Polynomial expansion of RPM^2 * f(60 V / (D RPM)); avoids 0/0 at rest.
    // At zero RPM this is the algebraic limit, not a measured windmilling
    // model.
    const double a    = 60.0 * axial / cfg_.diameter;
    auto         eval = [a, rpm](const Eigen::Vector3d &p)
    { return p[0] * a * a + p[1] * a * rpm + p[2] * rpm * rpm; };
    return {cfg_.ct * eval(cfg_.thrust_poly), cfg_.cq * eval(cfg_.torque_poly)};
  }

  Wrench MotorModel::wrench(const State &s) const
  {
    const Eigen::Matrix3d c     = wing_from_flu();
    const Eigen::Vector3d rates = c * s.angular_velocity;
    const Eigen::Vector3d velocity =
        c * (s.attitude.conjugate() * (s.velocity - cfg_.wind_world));
    Eigen::Vector3d force  = Eigen::Vector3d::Zero(),
                    moment = cfg_.damping_wing.cwiseProduct(rates);
    constexpr double pi    = 3.14159265358979323846;
    for (int i = 0; i < 4; ++i)
    {
      const auto &r = rotations_[i];
      const auto  tq =
          propeller(s.motors.rpm[i], (r.transpose() * velocity).x());
      const double          reaction_sign = i % 2 == 0 ? -1.0 : 1.0;
      const Eigen::Vector3d f(tq[0], 0.0, 0.0);
      force += r * f;
      moment += r * (Eigen::Vector3d(reaction_sign * tq[1], 0.0, 0.0) +
                     Eigen::Vector3d(0.0, cfg_.arms[i], 0.0).cross(f));
      const Eigen::Vector3d h =
          r *
          Eigen::Vector3d(
              -reaction_sign * cfg_.jm * s.motors.rpm[i] * 2 * pi / 60.0, 0, 0);
      moment += h.cross(rates);
    }
    return {c.transpose() * force, c.transpose() * moment};
  }

  Eigen::Vector4d MotorModel::derivative(const State &s) const
  {
    return (s.motors.rpm_sp - s.motors.rpm) / cfg_.tau;
  }

  double MotorModel::throttle(double specific_force) const
  {
    // New interface adapter: NMPC provides N/kg, MATLAB expects [0,1].
    // Invert the STATIC balanced-motor thrust curve. Inflow remains in plant.
    double projection = 0.0;
    for (const auto &r : rotations_)
      projection += r(0, 0);
    const double target_rpm =
        std::sqrt(mass_ * std::clamp(specific_force, 0.0, max_force_) /
                  (projection * cfg_.ct * cfg_.thrust_poly[2]));
    double lo = cfg_.pwm_min, hi = cfg_.pwm_max;
    for (int k = 0; k < 60; ++k)
    {
      const double mid = (lo + hi) / 2.0;
      if (esc_rpm(mid) < target_rpm)
        lo = mid;
      else
        hi = mid;
    }
    return std::clamp(
        ((lo + hi) / 2.0 - cfg_.pwm_min) / (cfg_.pwm_max - cfg_.pwm_min), 0.0,
        1.0);
  }

  State MotorModel::initialize(const State   &initial,
                               const Command &command) const
  {
    State s               = initial;
    s.motors              = MotorState{};
    const double fraction = throttle(command.specific_force);
    s.motors.duty.setConstant(2.0 * fraction - 1.0);
    s.motors.rpm.setConstant(
        esc_rpm(cfg_.pwm_min + fraction * (cfg_.pwm_max - cfg_.pwm_min)));
    s.motors.rpm_sp = s.motors.rpm;
    return s;
  }

  State MotorModel::prepare(const State &state, const Command &command) const
  {
    State s = state;
    if (s.motors.physics_ticks % inner_stride_ == 0)
    {
      const double          dt = static_cast<double>(cfg_.inner_dt_ns) * 1e-9;
      const Eigen::Vector3d setpoint =
          command.rates.cwiseMax(-max_rate_).cwiseMin(max_rate_);
      const Eigen::Vector3d error =
          wing_from_flu() * (setpoint - s.angular_velocity);
      if (!s.motors.limited)
        s.motors.integral += dt * cfg_.ki.cwiseProduct(error);
      const Eigen::Vector3d action =
          cfg_.kp.cwiseProduct(error) + s.motors.integral +
          cfg_.kd.cwiseProduct(error - s.motors.previous_error) / dt;
      s.motors.previous_error       = error;
      const Eigen::Vector3d limited = action.cwiseMax(-Eigen::Vector3d::Ones())
                                          .cwiseMin(Eigen::Vector3d::Ones());
      Eigen::Vector4d input;
      input << throttle(command.specific_force), limited;
      const auto mixed = mix(input);
      s.motors.limited =
          mixed.lower || mixed.upper || (action.array().abs() > 1.0).any();
      s.motors.duty =
          (2.0 * mixed.fraction.array() - 1.0).max(-1.0).min(1.0).matrix();
      for (int i = 0; i < 4; ++i)
      {
        const double pwm =
            s.motors.duty[i] * (cfg_.pwm_max - cfg_.pwm_min) / 2.0 +
            (cfg_.pwm_max + cfg_.pwm_min) / 2.0;
        s.motors.rpm_sp[i] = esc_rpm(pwm);
      }
    }
    ++s.motors.physics_ticks;
    return s;
  }
}  // namespace simple_sim
