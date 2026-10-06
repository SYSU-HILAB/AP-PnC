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

  MixerResult MotorModel::mix(const Eigen::Vector4d &input) const
  {
    if (!input.allFinite())
      throw std::invalid_argument("nonfinite mixer input");
    const double          thrust = std::clamp(input[0], 0.0, 1.0);
    const Eigen::Vector4d moment_output =
        cfg_.mixer.rightCols<3>() * input.tail<3>();
    Eigen::Vector4d output = moment_output + thrust * cfg_.mixer.col(0);
    // MATLAB <=/>= scans pick the LAST tied index. This affects desaturation.
    int lowest = 0, highest = 0;
    for (int i = 0; i < 4; ++i)
    {
      if (output[i] <= output[lowest])
        lowest = i;
      if (output[i] >= output[highest])
        highest = i;
    }
    const double lo = output[lowest], hi = output[highest], span = hi - lo;
    double       increase = 0.3 * thrust, decrease = 0.3 * thrust;
    double       boost = 0.0, scale = 1.0;
    auto         limit_both_ends = [&]()
    {
      const double to_lower = (0.0 - (thrust + boost)) / moment_output[lowest];
      const double to_upper = (1.0 - (thrust + boost)) / moment_output[highest];
      scale                 = std::min(to_lower, to_upper);
    };
    if (lo < 0.0 && hi < 1.0 && span <= 1.0)
    {
      if (increase >= -lo)
        boost = -lo;
      else
      {
        boost = increase;
        scale = -(thrust + boost) / moment_output[lowest];
      }
    }
    else if (hi > 1.0 && lo > 0.0 && span <= 1.0)
    {
      if (decrease >= hi - 1.0)
        boost = -(hi - 1.0);
      else
      {
        boost = -decrease;
        scale = (1.0 - thrust - boost) / moment_output[highest];
      }
    }
    else if (lo < 0.0 && hi < 1.0 && span > 1.0)
    {
      increase = std::clamp(increase, 0.0, 0.5 * (1.0 - thrust));
      boost    = std::clamp((hi - 1.0 - lo) / 2.0, 0.0, increase);
      limit_both_ends();
    }
    else if (hi > 1.0 && lo > 0.0 && span > 1.0)
    {
      decrease = std::clamp(decrease, 0.0, 0.5 * thrust);
      boost    = std::clamp(-(hi - 1.0 - lo) / 2.0, -decrease, 0.0);
      limit_both_ends();
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
      limit_both_ends();
    }
    MixerResult result;
    result.fraction = moment_output * scale + cfg_.mixer.col(0) * thrust +
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
    const double advance_ratio = 60.0 * axial / cfg_.diameter;
    auto evaluate = [advance_ratio, rpm](const Eigen::Vector3d &polynomial)
    {
      return polynomial[0] * advance_ratio * advance_ratio +
             polynomial[1] * advance_ratio * rpm + polynomial[2] * rpm * rpm;
    };
    return {cfg_.ct * evaluate(cfg_.thrust_poly),
            cfg_.cq * evaluate(cfg_.torque_poly)};
  }

  Wrench MotorModel::wrench(const State &state) const
  {
    const Eigen::Vector3d rates    = wing_from_flu(state.angular_velocity);
    const Eigen::Vector3d velocity = wing_from_flu(
        state.attitude.conjugate() * (state.velocity - cfg_.wind_world));
    Eigen::Vector3d force  = Eigen::Vector3d::Zero(),
                    moment = cfg_.damping_wing.cwiseProduct(rates);
    constexpr double pi    = 3.14159265358979323846;
    for (int i = 0; i < 4; ++i)
    {
      const auto &motor_rotation = rotations_[i];
      const auto  propeller_load = propeller(
          state.motors.rpm[i], (motor_rotation.transpose() * velocity).x());
      const double          reaction_sign = i % 2 == 0 ? -1.0 : 1.0;
      const Eigen::Vector3d thrust_local(propeller_load[0], 0.0, 0.0);
      force += motor_rotation * thrust_local;
      moment += motor_rotation *
                (Eigen::Vector3d(reaction_sign * propeller_load[1], 0.0, 0.0) +
                 Eigen::Vector3d(0.0, cfg_.arms[i], 0.0).cross(thrust_local));
      const Eigen::Vector3d rotor_angular_momentum =
          motor_rotation *
          Eigen::Vector3d(
              -reaction_sign * cfg_.jm * state.motors.rpm[i] * 2 * pi / 60.0, 0,
              0);
      moment += rotor_angular_momentum.cross(rates);
    }
    return {flu_from_wing(force), flu_from_wing(moment)};
  }

  Eigen::Vector4d MotorModel::derivative(const State &state) const
  {
    return (state.motors.rpm_sp - state.motors.rpm) / cfg_.tau;
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
    double pwm_low = cfg_.pwm_min, pwm_high = cfg_.pwm_max;
    for (int iteration = 0; iteration < 60; ++iteration)
    {
      const double pwm_mid = (pwm_low + pwm_high) / 2.0;
      if (esc_rpm(pwm_mid) < target_rpm)
        pwm_low = pwm_mid;
      else
        pwm_high = pwm_mid;
    }
    return std::clamp(((pwm_low + pwm_high) / 2.0 - cfg_.pwm_min) /
                          (cfg_.pwm_max - cfg_.pwm_min),
                      0.0, 1.0);
  }

  State MotorModel::initialize(const State   &initial,
                               const Command &command) const
  {
    State state           = initial;
    state.motors          = MotorState{};
    const double fraction = throttle(command.specific_force);
    state.motors.duty.setConstant(2.0 * fraction - 1.0);
    state.motors.rpm.setConstant(
        esc_rpm(cfg_.pwm_min + fraction * (cfg_.pwm_max - cfg_.pwm_min)));
    state.motors.rpm_sp = state.motors.rpm;
    return state;
  }

  State MotorModel::prepare(const State &state, const Command &command) const
  {
    State result = state;
    if (result.motors.physics_ticks % inner_stride_ == 0)
    {
      const double          dt = static_cast<double>(cfg_.inner_dt_ns) * 1e-9;
      const Eigen::Vector3d setpoint =
          command.rates.cwiseMax(-max_rate_).cwiseMin(max_rate_);
      const Eigen::Vector3d error =
          wing_from_flu(setpoint - result.angular_velocity);
      if (!result.motors.limited)
        result.motors.integral += dt * cfg_.ki.cwiseProduct(error);
      const Eigen::Vector3d action =
          cfg_.kp.cwiseProduct(error) + result.motors.integral +
          cfg_.kd.cwiseProduct(error - result.motors.previous_error) / dt;
      result.motors.previous_error  = error;
      const Eigen::Vector3d limited = action.cwiseMax(-Eigen::Vector3d::Ones())
                                          .cwiseMin(Eigen::Vector3d::Ones());
      Eigen::Vector4d input;
      input << throttle(command.specific_force), limited;
      const auto mixed = mix(input);
      result.motors.limited =
          mixed.lower || mixed.upper || (action.array().abs() > 1.0).any();
      result.motors.duty =
          (2.0 * mixed.fraction.array() - 1.0).max(-1.0).min(1.0).matrix();
      for (int i = 0; i < 4; ++i)
      {
        const double pwm =
            result.motors.duty[i] * (cfg_.pwm_max - cfg_.pwm_min) / 2.0 +
            (cfg_.pwm_max + cfg_.pwm_min) / 2.0;
        result.motors.rpm_sp[i] = esc_rpm(pwm);
      }
    }
    ++result.motors.physics_ticks;
    return result;
  }
}  // namespace simple_sim
