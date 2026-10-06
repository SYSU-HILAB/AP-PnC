#include "simple_sim/plant.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace simple_sim
{
  namespace
  {
    void validate_wrench(const Wrench &w)
    {
      if (!w.force.allFinite() || !w.moment.allFinite())
        throw std::runtime_error("non-finite applied wrench");
    }
    using Vector = Eigen::Matrix<double, 17, 1>;
    Vector pack(const State &s)
    {
      Vector x;
      x << s.position, s.velocity, s.attitude.w(), s.attitude.x(),
          s.attitude.y(), s.attitude.z(), s.angular_velocity, s.motors.rpm;
      return x;
    }
    State unpack(const Vector &x, const State &held)
    {
      State s    = held;
      s.position = x.segment<3>(0);
      s.velocity = x.segment<3>(3);
      s.attitude = Eigen::Quaterniond(x[6], x[7], x[8], x[9]);
      if (!s.attitude.coeffs().allFinite() || s.attitude.norm() < 1e-12)
        throw std::runtime_error("invalid integration quaternion");
      s.attitude.normalize();
      s.angular_velocity = x.segment<3>(10);
      s.motors.rpm       = x.segment<4>(13);
      validate_state(s);
      return s;
    }
  }  // namespace

  Plant::Plant(PlantConfig config, ForceModel aerodynamic,
               MotorDerivative motors)
      : config_(std::move(config)),
        aerodynamic_(std::move(aerodynamic)),
        motors_(std::move(motors))
  {
    config_.validate();
  }
  Wrench Plant::aero(const State &state) const
  {
    Wrench aerodynamic = aerodynamic_ ? aerodynamic_(state) : Wrench{};
    validate_wrench(aerodynamic);
    return aerodynamic;
  }
  Feedback Plant::feedback(const State &state, const Command &command,
                           const Actuation &actuation) const
  {
    validate_state(state);
    validate_command(command);
    Feedback feedback;
    feedback.aerodynamic  = aero(state);
    const Wrench actuated = actuation(state, command);
    validate_wrench(actuated);
    feedback.applied = actuated;
    feedback.specific_force =
        (actuated.force + feedback.aerodynamic.force) / config_.mass;
    return feedback;
  }
  State Plant::advance(const State &state, const Command &command, double dt,
                       const Actuation &actuation) const
  {
    validate_state(state);
    validate_command(command);
    if (!std::isfinite(dt) || dt <= 0.0 || !actuation)
      throw std::invalid_argument("invalid integration step or actuation");
    const auto derivative = [&](const Vector &x) -> Vector
    {
      const State  stage       = unpack(x, state);
      const Wrench actuated    = actuation(stage, command);
      const Wrench aerodynamic = aero(stage);
      validate_wrench(actuated);
      Vector state_rate        = Vector::Zero();
      state_rate.segment<3>(0) = stage.velocity;
      state_rate.segment<3>(3) =
          Eigen::Vector3d(0.0, 0.0, -config_.gravity) +
          stage.attitude *
              ((actuated.force + aerodynamic.force) / config_.mass);
      const Eigen::Vector3d omega_body = stage.angular_velocity;
      // q_dot = 0.5 * q * [0, omega_body], Hamilton quaternion.
      const Eigen::Quaterniond quaternion_rate =
          stage.attitude * Eigen::Quaterniond(0.0, omega_body.x(),
                                              omega_body.y(), omega_body.z());
      state_rate.segment<4>(6) << quaternion_rate.w() * 0.5,
          quaternion_rate.x() * 0.5, quaternion_rate.y() * 0.5,
          quaternion_rate.z() * 0.5;
      state_rate.segment<3>(10) =
          (actuated.moment + aerodynamic.moment -
           omega_body.cross(config_.inertia.cwiseProduct(omega_body)))
              .cwiseQuotient(config_.inertia);
      if (motors_)
        state_rate.segment<4>(13) = motors_(stage);
      if (!state_rate.allFinite())
        throw std::runtime_error("non-finite integration derivative");
      return state_rate;
    };
    const Vector x  = pack(state);
    const Vector k1 = derivative(x);
    const Vector k2 = derivative(x + 0.5 * dt * k1);
    const Vector k3 = derivative(x + 0.5 * dt * k2);
    const Vector k4 = derivative(x + dt * k3);
    return unpack(x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4), state);
  }
}  // namespace simple_sim
