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
    using Vector = Eigen::Matrix<double, 13, 1>;
    Vector pack(const State &s)
    {
      Vector x;
      x << s.position, s.velocity, s.attitude.w(), s.attitude.x(),
          s.attitude.y(), s.attitude.z(), s.angular_velocity;
      return x;
    }
    State unpack(const Vector &x)
    {
      State s;
      s.position = x.segment<3>(0);
      s.velocity = x.segment<3>(3);
      s.attitude = Eigen::Quaterniond(x[6], x[7], x[8], x[9]);
      if (!s.attitude.coeffs().allFinite() || s.attitude.norm() < 1e-12)
        throw std::runtime_error("invalid integration quaternion");
      s.attitude.normalize();
      s.angular_velocity = x.segment<3>(10);
      validate_state(s);
      return s;
    }
  }  // namespace

  Plant::Plant(PlantConfig config, ForceModel aerodynamic)
      : config_(std::move(config)), aerodynamic_(std::move(aerodynamic))
  {
    config_.validate();
  }
  Wrench Plant::aero(const State &s) const
  {
    Wrench w = aerodynamic_ ? aerodynamic_(s) : Wrench{};
    validate_wrench(w);
    return w;
  }
  Feedback Plant::feedback(const State &s, const Command &c,
                           const Actuation &act) const
  {
    validate_state(s);
    validate_command(c);
    Feedback f;
    f.aerodynamic        = aero(s);
    const Wrench applied = act(s, c);
    validate_wrench(applied);
    f.specific_force = (applied.force + f.aerodynamic.force) / config_.mass;
    return f;
  }
  State Plant::advance(const State &s, const Command &c, double dt,
                       const Actuation &act) const
  {
    validate_state(s);
    validate_command(c);
    if (!std::isfinite(dt) || dt <= 0.0 || !act)
      throw std::invalid_argument("invalid integration step or actuation");
    const auto derivative = [&](const Vector &x) -> Vector
    {
      const State  stage = unpack(x);
      const Wrench a     = act(stage, c);
      const Wrench air   = aero(stage);
      validate_wrench(a);
      Vector d        = Vector::Zero();
      d.segment<3>(0) = stage.velocity;
      d.segment<3>(3) = Eigen::Vector3d(0.0, 0.0, -config_.gravity) +
                        stage.attitude * ((a.force + air.force) / config_.mass);
      const Eigen::Vector3d w = stage.angular_velocity;
      // q_dot = 0.5 * q * [0, omega_body], Hamilton quaternion.
      const Eigen::Quaterniond qdot =
          stage.attitude * Eigen::Quaterniond(0.0, w.x(), w.y(), w.z());
      d.segment<4>(6) << qdot.w() * 0.5, qdot.x() * 0.5, qdot.y() * 0.5,
          qdot.z() * 0.5;
      d.segment<3>(10) =
          (a.moment + air.moment - w.cross(config_.inertia.cwiseProduct(w)))
              .cwiseQuotient(config_.inertia);
      if (!d.allFinite())
        throw std::runtime_error("non-finite integration derivative");
      return d;
    };
    const Vector x  = pack(s);
    const Vector k1 = derivative(x);
    const Vector k2 = derivative(x + 0.5 * dt * k1);
    const Vector k3 = derivative(x + 0.5 * dt * k2);
    const Vector k4 = derivative(x + dt * k3);
    return unpack(x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
  }
}  // namespace simple_sim
