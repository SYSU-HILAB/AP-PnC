#include "simple_sim/config.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "simple_sim/types.hpp"

namespace simple_sim
{
  namespace
  {
    void positive_vector(const Eigen::Vector3d &v, const char *name)
    {
      if (!v.allFinite() || (v.array() <= 0.0).any())
        throw std::invalid_argument(name);
    }
  }  // namespace

  void PlantConfig::validate() const
  {
    if (!std::isfinite(mass) || mass <= 0.0 || !std::isfinite(gravity) ||
        gravity <= 0.0)
      throw std::invalid_argument(
          "mass and gravity must be positive and finite");
    positive_vector(inertia, "diagonal inertia must be positive and finite");
  }
  void RateLoopConfig::validate() const
  {
    positive_vector(gain, "rate gains must be positive and finite");
    positive_vector(max_rate, "rate limits must be positive and finite");
    positive_vector(max_moment, "moment limits must be positive and finite");
    if (!std::isfinite(max_specific_force) || max_specific_force <= 0.0)
      throw std::invalid_argument(
          "specific-force limit must be positive and finite");
    if (!std::isfinite(thrust_tau_s) || thrust_tau_s <= 0.0)
      throw std::invalid_argument(
          "thrust time constant must be positive and finite");
  }
  void RunnerConfig::validate() const
  {
    if (physics_dt_ns <= 0 || control_dt_ns <= 0 || steps <= 0 ||
        control_dt_ns % physics_dt_ns != 0 ||
        steps > std::numeric_limits<std::int64_t>::max() / control_dt_ns)
      throw std::invalid_argument(
          "invalid step count or non-integral control/physics ratio");
  }
  void validate_state(const State &s)
  {
    if (!s.position.allFinite() || !s.velocity.allFinite() ||
        !s.angular_velocity.allFinite() || !s.attitude.coeffs().allFinite() ||
        std::abs(s.attitude.norm() - 1.0) > 1e-6 || !s.motors.rpm.allFinite() ||
        !s.motors.rpm_sp.allFinite() || !s.motors.duty.allFinite() ||
        !s.motors.integral.allFinite() ||
        !s.motors.previous_error.allFinite() || s.motors.physics_ticks < 0 ||
        (s.motors.rpm.array() < 0.0).any() ||
        (s.motors.rpm_sp.array() < 0.0).any())
      throw std::runtime_error("non-finite state or non-unit quaternion");
  }
  void validate_command(const Command &c)
  {
    if (!std::isfinite(c.specific_force) || !c.rates.allFinite())
      throw std::runtime_error("non-finite control command");
  }
}  // namespace simple_sim
