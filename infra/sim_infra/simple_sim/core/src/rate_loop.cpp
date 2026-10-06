#include "simple_sim/rate_loop.hpp"

#include <algorithm>
#include <utility>

namespace simple_sim
{
  RateLoop::RateLoop(PlantConfig plant, RateLoopConfig config,
                     bool filtered_thrust)
      : plant_(std::move(plant)),
        config_(std::move(config)),
        filtered_thrust_(filtered_thrust)
  {
    plant_.validate();
    config_.validate();
  }
  Wrench RateLoop::operator()(const State &s, const Command &c) const
  {
    validate_state(s);
    validate_command(c);
    Wrench       result;
    const double thrust =
        filtered_thrust_
            ? s.motors.specific_force
            : std::clamp(c.specific_force, 0.0, config_.max_specific_force);
    result.force.z() = plant_.mass * thrust;
    const Eigen::Vector3d desired =
        c.rates.cwiseMax(-config_.max_rate).cwiseMin(config_.max_rate);
    result.moment = plant_.inertia.cwiseProduct(config_.gain.cwiseProduct(
                        desired - s.angular_velocity)) +
                    s.angular_velocity.cross(
                        plant_.inertia.cwiseProduct(s.angular_velocity));
    result.moment = result.moment.cwiseMax(-config_.max_moment)
                        .cwiseMin(config_.max_moment);
    return result;
  }
  double RateLoop::derivative(const State &state, const Command &command) const
  {
    validate_state(state);
    validate_command(command);
    const double target =
        std::clamp(command.specific_force, 0.0, config_.max_specific_force);
    return (target - state.motors.specific_force) / config_.thrust_tau_s;
  }
}  // namespace simple_sim
