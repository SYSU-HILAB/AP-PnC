#include "simple_sim/rate_loop.hpp"

#include <algorithm>
#include <utility>

namespace simple_sim
{
  RateLoop::RateLoop(PlantConfig plant, RateLoopConfig config)
      : plant_(std::move(plant)), config_(std::move(config))
  {
    plant_.validate();
    config_.validate();
  }
  Wrench RateLoop::operator()(const State &s, const Command &c) const
  {
    validate_state(s);
    validate_command(c);
    Wrench result;
    result.force.z() = plant_.mass * std::clamp(c.specific_force, 0.0,
                                                config_.max_specific_force);
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
}  // namespace simple_sim
