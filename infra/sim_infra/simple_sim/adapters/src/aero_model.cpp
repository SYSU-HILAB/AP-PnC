#include "simple_sim/adapters/aero_model.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace simple_sim
{
  ForceModel tailsitter_aero(
      std::shared_ptr<aerodynamics::AerodynamicsInterface> model,
      AeroConfig                                           cfg)
  {
    if (!model || !cfg.wind_world.allFinite() || !std::isfinite(cfg.scale) ||
        cfg.scale < 0.0)
      throw std::invalid_argument("invalid aerodynamics configuration");
    return [model = std::move(model), cfg](const State &s)
    {
      const Eigen::Vector3d v =
          s.attitude.conjugate() * (s.velocity - cfg.wind_world);
      const Eigen::Vector3d wing_velocity(v.z(), -v.y(), v.x());
      Eigen::Vector3d       force = Eigen::Vector3d::Zero(),
                      moment      = Eigen::Vector3d::Zero();
      double alpha = 0.0, beta = 0.0;
      if (!model->getAeroWrench(wing_velocity, force, moment, alpha, beta) ||
          !force.allFinite() || !moment.allFinite() || !std::isfinite(alpha) ||
          !std::isfinite(beta))
        throw std::runtime_error("aerodynamics evaluation failed");
      return Wrench{
          cfg.scale * Eigen::Vector3d(force.z(), -force.y(), force.x()),
          cfg.scale * Eigen::Vector3d(moment.z(), -moment.y(), moment.x())};
    };
  }
}  // namespace simple_sim
