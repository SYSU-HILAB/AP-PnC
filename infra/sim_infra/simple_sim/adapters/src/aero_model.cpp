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
    return [model = std::move(model), cfg](const State &state)
    {
      const Eigen::Vector3d body_velocity =
          state.attitude.conjugate() * (state.velocity - cfg.wind_world);
      Eigen::Vector3d force  = Eigen::Vector3d::Zero(),
                      moment = Eigen::Vector3d::Zero();
      double          alpha = 0.0, beta = 0.0;
      if (!model->getAeroWrench(body_velocity, force, moment, alpha, beta) ||
          !force.allFinite() || !moment.allFinite() || !std::isfinite(alpha) ||
          !std::isfinite(beta))
        throw std::runtime_error("aerodynamics evaluation failed");
      moment *= cfg.scale;
      if (cfg.lateral_moment_only)
      {
        // Body-X (wing normal) and body-Z (thrust axis) aerodynamic moments are
        // dropped; only the lateral pitching moment survives.
        moment.x() = 0.0;
        moment.z() = 0.0;
      }
      return Wrench{cfg.scale * force, moment,
                    AeroObservation{body_velocity, alpha, beta, true}};
    };
  }
}  // namespace simple_sim
