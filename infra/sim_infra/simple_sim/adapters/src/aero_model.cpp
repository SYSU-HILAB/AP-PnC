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
      const Eigen::Vector3d wing_velocity = wing_from_flu(
          state.attitude.conjugate() * (state.velocity - cfg.wind_world));
      Eigen::Vector3d force_wing  = Eigen::Vector3d::Zero(),
                      moment_wing = Eigen::Vector3d::Zero();
      double alpha = 0.0, beta = 0.0;
      if (!model->getAeroWrench(wing_velocity, force_wing, moment_wing, alpha,
                                beta) ||
          !force_wing.allFinite() || !moment_wing.allFinite() ||
          !std::isfinite(alpha) || !std::isfinite(beta))
        throw std::runtime_error("aerodynamics evaluation failed");
      Eigen::Vector3d moment_flu = cfg.scale * flu_from_wing(moment_wing);
      if (cfg.lateral_moment_only)
      {
        // Body-X (wing normal) and body-Z (thrust axis) aerodynamic moments are
        // dropped; only the lateral pitching moment survives.
        moment_flu.x() = 0.0;
        moment_flu.z() = 0.0;
      }
      return Wrench{cfg.scale * flu_from_wing(force_wing), moment_flu,
                    AeroObservation{wing_velocity, alpha, beta, true}};
    };
  }
}  // namespace simple_sim
