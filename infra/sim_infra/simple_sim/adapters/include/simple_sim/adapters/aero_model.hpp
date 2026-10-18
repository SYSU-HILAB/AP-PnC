#pragma once

#include <aerodynamics/aero_interface.hpp>
#include <memory>

#include "simple_sim/plant.hpp"

namespace simple_sim
{

  struct AeroConfig
  {
    Eigen::Vector3d wind_world = Eigen::Vector3d::Zero();  // ENU, m/s
    double          scale      = 1.0;
    // Keep only the lateral (body-Y) aerodynamic moment. Body-X and body-Z are
    // forced to zero, matching the paper's neglect of the roll/yaw aerodynamic
    // moment (M_a = 0 in Eq. 26). Explicit, not silent: see simple_sim.yaml.
    bool lateral_moment_only = false;
    // Inject the aerodynamic force along the thrust axis (body +Z)?
    // -1 = follow the profile rule (the ideal profile's first-order thrust loop
    // already closes that axis, so the force is not injected a second time),
    // 0 = never inject, 1 = always inject. The controller's feedforward model
    // reads the same value, so plant and prediction cannot diverge.
    int inject_thrust_axis = -1;
  };

  // Rotate ENU air-relative velocity into body FLU; models return FLU wrenches.
  // No model-specific body-frame conversion is performed.
  ForceModel tailsitter_aero(
      std::shared_ptr<aerodynamics::AerodynamicsInterface> model,
      AeroConfig                                           config);

}  // namespace simple_sim
