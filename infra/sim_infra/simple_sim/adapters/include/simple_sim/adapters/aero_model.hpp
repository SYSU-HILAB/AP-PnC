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
  };

  // Zhang-Lyu uses the wing FRD frame: (x,y,z)_wing = (z,-y,x)_FLU.
  // The single definition of that mapping lives in simple_sim::wing_from_flu /
  // simple_sim::flu_from_wing (core/include/simple_sim/types.hpp).
  ForceModel tailsitter_aero(
      std::shared_ptr<aerodynamics::AerodynamicsInterface> model,
      AeroConfig                                           config);

}  // namespace simple_sim
