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
  };

  // Zhang-Lyu uses the wing FRD frame: (x,y,z)_wing = (z,-y,x)_FLU.
  ForceModel tailsitter_aero(
      std::shared_ptr<aerodynamics::AerodynamicsInterface> model,
      AeroConfig                                           config);

}  // namespace simple_sim
