/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#include <spdlog/spdlog.h>

#include <aerodynamics/non_aerodynamics.hpp>

namespace aerodynamics
{
  bool NonAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                      Eigen::Vector3d&       aero_force_b,
                                      Eigen::Vector3d&       aero_moment_b,
                                      double& alpha, double& beta)
  {
    // Calculate alpha and beta only when speed is above threshold
    constexpr double vThreshold = 5e-1;
    if (body_speed.norm() < vThreshold)
    {
      alpha = 0.0;
      beta  = 0.0;
    }
    else
    {
      alpha = std::atan2(body_speed(2), body_speed(0));
      beta  = std::asin(body_speed(1) / body_speed.norm());
    }

    // Set forces and moments to zero
    aero_force_b  = Eigen::Vector3d::Zero();
    aero_moment_b = Eigen::Vector3d::Zero();

    return true;
  }

  bool NonAerodynamicsCreator::createAerodynamics()
  {
    std::unique_ptr<NonAerodynamics> instance =
        std::make_unique<NonAerodynamics>();

    _instance = std::move(instance);
    return true;
  }

}  // namespace aerodynamics
