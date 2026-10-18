/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <spdlog/spdlog.h>

#include <aerodynamics/non_aerodynamics.hpp>

namespace aerodynamics
{
  bool NonAerodynamics::getAeroWrench(const Eigen::Vector3d &body_speed,
                                      Eigen::Vector3d       &aero_force_b,
                                      Eigen::Vector3d       &aero_moment_b,
                                      double &alpha, double &beta)
  {
    std::tie(alpha, beta) = air_angles(body_speed);

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
