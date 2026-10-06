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

#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

#include "Zhang/Aerodynamics.h"

namespace aerodynamics
{

  bool LyuAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                      Eigen::Vector3d&       aero_force_b,
                                      Eigen::Vector3d&       aero_moment_b,
                                      double& alpha, double& beta)
  {
    double v_b[3] = {body_speed(0), body_speed(1), body_speed(2)};
    double f_aero_b[3];
    double moment_aero_b[3];

    Aerodynamics(v_b, f_aero_b, moment_aero_b, &alpha, &beta);

    aero_force_b = Eigen::Vector3d(f_aero_b[0], f_aero_b[1], f_aero_b[2]);
    aero_moment_b =
        Eigen::Vector3d(moment_aero_b[0], moment_aero_b[1], moment_aero_b[2]);
    return true;
  }

  bool LyuAerodynamicsCreator::createAerodynamics()
  {
    std::unique_ptr<LyuAerodynamics> instance =
        std::make_unique<LyuAerodynamics>();

    _instance = std::move(instance);
    return true;
  }

};  // namespace aerodynamics
