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

#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

#include "Zhang/Aerodynamics.h"

namespace aerodynamics
{

  bool LyuAerodynamics::getAeroWrench(const Eigen::Vector3d &body_speed,
                                      Eigen::Vector3d       &aero_force_b,
                                      Eigen::Vector3d       &aero_moment_b,
                                      double &alpha, double &beta)
  {
    std::tie(alpha, beta) = air_angles(body_speed);
    const double speed    = body_speed.norm();
    const double fit_beta = speed >= 1e-4 ? beta : 0.0;
    double       f_aero_b[3];
    double       moment_aero_b[3];

    AerodynamicsFlu(speed, alpha, fit_beta, f_aero_b, moment_aero_b);

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
