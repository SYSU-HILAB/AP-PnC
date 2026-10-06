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

#pragma once
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{
  /**
   * @brief Implementation of Zhang-Lyu aerodynamics model
   *
   * This class implements the aerodynamics model based on the work by Zhang
   * and Lyu. It uses a MATLAB-generated implementation for force and moment
   * calculations.
   */
  class LyuAerodynamics : public AerodynamicsInterface
  {
   public:
    LyuAerodynamics() = default;

    /**
     * @brief Compute aerodynamic forces using Zhang-Lyu model
     *
     * Forces and moments are computed using a MATLAB-generated
     * implementation that takes into account complex aerodynamic effects.
     *
     * @param[in] body_speed Velocity vector in body frame (m/s)
     * @param[out] aero_force_b Aerodynamic force vector in body frame (N)
     * @param[out] aero_moment_b Aerodynamic moment vector in body frame
     * (N⋅m)
     * @param[out] alpha Angle of attack (rad)
     * @param[out] beta Sideslip angle (rad)
     * @return true if computation successful
     */
    bool getAeroWrench(const Eigen::Vector3d& body_speed,
                       Eigen::Vector3d&       aero_force_b,
                       Eigen::Vector3d& aero_moment_b, double& alpha,
                       double& beta) override;
    ~LyuAerodynamics() = default;
  };

  /**
   * @brief Factory for creating LyuAerodynamics instances
   */
  class LyuAerodynamicsCreator : public AerodynamicsCreator
  {
   public:
    bool createAerodynamics() override;
    ~LyuAerodynamicsCreator() = default;
  };
};  // namespace aerodynamics
