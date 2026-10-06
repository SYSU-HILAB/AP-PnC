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
   * @brief Implementation of zero aerodynamics (no forces/moments)
   *
   * This class implements a null aerodynamics model that returns zero forces
   * and moments. Useful for testing or when aerodynamics should be disabled.
   */
  class NonAerodynamics : public AerodynamicsInterface
  {
   public:
    NonAerodynamics()  = default;
    ~NonAerodynamics() = default;

    /**
     * @brief Returns zero forces and moments, but calculates alpha/beta
     *
     * @param[in] body_speed Velocity vector in body frame (m/s)
     * @param[out] aero_force_b Will be set to zero vector
     * @param[out] aero_moment_b Will be set to zero vector
     * @param[out] alpha Angle of attack (rad), still calculated
     * @param[out] beta Sideslip angle (rad), still calculated
     * @return true Always returns true
     */
    bool getAeroWrench(const Eigen::Vector3d& body_speed,
                       Eigen::Vector3d&       aero_force_b,
                       Eigen::Vector3d& aero_moment_b, double& alpha,
                       double& beta) override;
  };

  /**
   * @brief Factory for creating NonAerodynamics instances
   */
  class NonAerodynamicsCreator : public AerodynamicsCreator
  {
   public:
    bool createAerodynamics() override;
    ~NonAerodynamicsCreator() = default;
  };

}  // namespace aerodynamics
