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

#pragma once
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{

  /**
   * @brief Phi-theory based aerodynamics model
   *
   * This class implements aerodynamic forces using a linear mapping (phi
   * matrix) between body velocities and aerodynamic forces. The phi matrix
   * encapsulates the linear aerodynamic coefficients that relate velocity to
   * force.
   *
   * The force model is of the form:
   * F = Φ * v * |v|
   * where:
   * - F is the aerodynamic force vector
   * - Φ is the 3x3 phi matrix of aerodynamic coefficients
   * - v is the body velocity vector
   * - |v| is the magnitude of the velocity
   */
  class PhiAerodynamics : public AerodynamicsInterface
  {
   private:
    /// Matrix of aerodynamic coefficients mapping velocity to force
    Eigen::Matrix3d _phi_matrix;

   public:
    PhiAerodynamics() = default;

    /**
     * @brief Set the phi matrix of aerodynamic coefficients
     *
     * @param phi_matrix 3x3 matrix of aerodynamic coefficients
     * @return true Always returns true
     *
     * @note The phi matrix should be properly scaled to account for
     * air density, reference area, and mass properties.
     */
    bool setPhi(const Eigen::Matrix3d& phi_matrix)
    {
      _phi_matrix = phi_matrix;
      return true;
    }

    /**
     * @brief Compute aerodynamic forces using the phi-theory model
     *
     * Forces are computed using a linear mapping between body velocities
     * and aerodynamic forces, scaled by the velocity magnitude. Moments
     * are currently set to zero.
     *
     * @param[in] body_speed Velocity vector in body frame (m/s)
     * @param[out] aero_force_b Aerodynamic force vector in body frame (N)
     * @param[out] aero_moment_b Aerodynamic moment vector in body frame
     * (N⋅m)
     * @param[out] alpha Angle of attack (rad)
     * @param[out] beta Sideslip angle (rad)
     * @return true Always returns true
     *
     * @note All vectors are expressed in the body frame following the
     * Forward-Right-Down (FRD) convention
     */
    bool getAeroWrench(const Eigen::Vector3d& body_speed,
                       Eigen::Vector3d&       aero_force_b,
                       Eigen::Vector3d& aero_moment_b, double& alpha,
                       double& beta) override;

    ~PhiAerodynamics() = default;
  };

  /**
   * @brief Factory for creating PhiAerodynamics instances
   *
   * Creates and initializes PhiAerodynamics objects with parameters
   * loaded from configuration. The phi matrix is loaded from a database
   * and properly scaled before being applied to the aerodynamics model.
   */
  class PhiAerodynamicsCreator : public AerodynamicsCreator
  {
   public:
    /**
     * @brief Create and initialize a PhiAerodynamics instance
     *
     * Loads phi matrix values from configuration database and creates
     * a new PhiAerodynamics instance with those values.
     *
     * @return true if creation and initialization successful
     * @return false if database access fails
     */
    bool createAerodynamics() override;

    ~PhiAerodynamicsCreator() = default;
  };

}  // namespace aerodynamics
