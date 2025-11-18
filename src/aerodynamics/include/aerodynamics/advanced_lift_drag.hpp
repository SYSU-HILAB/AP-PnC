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
#include <memory>

namespace aerodynamics
{
  /**
   * @brief Advanced lift and drag based aerodynamics model
   *
   * This class implements aerodynamic forces using separate lift and drag
   * coefficients that vary with angle of attack. The model accounts for both
   * linear and nonlinear aerodynamic effects in the normal and axial
   * directions.
   */
  class AdvancedLiftDrag : public AerodynamicsInterface
  {
   private:
    class AdvancedLiftDragDataPrivate;
    std::unique_ptr<AdvancedLiftDragDataPrivate> pimpl_;

   public:
    /**
     * @brief Construct a new Advanced Lift Drag object
     */
    AdvancedLiftDrag();

    /**
     * @brief Compute aerodynamic forces using advanced lift-drag model
     *
     * Forces are computed using lift and drag coefficients that vary with
     * angle of attack, including stall effects. The model accounts for both
     * parasitic and induced drag components.
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

    // Need to declare destructor in header but define in cpp where
    // implementation is complete
    ~AdvancedLiftDrag();

    // Delete copy operations since we're using unique_ptr
    AdvancedLiftDrag(const AdvancedLiftDrag&)            = delete;
    AdvancedLiftDrag& operator=(const AdvancedLiftDrag&) = delete;

    // Allow move operations
    AdvancedLiftDrag(AdvancedLiftDrag&&) noexcept;
    AdvancedLiftDrag& operator=(AdvancedLiftDrag&&) noexcept;
  };

  /**
   * @brief Factory for creating AdvancedLiftDrag instances
   *
   * Creates and initializes AdvancedLiftDrag objects with parameters
   * loaded from configuration.
   */
  class AdvancedLiftDragCreator : public AerodynamicsCreator
  {
   public:
    /**
     * @brief Create and initialize an AdvancedLiftDrag instance
     *
     * Loads aerodynamic parameters from configuration database and creates
     * a new AdvancedLiftDrag instance with those values.
     *
     * @return true if creation and initialization successful
     * @return false if database access fails
     */
    bool createAerodynamics() override;

    ~AdvancedLiftDragCreator() = default;
  };

}  // namespace aerodynamics
