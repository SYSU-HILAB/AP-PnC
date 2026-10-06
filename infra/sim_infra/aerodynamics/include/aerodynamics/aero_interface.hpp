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
#include <Eigen/Dense>
#include <memory>     // For std::unique_ptr
#include <stdexcept>  // For std::runtime_error

namespace aerodynamics
{

  class AerodynamicsInterface
  {
   public:
    /**
     * @brief Compute aerodynamic forces and moments
     *
     * @param[in] body_speed Velocity vector in body frame (m/s)
     * @param[out] aero_force_b Aerodynamic force vector in body frame (N)
     * @param[out] aero_moment_b Aerodynamic moment vector in body frame
     * (N⋅m)
     * @param[out] alpha Angle of attack (rad)
     * @param[out] beta Sideslip angle (rad)
     * @return true if computation successful
     */
    virtual bool getAeroWrench(const Eigen::Vector3d& body_speed,
                               Eigen::Vector3d&       aero_force_b,
                               Eigen::Vector3d& aero_moment_b, double& alpha,
                               double& beta) = 0;

    virtual ~AerodynamicsInterface() = default;
  };

  /**
   * @brief Factory base class for creating aerodynamics model instances
   */
  class AerodynamicsCreator
  {
   protected:
    std::unique_ptr<AerodynamicsInterface> _instance;

   public:
    /**
     * @brief Create and initialize an aerodynamics model instance
     *
     * @return true if creation and initialization successful
     */
    virtual bool createAerodynamics() = 0;

    /**
     * @brief Compute aerodynamic forces using the created model instance
     *
     * @param[in] body_speed Velocity vector in body frame (m/s)
     * @param[out] aero_force_b Aerodynamic force vector in body frame (N)
     * @param[out] aero_moment_b Aerodynamic moment vector in body frame
     * (N⋅m)
     * @param[out] alpha Angle of attack (rad)
     * @param[out] beta Sideslip angle (rad)
     * @return true if computation successful
     * @throws std::runtime_error if aerodynamics instance not created
     */
    bool getAeroWrench(const Eigen::Vector3d& body_speed,
                       Eigen::Vector3d&       aero_force_b,
                       Eigen::Vector3d& aero_moment_b, double& alpha,
                       double& beta)
    {
      if (!_instance)
      {
        throw std::runtime_error("Aerodynamics instance not created");
      }
      return _instance->getAeroWrench(body_speed, aero_force_b, aero_moment_b,
                                      alpha, beta);
    }

    /**
     * @brief Release ownership of the created model instance
     *
     * Used by aerodynamics::make_aero() to hand the instance to callers
     * that want the interface, not the creator.
     */
    std::unique_ptr<AerodynamicsInterface> takeInstance()
    {
      return std::move(_instance);
    }

    virtual ~AerodynamicsCreator() = default;
  };

}  // namespace aerodynamics
