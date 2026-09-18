/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#ifndef SIMPLE_SIM_DYNAMICS_HPP_
#define SIMPLE_SIM_DYNAMICS_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "aerodynamics/aero_interface.hpp"

namespace simple_sim
{

  struct AeroConfig
  {
    bool            enable           = true;
    std::string     model            = "lyu";         // "lyu", "phi", "none"
    std::string     frame_convention = "tailsitter";  // "tailsitter" or "standard"
    double          scale            = 1.0;
    bool            omit_moment      = false;
    bool            omit_force       = false;
    Eigen::Vector3d wind_world       = Eigen::Vector3d::Zero();
  };

  struct SimState
  {
    Eigen::Vector3d    position         = Eigen::Vector3d::Zero();
    Eigen::Vector3d    velocity         = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude         = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    angular_velocity = Eigen::Vector3d::Zero();

    // Aerodynamic wrench and angles in body FLU frame
    Eigen::Vector3d aero_force_flu  = Eigen::Vector3d::Zero();
    Eigen::Vector3d aero_moment_flu = Eigen::Vector3d::Zero();
    double          alpha           = 0.0;
    double          beta            = 0.0;
  };

  /**
   * Flight dynamics integrating rigid-body physics and aerodynamic models.
   *
   * @param[in]  mass        Vehicle mass [kg]
   * @param[in]  inertia     Diagonal inertia tensor [kg*m^2]
   * @param[in]  gravity     Gravitational acceleration magnitude [m/s^2]
   * @param[in]  aero        Aerodynamics model (AerodynamicsInterface)
   * @param[in]  aero_cfg    Aerodynamic settings
   */
  class Dynamics
  {
   public:
    Dynamics(double mass, const Eigen::Matrix3d &inertia, double gravity,
             std::shared_ptr<aerodynamics::AerodynamicsInterface> aero     = nullptr,
             const AeroConfig                                    &aero_cfg = {});

    /**
     * Integrate one time step with Euler discretization.
     * Computes aerodynamic forces & moments and combines them with thrust.
     *
     * @param[in]  dt           Time step [s]
     * @param[in]  body_thrust  Body-frame thrust force [N]
     * @param[in]  body_moment  Body-frame control moment [N*m]
     * @param[inout] state      Vehicle state, advanced by dt
     */
    void step(double dt, const Eigen::Vector3d &body_thrust,
              const Eigen::Vector3d &body_moment, SimState *state) const;

    const AeroConfig &aeroConfig() const { return aero_cfg_; }
    void              setAeroConfig(const AeroConfig &cfg) { aero_cfg_ = cfg; }
    void setAerodynamics(std::shared_ptr<aerodynamics::AerodynamicsInterface> aero)
    {
      aero_ = std::move(aero);
    }

   private:
    double                                               mass_;
    Eigen::Matrix3d                                      inertia_;
    Eigen::Matrix3d                                      inertia_inv_;
    double                                               gravity_;
    std::shared_ptr<aerodynamics::AerodynamicsInterface> aero_;
    AeroConfig                                           aero_cfg_;
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_DYNAMICS_HPP_
