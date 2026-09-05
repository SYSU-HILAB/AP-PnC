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

namespace simple_sim
{

  struct SimState
  {
    Eigen::Vector3d    position         = Eigen::Vector3d::Zero();
    Eigen::Vector3d    velocity         = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude         = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    angular_velocity = Eigen::Vector3d::Zero();
  };

  /**
   * Simplified rigid-body flight dynamics (thrust along body z-axis).
   *
   * @param[in]  mass     Vehicle mass [kg]
   * @param[in]  inertia  Diagonal inertia tensor [kg*m^2]
   * @param[in]  gravity  Gravitational acceleration magnitude [m/s^2]
   */
  class Dynamics
  {
   public:
    Dynamics(double mass, const Eigen::Matrix3d &inertia, double gravity);

    /**
     * Integrate one time step with Euler discretization.
     *
     * @param[in]  dt           Time step [s]
     * @param[in]  body_thrust  Body-frame force [N]
     * @param[in]  body_moment  Body-frame moment [N*m]
     * @param[inout] state      Vehicle state, advanced by dt
     */
    void step(double dt, const Eigen::Vector3d &body_thrust,
              const Eigen::Vector3d &body_moment, SimState *state) const;

   private:
    double          mass_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    double          gravity_;
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_DYNAMICS_HPP_
