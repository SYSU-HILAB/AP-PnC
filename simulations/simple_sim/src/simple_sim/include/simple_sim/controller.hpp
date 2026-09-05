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

#ifndef SIMPLE_SIM_CONTROLLER_HPP_
#define SIMPLE_SIM_CONTROLLER_HPP_

#include <Eigen/Dense>

#include "simple_sim/dynamics.hpp"

namespace simple_sim
{

  struct ControlGains
  {
    Eigen::Vector3d kp_pos = Eigen::Vector3d::Constant(12.0);
    Eigen::Vector3d kd_vel = Eigen::Vector3d::Constant(3.0);
    Eigen::Vector3d ki_pos = Eigen::Vector3d::Constant(0.5);
    Eigen::Vector3d kp_att = Eigen::Vector3d::Constant(8.0);
  };

  /**
   * Simplified outer position / inner attitude controller producing
   * body-frame thrust and moment commands. The thrust direction is
   * single-sided (body +z only); downward commands are clamped to zero
   * rather than flipping the vehicle.
   *
   * @param[in] gains  Controller gains [1/s^2, 1/s, 1/s, 1/s]
   * @param[in] mass   Vehicle mass [kg]
   * @param[in] gravity Gravitational acceleration magnitude [m/s^2]
   */
  class Controller
  {
   public:
    Controller(const ControlGains &gains, double mass, double gravity);

    /**
     * Compute control command for the current state.
     *
     * @param[in]  state      Current vehicle state
     * @param[in]  pos_des    Desired position [m]
     * @param[in]  vel_des    Desired velocity [m/s]
     * @param[in]  dt         Control time step [s]
     * @param[out] thrust_body Body-frame force command [N]
     * @param[out] moment_body Body-frame moment command [N*m]
     */
    void compute(const SimState &state, const Eigen::Vector3d &pos_des,
                 const Eigen::Vector3d &vel_des, double dt,
                 Eigen::Vector3d *thrust_body, Eigen::Vector3d *moment_body);

    void resetIntegral() { integral_ = Eigen::Vector3d::Zero(); }

   private:
    ControlGains    gains_;
    double          mass_;
    double          gravity_;
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_CONTROLLER_HPP_
