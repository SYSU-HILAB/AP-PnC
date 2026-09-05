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

#include "simple_sim/controller.hpp"

#include <cmath>

namespace simple_sim
{

  Controller::Controller(const ControlGains &gains, double mass, double gravity)
      : gains_(gains), mass_(mass), gravity_(gravity)
  {
  }

  void Controller::compute(const SimState        &state,
                           const Eigen::Vector3d &pos_des,
                           const Eigen::Vector3d &vel_des, double dt,
                           Eigen::Vector3d *thrust_body,
                           Eigen::Vector3d *moment_body)
  {
    const Eigen::Vector3d gravity_vec(0.0, 0.0, -gravity_);
    const Eigen::Matrix3d R = state.attitude.toRotationMatrix();

    // Position error and integrator (anti-windup: clamp the integral
    // contribution to +/-max_ia [m/s^2] so it can only cancel residual error).
    const Eigen::Vector3d err_pos = pos_des - state.position;
    const Eigen::Vector3d err_vel = vel_des - state.velocity;
    integral_ += err_pos * dt;
    const double max_ia    = 5.0;
    const double ki_max    = std::max(gains_.ki_pos.maxCoeff(), 1e-9);
    const double int_limit = max_ia / ki_max;
    for (int axis = 0; axis < 3; ++axis)
    {
      integral_[axis] = std::clamp(integral_[axis], -int_limit, int_limit);
    }

    // Desired total force in world frame: thrust must produce the PD
    // acceleration on top of counteracting gravity (a_des = kp*e + kd*e_dot
    // + ki*int, force = m*(a_des - g_vec)). Hovering gives force = +mg.
    Eigen::Vector3d force_des =
        mass_ * (gains_.kp_pos.cwiseProduct(err_pos) +
                 gains_.kd_vel.cwiseProduct(err_vel) +
                 gains_.ki_pos.cwiseProduct(integral_) - gravity_vec);

    // Single-sided thrust: the body z-axis can only push. A downward demand
    // reduces to free-fall rather than flipping the vehicle upside-down.
    if (force_des.z() < 0.0)
    {
      force_des.z() = 0.0;
    }

    // Map the force to body frame; the body z-axis is the thrust direction.
    *thrust_body = R.transpose() * force_des;

    // Desired attitude: align body z with the desired force direction.
    Eigen::Matrix3d R_des  = R;
    const double    f_norm = force_des.norm();
    if (f_norm > 1e-6)
    {
      const Eigen::Vector3d z_des = force_des / f_norm;
      Eigen::Vector3d x_des = Eigen::Vector3d::UnitX() - z_des * z_des.x();
      if (x_des.norm() < 1e-6)
      {
        x_des = Eigen::Vector3d::UnitY() - z_des * z_des.y();
      }
      x_des.normalize();
      const Eigen::Vector3d y_des = z_des.cross(x_des);
      R_des.col(0)                = x_des;
      R_des.col(1)                = y_des;
      R_des.col(2)                = z_des;
    }

    // Proportional attitude loop: rotation error expressed in body frame.
    const Eigen::Matrix3d   R_err = R_des * R.transpose();
    const Eigen::AngleAxisd aa(R_err);
    const Eigen::Vector3d error_body = R.transpose() * (aa.axis() * aa.angle());
    *moment_body                     = gains_.kp_att.cwiseProduct(error_body);
  }

}  // namespace simple_sim
