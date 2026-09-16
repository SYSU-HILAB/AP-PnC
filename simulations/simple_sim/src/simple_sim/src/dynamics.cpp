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

#include "simple_sim/dynamics.hpp"

#include <cmath>

namespace simple_sim
{

  Dynamics::Dynamics(double mass, const Eigen::Matrix3d &inertia,
                     double gravity,
                     std::shared_ptr<aerodynamics::AerodynamicsInterface> aero,
                     const AeroConfig &aero_cfg)
      : mass_(mass),
        inertia_(inertia),
        inertia_inv_(inertia.inverse()),
        gravity_(gravity),
        aero_(std::move(aero)),
        aero_cfg_(aero_cfg)
  {
  }

  void Dynamics::step(double dt, const Eigen::Vector3d &body_thrust,
                      const Eigen::Vector3d &body_moment, SimState *state) const
  {
    const Eigen::Vector3d gravity_vec(0.0, 0.0, -gravity_);
    const Eigen::Matrix3d R = state->attitude.toRotationMatrix();

    Eigen::Vector3d aero_f_flu = Eigen::Vector3d::Zero();
    Eigen::Vector3d aero_m_flu = Eigen::Vector3d::Zero();
    double alpha = 0.0;
    double beta  = 0.0;

    if (aero_ && aero_cfg_.enable)
    {
      // Relative velocity in world frame (considering wind)
      const Eigen::Vector3d v_rel_world =
          state->velocity - aero_cfg_.wind_world;
      // Body velocity in FLU frame
      const Eigen::Vector3d v_body_flu = R.transpose() * v_rel_world;

      // Convert body velocity to aerodynamic model frame (FRD)
      Eigen::Vector3d v_aero = Eigen::Vector3d::Zero();
      if (aero_cfg_.frame_convention == "tailsitter")
      {
        // Tailsitter wing frame (FRD):
        // x_wing (chord forward / rotor thrust direction) = z_flu
        // y_wing (right wing)                             = -y_flu
        // z_wing (down / belly)                           = x_flu
        v_aero << v_body_flu.z(), -v_body_flu.y(), v_body_flu.x();
      }
      else
      {
        // Standard quad FLU -> FRD (180 deg roll around X)
        v_aero << v_body_flu.x(), -v_body_flu.y(), -v_body_flu.z();
      }

      // Guard airspeed against extreme magnitude to prevent polynomial overflow
      const double v_norm = v_aero.norm();
      if (v_norm > 80.0)
      {
        v_aero *= (80.0 / v_norm);
      }

      Eigen::Vector3d f_aero_b = Eigen::Vector3d::Zero();
      Eigen::Vector3d m_aero_b = Eigen::Vector3d::Zero();
      aero_->getAeroWrench(v_aero, f_aero_b, m_aero_b, alpha, beta);

      // Numerical safety guard against non-finite values
      if (!f_aero_b.allFinite() || !m_aero_b.allFinite() || !std::isfinite(alpha) || !std::isfinite(beta))
      {
        f_aero_b.setZero();
        m_aero_b.setZero();
        alpha = 0.0;
        beta  = 0.0;
      }

      f_aero_b *= aero_cfg_.scale;
      m_aero_b *= aero_cfg_.scale;

      if (aero_cfg_.omit_force)
      {
        f_aero_b.setZero();
      }
      if (aero_cfg_.omit_moment)
      {
        m_aero_b.setZero();
      }

      // Convert aerodynamic wrench from aero FRD frame back to FLU
      if (aero_cfg_.frame_convention == "tailsitter")
      {
        aero_f_flu << f_aero_b.z(), -f_aero_b.y(), f_aero_b.x();
        aero_m_flu << m_aero_b.z(), -m_aero_b.y(), m_aero_b.x();
      }
      else
      {
        aero_f_flu << f_aero_b.x(), -f_aero_b.y(), -f_aero_b.z();
        aero_m_flu << m_aero_b.x(), -m_aero_b.y(), -m_aero_b.z();
      }
    }

    state->aero_force_flu  = aero_f_flu;
    state->aero_moment_flu = aero_m_flu;
    state->alpha           = alpha;
    state->beta            = beta;

    // Translational dynamics in world frame: gravity + body thrust + aero force
    const Eigen::Vector3d total_body_force = body_thrust + aero_f_flu;
    const Eigen::Vector3d accel_world =
        gravity_vec + R * total_body_force / mass_;
    state->velocity += accel_world * dt;
    state->position += state->velocity * dt;

    // Rotational dynamics: body-frame Euler equation with total moment
    const Eigen::Vector3d total_body_moment = body_moment + aero_m_flu;
    const Eigen::Vector3d &w                = state->angular_velocity;
    const Eigen::Vector3d  alpha_body =
        inertia_inv_ * (total_body_moment - w.cross(inertia_ * w));
    state->angular_velocity += alpha_body * dt;

    // Quaternion kinematics
    Eigen::Quaterniond q_dot;
    q_dot.w() =
        -0.5 * (w.x() * state->attitude.x() + w.y() * state->attitude.y() +
                w.z() * state->attitude.z());
    q_dot.x() =
        0.5 * (w.x() * state->attitude.w() + w.y() * state->attitude.z() -
               w.z() * state->attitude.y());
    q_dot.y() =
        0.5 * (w.y() * state->attitude.w() - w.x() * state->attitude.z() +
               w.z() * state->attitude.x());
    q_dot.z() =
        0.5 * (w.z() * state->attitude.w() + w.x() * state->attitude.y() -
               w.y() * state->attitude.x());
    state->attitude.w() += q_dot.w() * dt;
    state->attitude.x() += q_dot.x() * dt;
    state->attitude.y() += q_dot.y() * dt;
    state->attitude.z() += q_dot.z() * dt;
    state->attitude.normalize();
  }

}  // namespace simple_sim
