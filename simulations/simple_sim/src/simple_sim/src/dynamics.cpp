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
                     double gravity)
      : mass_(mass),
        inertia_(inertia),
        inertia_inv_(inertia.inverse()),
        gravity_(gravity)
  {
  }

  void Dynamics::step(double dt, const Eigen::Vector3d &body_thrust,
                      const Eigen::Vector3d &body_moment, SimState *state) const
  {
    const Eigen::Vector3d gravity_vec(0.0, 0.0, -gravity_);
    const Eigen::Matrix3d R = state->attitude.toRotationMatrix();

    // Translational dynamics in world frame.
    const Eigen::Vector3d accel_world = gravity_vec + R * body_thrust / mass_;
    state->velocity += accel_world * dt;
    state->position += state->velocity * dt;

    // Rotational dynamics: body-frame Euler equation with quaternion
    // kinematics.
    const Eigen::Vector3d &w = state->angular_velocity;
    const Eigen::Vector3d  alpha_body =
        inertia_inv_ * (body_moment - w.cross(inertia_ * w));
    state->angular_velocity += alpha_body * dt;

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
