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

/*
 * Created on Wed June 01 2025
 *
 * Copyright (c) 2025 SYSU
 * Author: WarriorHanamy ;
 * Email: rongerch@outlook.com; rongerch@outlook.com
 *
 */

#include <basic_trajectories/vertical_loop.hpp>

namespace basic_trajectories
{

  VerticalLoop::VerticalLoop(double radius_in, double angular_speed_in,
                             double height_in)
  {
    radius        = radius_in;
    angular_speed = angular_speed_in;  // Keep positive
    height        = height_in;
    type          = basic_trajectories::Type::INIT;
  }

  Vec3E VerticalLoop::pos(double t)
  {
    Vec3E pos = Vec3E::Zero();
    // Vertical circle in X-Z plane (CLOCKWISE, starts at bottom and moves
    // UP)
    double phase_shift = -M_PI / 2;  // Start at bottom (X=0, Z=height-R)
    pos[0]             = radius *
             cos(-angular_speed * t + phase_shift);  // X = R * cos(-ωt - π/2)
    pos[1] = 0.0;  // Y = 0 (no lateral motion)
    pos[2] =
        height + radius * sin(-angular_speed * t +
                              phase_shift);  // Z = height + R * sin(-ωt - π/2)
    return pos;
  }

  Vec3E VerticalLoop::vel(double t)
  {
    Vec3E vel = Vec3E::Zero();
    // Velocity in X-Z plane (UPWARD initial motion)
    double phase_shift = -M_PI / 2;
    vel[0] =
        radius * angular_speed *
        sin(-angular_speed * t + phase_shift);  // dX/dt = Rω sin(-ωt - π/2)
    vel[2] =
        -radius * angular_speed *
        cos(-angular_speed * t + phase_shift);  // dZ/dt = -Rω cos(-ωt - π/2)
    return vel;
  }

  Vec3E VerticalLoop::acc(double t)
  {
    Vec3E acc = Vec3E::Zero();

    return acc;
  }

  Vec3E VerticalLoop::jerk(double t)
  {
    Vec3E jerk = Vec3E::Zero();

    return jerk;
  }

}  // namespace basic_trajectories
