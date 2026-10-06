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
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * Author: WarriorHanamy ; Email: rongerch@outlook.com
 */

#include <basic_trajectories/sin.hpp>

namespace basic_trajectories
{
  Sin::Sin(double a_in, double wave_len_in, double height_in)
  {
    radius        = a_in;
    height        = height_in;
    wave_len      = wave_len_in;
    angular_speed = 8 * M_PI / wave_len;
    type          = basic_trajectories::Type::INIT;
  }

  // y = radius * np.sin(2 * np.pi / width_radius * x + phi)
  // change to position
  Vec3E Sin::pos(double t)
  {
    Vec3E pos = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    pos[0] = t;
    pos[1] = radius * sin(2 * M_PI / wave_len * pos[0]);
    pos[2] = height;
    return pos;
  }

  Vec3E Sin::vel(double t)
  {
    Vec3E vel = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    vel[0] = 1;
    vel[1] = radius * 2 * M_PI / wave_len * (2 * M_PI / wave_len * t);
    return vel;
  }

  Vec3E Sin::acc(double t)
  {
    Vec3E acc = Vec3E::Zero();
    return acc;
  }

  Vec3E Sin::jerk(double t)
  {
    Vec3E jerk = Vec3E::Zero();
    return jerk;
  }

}  // namespace basic_trajectories
