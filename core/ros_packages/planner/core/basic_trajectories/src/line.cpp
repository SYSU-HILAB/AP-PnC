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

#include <basic_trajectories/line.hpp>

namespace basic_trajectories
{

  Line::Line(double radius_in, double angular_speed_in, double height_in)
  {
    radius        = radius_in;
    angular_speed = angular_speed_in;
    height        = height_in;
    type          = basic_trajectories::Type::INIT;
    linear_vel_   = 2 * radius * angular_speed / (2 * M_PI);
  }

  Vec3E Line::pos(double t)
  {
    Vec3E pos = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    pos[0] = linear_vel_ * t;
    pos[1] = 0;
    pos[2] = height;
    return pos;
  }

  Vec3E Line::vel(double t)
  {
    Vec3E vel = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    vel[0] = linear_vel_;
    vel[1] = 0;
    return vel;
  }

  Vec3E Line::acc(double t)
  {
    Vec3E acc = Vec3E::Zero();
    return acc;
  }

  Vec3E Line::jerk(double t)
  {
    Vec3E jerk = Vec3E::Zero();
    return jerk;
  }

}  // namespace basic_trajectories
