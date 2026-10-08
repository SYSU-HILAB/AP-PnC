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

/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 */

#include <basic_trajectories/circle.hpp>

namespace basic_trajectories
{

  Circle::Circle(double radius_in, double angular_speed_in, double height_in)
  {
    radius        = radius_in;
    angular_speed = angular_speed_in;
    height        = height_in;
    type          = basic_trajectories::Type::INIT;
  }

  Vec3E Circle::pos(double t)
  {
    Vec3E pos = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    double phaseshift = M_PI / 2.0;
    pos[0]            = radius * cos(-angular_speed * t + phaseshift);
    pos[1]            = radius * sin(-angular_speed * t + phaseshift);
    pos[2]            = height;
    return pos;
  }

  Vec3E Circle::vel(double t)
  {
    Vec3E vel = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    double phaseshift = M_PI / 2.0;
    vel[0] = radius * angular_speed * sin(-angular_speed * t + phaseshift);
    vel[1] = -radius * angular_speed *
             (cos(-angular_speed * t + phaseshift) *
                  cos(-angular_speed * t + phaseshift) -
              sin(-angular_speed * t + phaseshift) *
                  sin(-angular_speed * t + phaseshift));
    return vel;
  }

  Vec3E Circle::acc(double t)
  {
    Vec3E acc = Vec3E::Zero();
    return acc;
  }

  Vec3E Circle::jerk(double t)
  {
    Vec3E jerk = Vec3E::Zero();
    return jerk;
  }

}  // namespace basic_trajectories
