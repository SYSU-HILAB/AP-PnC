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

#include <basic_trajectories/lemniscate.hpp>

namespace basic_trajectories
{

  Lemniscate::Lemniscate(double radius_in, double angular_speed_in,
                         double height_in, double curvature_factor_in)
  {
    radius           = radius_in;
    angular_speed    = angular_speed_in;
    height           = height_in;
    type             = basic_trajectories::Type::INIT;
    curvature_factor = curvature_factor_in;
  }

  Vec3E Lemniscate::pos(double t)
  {
    Vec3E  pos        = Vec3E::Zero();
    double phaseshift = M_PI;
    // Only change first two, i.e., x and y;
    pos[0] = radius * cos(angular_speed * t + phaseshift) /
             (1 + curvature_factor * sin(angular_speed * t + phaseshift) *
                      sin(angular_speed * t + phaseshift));
    pos[1] = radius * sin(angular_speed * t + phaseshift) *
             cos(angular_speed * t + phaseshift) /
             (1 + curvature_factor * sin(angular_speed * t + phaseshift) *
                      sin(angular_speed * t + phaseshift));
    pos[2] = height;
    return pos;
  }

  Vec3E Lemniscate::vel(double t)
  {
    Vec3E  vel        = Vec3E::Zero();
    double phaseshift = M_PI;
    // Only change first two, i.e., x and y;
    vel[0] = -radius * angular_speed * sin(angular_speed * t + phaseshift);
    vel[1] = radius * angular_speed *
             (cos(angular_speed * t + M_PI) * cos(angular_speed * t + M_PI) -
              sin(angular_speed * t + M_PI) * sin(angular_speed * t + M_PI));
    return vel;
  }

  Vec3E Lemniscate::acc(double t)
  {
    Vec3E acc = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    return acc;
  }

  Vec3E Lemniscate::jerk(double t)
  {
    Vec3E jerk = Vec3E::Zero();
    // Only change first two, i.e., x and y;
    return jerk;
  }

}  // namespace basic_trajectories
