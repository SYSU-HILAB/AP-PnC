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
 * author: WarriorHanamy ; email: rongerch@outlook.com
 */

#pragma once
#include <basic_trajectories/trajectory.hpp>
namespace basic_trajectories
{

  class VerticalLoop : public Trajectory
  {
   public:
    VerticalLoop(/* args */) = delete;
    explicit VerticalLoop(double a, double w, double height = 5);
    Vec3E pos(double t) override;
    Vec3E vel(double t) override;
    Vec3E acc(double t) override;
    Vec3E jerk(double t) override;
    // VecXE point(double t) override;
  };

}  // namespace basic_trajectories
