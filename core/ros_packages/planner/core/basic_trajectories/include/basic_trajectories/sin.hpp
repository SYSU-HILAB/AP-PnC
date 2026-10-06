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

#pragma once
#include <basic_trajectories/trajectory.hpp>
namespace basic_trajectories
{
  constexpr double kLenSigmoid   = 8;
  constexpr double kPieceOfMinco = 10;
  class Sin : public Trajectory
  {
   public:
    Sin(/* args */) = delete;
    // a is intuitively thought as the width of the horizontal "S" curve
    // w is intuitively thought as the yaw rate of the "S" curve
    explicit Sin(double a, double wave_len = 9.0, double height = 5);
    Vec3E pos(double t) override;
    Vec3E vel(double t) override;
    Vec3E acc(double t) override;
    Vec3E jerk(double t) override;
  };

}  // namespace basic_trajectories
