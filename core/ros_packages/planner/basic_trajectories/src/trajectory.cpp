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

#include <basic_trajectories/trajectory.hpp>

namespace basic_trajectories
{

  Trajectory::Trajectory()
  {
    type = Type::UNDEFINED;
  }
  // Trajectory

  Mat3x4 Trajectory::point(double t)
  {
    static Mat3x4 point;
    point.col(0) = pos(t);
    point.col(1) = vel(t);
    point.col(2) = acc(t);
    return point;
  }

  std::vector<Mat3x4> Trajectory::samplingPointArray(int num)
  {
    std::vector<Mat3x4> res;
    double              totaltime      = 2 * M_PI / angular_speed;
    double              resolutionTime = totaltime / num;
    for (double t = 0; t <= totaltime + 1e-6; t += resolutionTime)
    {
      res.push_back(point(t));
    }
    return res;
  }
}  // namespace basic_trajectories
