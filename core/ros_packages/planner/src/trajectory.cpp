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

#include <algorithm>
#include <cmath>
#include <planner_core/trajectory.hpp>
#include <stdexcept>
#include <tailsitter_df/flu_reference.hpp>

namespace planner::core
{

  ReferenceTrajectory::ReferenceTrajectory(TrajectoryRepresentation<5> traj)
      : traj_(std::move(traj)), duration_(traj_.getTotalDuration())
  {
  }

  ReferencePoint ReferenceTrajectory::sample(double t) const
  {
    if (empty() || !std::isfinite(t) || t < 0.0)
      throw std::invalid_argument("invalid trajectory sample time");
    // Repeat the complete final point, as the ROS reference buffer does.
    // Never extrapolate the terminal polynomial into the hover horizon.
    t = std::min(t, duration_);
    ReferencePoint rp;
    rp.t = t;
    rp.p = traj_.getPos(t);
    rp.v = traj_.getVel(t);
    rp.a = traj_.getAcc(t);

    const auto flatness =
        tailsitter_df::flu_reference(rp.v, rp.a, traj_.getJer(t));
    rp.rotation = flatness.rotation;
    rp.yb = rp.rotation.col(1);  // world representation of +Y_FLU, no sign fix
    rp.omega             = flatness.omega;
    rp.thrust            = flatness.specific_thrust;
    rp.flatness_fallback = flatness.fallback;
    return rp;
  }

}  // namespace planner::core
