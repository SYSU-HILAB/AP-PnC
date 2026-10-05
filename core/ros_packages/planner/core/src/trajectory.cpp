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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <planner_core/trajectory.hpp>
#include <tailsitter_df/flu_reference.hpp>

namespace planner::core
{

  ReferenceTrajectory::ReferenceTrajectory(TrajectoryRepresentation<5> traj)
      : traj_(std::move(traj)), duration_(traj_.getTotalDuration())
  {
  }

  ReferencePoint ReferenceTrajectory::sample(double t) const
  {
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
