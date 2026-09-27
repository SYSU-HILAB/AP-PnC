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
#include <tailsitter_df/tailsitter_df.hpp>

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

    // Flatness: body axis + rates from (v, a, jerk).
    double          thrust = 0.0;
    Eigen::Vector3d omega  = Eigen::Vector3d::Zero();
    auto &          data   = tailsitter_df::Data::getInstance();
    data.df_backward_auto(rp.v, rp.a, traj_.getJer(t), thrust, omega);
    rp.thrust = thrust;
    rp.omega  = omega;

    // reference trajectory uses quad FLU body frame, so negate yb
    const Eigen::Vector3d yb = data.getYb();
    rp.yb = Eigen::Vector3d(-yb(0), -yb(1), -yb(2));
    return rp;
  }

}  // namespace planner::core
