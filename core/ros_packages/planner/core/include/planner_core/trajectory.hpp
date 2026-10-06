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

#pragma once

#include <Eigen/Dense>
#include <gcopter/trajectory.hpp>
#include <utility>

namespace planner::core
{

  /**
   * @brief One reference point of a planned trajectory.
   *
   * Pure data: no ROS message types. The ROS adapter maps this into the
   * flight message and the pybind binding exposes it as numpy arrays.
   */
  struct ReferencePoint
  {
    double          t = 0.0;  ///< time since trajectory start [s]
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  ///< position, world [m]
    Eigen::Vector3d v = Eigen::Vector3d::Zero();  ///< velocity, world [m/s]
    Eigen::Vector3d a =
        Eigen::Vector3d::Zero();  ///< acceleration, world [m/s^2]
    Eigen::Vector3d yb =
        Eigen::Vector3d(0.0, 1.0, 0.0);               ///< body y-axis, quad FLU
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();  ///< body rate [rad/s]
    double thrust         = 0.0;  ///< collective specific thrust [N/kg], +Z_FLU
    Eigen::Matrix3d rotation =
        Eigen::Matrix3d::Identity();  ///< R_world_body, FLU->ENU
    bool flatness_fallback = false;
  };

  /**
   * @brief Continuous reference trajectory: the MINCO trajectory + flatness.
   *
   * Holds the optimized MINCO trajectory and evaluates it at *arbitrary* time.
   * Consumers query exactly the times they need (e.g. the NMPC horizon grid
   * t0 + k * traj_res_s); there is no stored sample table, no interpolation and
   * no resampling.
   */
  class ReferenceTrajectory
  {
   public:
    ReferenceTrajectory() = default;
    explicit ReferenceTrajectory(TrajectoryRepresentation<5> traj);

    /// Total duration [s]; 0 for an empty trajectory.
    double duration() const { return duration_; }
    bool   empty() const { return duration_ <= 0.0; }

    /// Exact evaluation at t >= 0; requests beyond duration repeat the final
    /// point (all fields). ReferencePoint::t is the clamped sample time.
    ReferencePoint sample(double t) const;

    /// Read-only access for exact trajectory serialization/replay (coefficients
    /// remain planner-owned).
    const TrajectoryRepresentation<5> &representation() const { return traj_; }

   private:
    TrajectoryRepresentation<5> traj_;
    double                      duration_ = 0.0;
  };

}  // namespace planner::core
