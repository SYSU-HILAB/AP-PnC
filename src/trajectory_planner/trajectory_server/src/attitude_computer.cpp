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

#include <trajectory_server/attitude_computer.hpp>

namespace trajectory_server
{

Eigen::Quaterniond compute_df_lyu_quat(const Eigen::Vector3d& vel,
                                       const Eigen::Vector3d& acc,
                                       double lyu_alpha)
{
  const Eigen::Vector3d gravity(0, 0, -9.81);
  constexpr double vThreshold = 0.5;

  // Singularity handling for low velocity
  if (vel.norm() < vThreshold)
  {
    // df_forward_singularity_vel logic
    Eigen::Vector3d eta = acc - gravity;
    Eigen::Vector3d xb  = eta.normalized();

    // Interpolate yb based on velocity scaling
    double scaling = vel.norm() / vThreshold;
    Eigen::Vector3d yb_from_v = vel.cross(eta).normalized();
    // For cirled trajectories, zb_fixed is North direction (0, 1, 0) in ENU
    Eigen::Vector3d zb_fixed(0, 1, 0);
    Eigen::Vector3d yb_from_fixed_frame = zb_fixed.cross(xb).normalized();

    Eigen::Vector3d yb =
        ((1.0 - scaling) * yb_from_fixed_frame + scaling * yb_from_v)
            .normalized();
    Eigen::Vector3d zb = xb.cross(yb).normalized();

    Eigen::Matrix3d rot;
    rot << xb, yb, zb;
    return Eigen::Quaterniond(rot);
  }
  else
  {
    // df_forward_nominal logic with lyu_alpha
    Eigen::Vector3d x_s = vel / vel.norm();

    // Compute z_s (direction of thrust, perpendicular to velocity)
    Eigen::Vector3d temp =
        -(Eigen::Matrix3d::Identity() - x_s * x_s.transpose()) * (acc - gravity);
    Eigen::Vector3d z_s = temp / temp.norm();

    // Compute y_s (completes the right-handed coordinate system)
    Eigen::Vector3d y_s = z_s.cross(x_s);

    // Use lyu_alpha instead of df_phi_alpha
    double alpha = lyu_alpha;

    // Compute body frame axes with alpha rotation
    Eigen::Vector3d xb = std::cos(alpha) * x_s - std::sin(alpha) * z_s;
    Eigen::Vector3d yb = y_s;
    Eigen::Vector3d zb = std::cos(alpha) * z_s + std::sin(alpha) * x_s;

    // Build rotation matrix and convert to quaternion
    Eigen::Matrix3d rot;
    rot << xb, yb, zb;
    return Eigen::Quaterniond(rot);
  }
}

}  // namespace trajectory_server
