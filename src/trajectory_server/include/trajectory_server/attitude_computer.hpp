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

#pragma once

#include <Eigen/Dense>

namespace trajectory_server
{

/**
 * @brief Compute df_lyu_quat from velocity, acceleration, and lyu_alpha
 *
 * Stateless helper function that computes the attitude quaternion based on
 * differential flatness with the Lyu aerodynamic model.
 *
 * Follows the same algorithm as df_forward_nominal but uses the provided
 * alpha value from the aerodynamic solver.
 *
 * Handles velocity singularity (v < vThreshold) similar to df_forward_singularity_vel.
 *
 * @param vel Velocity vector (m/s)
 * @param acc Acceleration vector (m/s²)
 * @param lyu_alpha Angle of attack from Lyu aerodynamic model (radians)
 * @return Quaternion representing body attitude
 */
Eigen::Quaterniond compute_df_lyu_quat(const Eigen::Vector3d& vel,
                                       const Eigen::Vector3d& acc,
                                       double lyu_alpha);

}  // namespace trajectory_server
