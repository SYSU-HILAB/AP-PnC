/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include "simple_sim/lidar_sim.hpp"

#include <cmath>

namespace simple_sim
{

  namespace
  {
    // Kronecker-sequence constants (irrational fractional parts): the i-th
    // sample of frame k sits at {(i+1/2)*alpha + k*beta} per axis, which
    // spreads points evenly over the FOV without scan-curve artifacts.
    constexpr double kAlphaH = 0.6180339887498949;  // golden ratio
    constexpr double kAlphaV = 0.7548776662466927;  // sqrt(2) - 1
    constexpr double kBetaH  = 0.5454545454545455;  // frame offset, h axis
    constexpr double kBetaV  = 0.3789290811307804;  // frame offset, v axis

    double frac(double x)
    {
      return x - std::floor(x);
    }
  }  // namespace

  LidarSim::LidarSim(const LidarConfig &config) : config_(config) {}

  void LidarSim::scan(const Scene &scene, const Eigen::Vector3d &position,
                      const Eigen::Quaterniond &attitude,
                      std::size_t               frame_index,
                      std::vector<LidarPoint>  *points) const
  {
    points->clear();
    points->reserve(config_.points_per_frame);

    const double fov_h = config_.fov_h_deg * M_PI / 180.0;
    const double v_lo  = config_.fov_v_min_deg * M_PI / 180.0;
    const double v_hi  = config_.fov_v_max_deg * M_PI / 180.0;

    for (std::size_t i = 0; i < config_.points_per_frame; ++i)
    {
      const double n = static_cast<double>(i) + 0.5;
      const double k = static_cast<double>(frame_index);

      const double h_frac  = frac(n * kAlphaH + k * kBetaH);
      const double v_frac  = frac(n * kAlphaV + k * kBetaV);
      const double h_angle = fov_h * h_frac;
      const double v       = v_lo + (v_hi - v_lo) * v_frac;

      const Eigen::Vector3d dir_sensor(std::cos(v) * std::cos(h_angle),
                                       std::cos(v) * std::sin(h_angle),
                                       std::sin(v));
      const Eigen::Vector3d dir_world = attitude * dir_sensor;

      double t = 0.0;
      if (!scene.raycast(position, dir_world, config_.max_range, &t))
      {
        continue;
      }
      if (t < config_.min_range)
      {
        continue;
      }
      LidarPoint pt;
      pt.point     = position + dir_world * t;
      pt.intensity = 1.0 - t / config_.max_range;
      points->push_back(pt);
    }
  }

}  // namespace simple_sim
