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
    // Lissajous pattern constants: coprime turns/lobes make the beam sweep
    // non-repetitively within one frame, like the Livox Mid-360.
    constexpr double kHorizontalTurns = 3.0;
    constexpr double kVerticalLobes   = 7.0;
    // Golden-ratio conjugate phase per frame: adjacent frames interleave.
    constexpr double kGoldenPhase = 0.6180339887498949;
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
    const double phase = static_cast<double>(frame_index) * kGoldenPhase;

    for (std::size_t i = 0; i < config_.points_per_frame; ++i)
    {
      const double u = (static_cast<double>(i) + 0.5) /
                       static_cast<double>(config_.points_per_frame);

      const double h_angle =
          fov_h * std::fmod(u * kHorizontalTurns + phase, 1.0);
      const double v =
          v_lo +
          (v_hi - v_lo) *
              (0.5 + 0.5 * std::sin(2.0 * M_PI * (u * kVerticalLobes + phase)));

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
