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

  LidarSim::LidarSim(const LidarConfig &config) : config_(config) {}

  void LidarSim::scan(const Scene &scene, const Eigen::Vector3d &position,
                      const Eigen::Quaterniond &attitude,
                      std::vector<LidarPoint>  *points) const
  {
    points->clear();
    points->reserve(config_.rings * config_.points_per_ring);

    const double      fov_h      = config_.fov_h_deg * M_PI / 180.0;
    const double      fov_v      = config_.fov_v_deg * M_PI / 180.0;
    const std::size_t ring_count = std::max<std::size_t>(config_.rings, 1);

    for (std::size_t ring = 0; ring < ring_count; ++ring)
    {
      const double v = ring_count == 1
                           ? 0.0
                           : -fov_v / 2.0 + fov_v * ring / (ring_count - 1);
      for (std::size_t h = 0; h < config_.points_per_ring; ++h)
      {
        const double          h_angle = fov_h * h / config_.points_per_ring;
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
  }

}  // namespace simple_sim
