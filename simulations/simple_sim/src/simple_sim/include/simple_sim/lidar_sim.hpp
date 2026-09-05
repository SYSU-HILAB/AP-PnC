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

#ifndef SIMPLE_SIM_LIDAR_SIM_HPP_
#define SIMPLE_SIM_LIDAR_SIM_HPP_

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

#include "simple_sim/scene.hpp"

namespace simple_sim
{

  /**
   * Livox Mid-360 style sensor: 360 deg horizontal, -7..+52 deg vertical,
   * emulated with a Lissajous non-repetitive scan pattern (MARSIM-style).
   */
  struct LidarConfig
  {
    std::size_t points_per_frame = 4000;
    double      fov_h_deg        = 360.0;
    double      fov_v_min_deg    = -7.0;
    double      fov_v_max_deg    = 52.0;
    double      max_range        = 40.0;
    double      min_range        = 0.1;
  };

  struct LidarPoint
  {
    Eigen::Vector3d point;
    double          intensity;
  };

  /**
   * Synthetic Mid-360 LiDAR using ray casting against the scene. Consecutive
   * frames use golden-ratio phase offsets so their samples interleave instead
   * of repeating.
   *
   * @param[in]  config    Sensor geometry (points per frame, FOV, range)
   */
  class LidarSim
  {
   public:
    explicit LidarSim(const LidarConfig &config);

    /**
     * Emit one scan frame from the sensor pose.
     *
     * @param[in]  scene        World geometry
     * @param[in]  position     Sensor position in world frame [m]
     * @param[in]  attitude     Sensor attitude in world frame [-]
     * @param[in]  frame_index  Monotonic frame counter for pattern phase [-]
     * @param[out] points       Hit points in world frame [m]
     */
    void scan(const Scene &scene, const Eigen::Vector3d &position,
              const Eigen::Quaterniond &attitude, std::size_t frame_index,
              std::vector<LidarPoint> *points) const;

   private:
    LidarConfig config_;
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_LIDAR_SIM_HPP_
