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

  struct LidarConfig
  {
    std::size_t rings           = 16;
    std::size_t points_per_ring = 360;
    double      fov_h_deg       = 360.0;
    double      fov_v_deg       = 30.0;
    double      max_range       = 30.0;
    double      min_range       = 0.2;
  };

  struct LidarPoint
  {
    Eigen::Vector3d point;
    double          intensity;
  };

  /**
   * Synthetic spherical-scan LiDAR using ray casting against the scene.
   *
   * @param[in]  config    Sensor geometry (rings, FOV, range)
   */
  class LidarSim
  {
   public:
    explicit LidarSim(const LidarConfig &config);

    /**
     * Emit one full scan from the sensor pose.
     *
     * @param[in]  scene     World geometry
     * @param[in]  position  Sensor position in world frame [m]
     * @param[in]  attitude  Sensor attitude in world frame [-]
     * @param[out] points    Hit points in world frame [m]
     */
    void scan(const Scene &scene, const Eigen::Vector3d &position,
              const Eigen::Quaterniond &attitude,
              std::vector<LidarPoint>  *points) const;

   private:
    LidarConfig config_;
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_LIDAR_SIM_HPP_
