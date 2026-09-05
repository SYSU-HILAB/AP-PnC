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

#ifndef SIMPLE_SIM_SCENE_HPP_
#define SIMPLE_SIM_SCENE_HPP_

#include <Eigen/Dense>
#include <vector>

namespace simple_sim
{

  struct Sphere
  {
    Eigen::Vector3d center;
    double          radius;
  };

  struct Box
  {
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
  };

  /**
   * Static world composed of a ground plane plus box/sphere obstacles.
   *
   * @param[in]  origin     Ray origin in world frame [m]
   * @param[in]  dir        Unit ray direction in world frame [-]
   * @param[in]  max_range  Maximum sensing distance [m]
   * @param[out] distance   Distance to nearest hit along the ray [m]
   * @return true if the ray hits geometry within max_range
   */
  class Scene
  {
   public:
    double              ground_z = 0.0;
    std::vector<Sphere> spheres;
    std::vector<Box>    boxes;

    bool raycast(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                 double max_range, double *distance) const;
  };

}  // namespace simple_sim

#endif  // SIMPLE_SIM_SCENE_HPP_
