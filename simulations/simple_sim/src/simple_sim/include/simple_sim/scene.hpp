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

  // Vertical cylinder: circle (center_xy, radius) extruded from z=0 to height.
  struct Cylinder
  {
    Eigen::Vector2d center_xy;
    double          radius;
    double          height;
  };

  /**
   * Parameters of the pillar-course world generator: a closed box
   * (floor, ceiling, four walls) with two straight rows of identical
   * upright pillars, symmetric about the y=0 flight line.
   *
   * @param[in] pillar_count   Number of x positions per row (density) [-]
   * @param[in] pillar_radius  Uniform pillar radius [m]
   * @param[in] pillar_offset  Pillar-row distance from the y=0 line [m]
   * @param[in] edge_margin    Clearance of the first/last pillar from the
   *                           end walls [m]
   */
  struct PillarCourseConfig
  {
    double length        = 100.0;
    double width         = 20.0;
    double height        = 20.0;
    int    pillar_count  = 8;
    double pillar_radius = 0.8;
    double pillar_offset = 5.0;
    double edge_margin   = 5.0;
    bool   build_shell   = true;
  };

  /**
   * Static world composed of floor/ceiling planes, wall boxes and upright
   * cylinder obstacles (pillar-course layout).
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
    double                ground_z  = 0.0;
    double                ceiling_z = -1.0;  // <= 0 disables the ceiling
    std::vector<Sphere>   spheres;
    std::vector<Box>      boxes;
    std::vector<Cylinder> cylinders;

    bool raycast(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                 double max_range, double *distance) const;
  };

  /**
   * Build the pillar-course world: floor and ceiling planes, four boundary
   * walls and two straight rows of identical upright pillars (full height,
   * symmetric about y=0). Layout is fully deterministic - no randomness.
   *
   * @param[in] cfg  World size, density and layout parameters
   * @return Generated scene
   */
  Scene makePillarCourse(const PillarCourseConfig &cfg);

}  // namespace simple_sim

#endif  // SIMPLE_SIM_SCENE_HPP_
