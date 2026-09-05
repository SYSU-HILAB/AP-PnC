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
   * (floor, ceiling, four walls) with pillar_count upright pillars
   * scattered completely at random (no rows, no mirroring).
   *
   * @param[in] pillar_count         Total number of pillars (density)
   * @param[in] radius_min/max       Pillar radius range [m]
   * @param[in] corridor_half_width  Keep-out: pillar surfaces never enter
   *                                 |y| < this value [m]
   * @param[in] edge_margin          Clearance from the end walls [m]
   * @param[in] seed                 RNG seed; same seed, same layout
   */
  struct PillarCourseConfig
  {
    double   length              = 100.0;
    double   width               = 20.0;
    double   height              = 20.0;
    int      pillar_count        = 16;
    double   radius_min          = 0.6;
    double   radius_max          = 1.2;
    double   corridor_half_width = 3.0;
    double   edge_margin         = 5.0;
    unsigned seed                = 42;
    bool     build_shell         = true;
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
   * walls and pillar_count full-height upright pillars placed by seeded
   * rejection sampling - independent x, side, offset and radius, with a
   * minimum center spacing so pillars never overlap. Pillar surfaces stay
   * outside the |y| < corridor_half_width flight channel.
   *
   * @param[in] cfg  World size, density and layout parameters
   * @return Generated scene
   */
  Scene makePillarCourse(const PillarCourseConfig &cfg);

}  // namespace simple_sim

#endif  // SIMPLE_SIM_SCENE_HPP_
