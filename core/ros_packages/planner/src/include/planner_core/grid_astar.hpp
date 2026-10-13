/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#pragma once

#include <Eigen/Dense>
#include <gcopter/voxel_map.hpp>
#include <vector>

namespace planner::core
{

  /**
   * @brief 26-connected grid A* over a voxel_map::VoxelMap.
   *
   * Dependency-free stand-in for upstream gcopter's OMPL/RRT*-based
   * `sfc_gen::planPath` (see gcopter/sfc_gen.hpp): same call contract
   * (start/goal world positions in, a collision-free polyline route out,
   * collision-checked via the map's own `query()`), but deterministic grid
   * search instead of sampling-based planning. Appropriate for the small
   * (tens-of-metres), bounded scenes this project flies; would need
   * revisiting for large/maze-like environments.
   *
   * @param start Start position, world frame [m]. Must be unoccupied.
   * @param goal  Goal position, world frame [m]. Must be unoccupied.
   * @param map   Dilated occupancy map to search.
   * @param route Output waypoints from start to goal inclusive, world frame.
   *              Left untouched on failure.
   * @return true if a route was found, false if start/goal are occupied or
   *         out of bounds, or no path exists.
   */
  bool gridAstar(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                 const voxel_map::VoxelMap    &map,
                 std::vector<Eigen::Vector3d> &route);

}  // namespace planner::core
