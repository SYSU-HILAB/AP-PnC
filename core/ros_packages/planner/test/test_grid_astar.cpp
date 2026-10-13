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

#include <gtest/gtest.h>

#include <gcopter/sfc_gen.hpp>
#include <gcopter/voxel_map.hpp>
#include <planner_core/grid_astar.hpp>

namespace planner::core
{
  namespace
  {
    // A wall across x in [-3, 3] at y = 0 with a gap near x = 0.
    voxel_map::VoxelMap make_wall_scene()
    {
      voxel_map::VoxelMap map(Eigen::Vector3i(40, 40, 10),
                              Eigen::Vector3d(-10, -10, -1), 0.5);
      for (double x = -3.0; x <= 3.0; x += 0.25)
      {
        if (x > -0.5 && x < 0.5)
        {
          continue;
        }
        for (double z = -1.0; z <= 3.0; z += 0.25)
        {
          map.setOccupied(Eigen::Vector3d(x, 0.0, z));
        }
      }
      map.dilate(1);
      return map;
    }

    voxel_map::VoxelMap make_fully_blocked_scene()
    {
      voxel_map::VoxelMap map(Eigen::Vector3i(40, 40, 10),
                              Eigen::Vector3d(-10, -10, -1), 0.5);
      for (double x = -3.0; x <= 3.0; x += 0.25)
      {
        for (double y = -10.0; y <= 10.0; y += 0.25)
        {
          for (double z = -1.0; z <= 3.0; z += 0.25)
          {
            map.setOccupied(Eigen::Vector3d(x, y, z));
          }
        }
      }
      map.dilate(1);
      return map;
    }
  }  // namespace

  TEST(GridAstar, RoutesThroughTheGapWithoutCollision)
  {
    const voxel_map::VoxelMap map = make_wall_scene();
    const Eigen::Vector3d     start(-8.0, 0.0, 0.0), goal(8.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> route;
    ASSERT_TRUE(gridAstar(start, goal, map, route));
    ASSERT_GE(route.size(), 2u);
    EXPECT_TRUE(route.front().isApprox(start));
    EXPECT_TRUE(route.back().isApprox(goal));

    for (const Eigen::Vector3d &p : route)
    {
      EXPECT_FALSE(map.query(p)) << "route point " << p.transpose()
                                 << " lies in occupied/dilated space";
    }

    double length = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
      length += (route[i] - route[i - 1]).norm();
    }
    // The detour around the wall must be longer than flying straight through
    // it, but not absurdly so for this small gap.
    EXPECT_GT(length, (goal - start).norm());
    EXPECT_LT(length, (goal - start).norm() * 2.0);
  }

  TEST(GridAstar, FailsClosedWhenNoRouteExists)
  {
    const voxel_map::VoxelMap map = make_fully_blocked_scene();
    const Eigen::Vector3d     start(-8.0, 0.0, 0.0), goal(8.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> route;
    EXPECT_FALSE(gridAstar(start, goal, map, route));
  }

  TEST(GridAstar, RejectsOccupiedEndpoints)
  {
    const voxel_map::VoxelMap    map = make_wall_scene();
    std::vector<Eigen::Vector3d> route;
    // (0.2, 0, 0) sits inside the dilated wall segment.
    EXPECT_FALSE(gridAstar(Eigen::Vector3d(0.2, 0.0, 0.0),
                           Eigen::Vector3d(8.0, 0.0, 0.0), map, route));
  }

  TEST(GridAstar, FeedsAConnectedCorridorChain)
  {
    const voxel_map::VoxelMap map = make_wall_scene();
    const Eigen::Vector3d     start(-8.0, 0.0, 0.0), goal(8.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> route;
    ASSERT_TRUE(gridAstar(start, goal, map, route));

    std::vector<Eigen::Vector3d> surf;
    map.getSurf(surf);
    ASSERT_FALSE(surf.empty());

    std::vector<Eigen::MatrixX4d> hPolys;
    sfc_gen::convexCover(route, surf, map.getOrigin(), map.getCorner(), 3.0,
                         2.0, hPolys);
    sfc_gen::shortCut(hPolys);

    ASSERT_FALSE(hPolys.empty());
    for (std::size_t i = 0; i + 1 < hPolys.size(); ++i)
    {
      EXPECT_TRUE(geo_utils::overlap(hPolys[i], hPolys[i + 1], 0.01))
          << "corridor gap between polytopes " << i << " and " << i + 1;
    }
  }

}  // namespace planner::core
