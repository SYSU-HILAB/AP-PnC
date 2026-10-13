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

#include <algorithm>
#include <cmath>
#include <planner_core/grid_astar.hpp>
#include <queue>
#include <unordered_map>

namespace planner::core
{

  namespace
  {
    // Linear index of a voxel, matching the map's own row-major layout
    // (x fastest, then y, then z); used only as a hash-map key here.
    int64_t linear_index(const Eigen::Vector3i &size, const Eigen::Vector3i &id)
    {
      return static_cast<int64_t>(id(0)) +
             static_cast<int64_t>(id(1)) * size(0) +
             static_cast<int64_t>(id(2)) * size(0) * size(1);
    }

    struct Node
    {
      double  f;
      int64_t idx;
    };

    struct NodeGreater
    {
      bool operator()(const Node &a, const Node &b) const { return a.f > b.f; }
    };

    // Max expanded nodes before giving up; bounds worst-case memory/time on
    // the small (tens-of-metres) scenes this project flies.
    constexpr int kMaxExpansions = 2'000'000;
  }  // namespace

  bool gridAstar(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                 const voxel_map::VoxelMap    &map,
                 std::vector<Eigen::Vector3d> &route)
  {
    const Eigen::Vector3i size    = map.getSize();
    const Eigen::Vector3i startId = map.posD2I(start);
    const Eigen::Vector3i goalId  = map.posD2I(goal);

    if (map.query(startId) || map.query(goalId))
    {
      return false;
    }

    const int64_t startIdx = linear_index(size, startId);
    const int64_t goalIdx  = linear_index(size, goalId);
    if (startIdx == goalIdx)
    {
      route = {start, goal};
      return true;
    }

    const double scale = map.getScale();

    std::priority_queue<Node, std::vector<Node>, NodeGreater> open;
    std::unordered_map<int64_t, double>                       gScore;
    std::unordered_map<int64_t, int64_t>                      parent;
    std::unordered_map<int64_t, Eigen::Vector3i>              idOf;

    auto heuristic = [&](const Eigen::Vector3i &id)
    { return (id - goalId).cast<double>().norm() * scale; };

    gScore[startIdx] = 0.0;
    idOf[startIdx]   = startId;
    open.push({heuristic(startId), startIdx});

    std::vector<Eigen::Vector3i> neighbors;
    neighbors.reserve(26);
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0)
          {
            continue;
          }
          neighbors.emplace_back(dx, dy, dz);
        }
      }
    }

    int expansions = 0;
    while (!open.empty())
    {
      const Node current = open.top();
      open.pop();

      if (current.idx == goalIdx)
      {
        std::vector<Eigen::Vector3d> reversed;
        int64_t                      cursor = goalIdx;
        reversed.push_back(goal);
        while (cursor != startIdx)
        {
          cursor = parent.at(cursor);
          reversed.push_back(cursor == startIdx ? start
                                                : map.posI2D(idOf.at(cursor)));
        }
        route.assign(reversed.rbegin(), reversed.rend());
        return true;
      }

      if (++expansions > kMaxExpansions)
      {
        return false;
      }

      const Eigen::Vector3i currentId = idOf.at(current.idx);
      // A closed node may be re-pushed with a stale f before it is popped the
      // first time; skip work for any such stale duplicate.
      if (current.f > heuristic(currentId) + gScore.at(current.idx) + 1e-9)
      {
        continue;
      }

      for (const Eigen::Vector3i &offset : neighbors)
      {
        const Eigen::Vector3i neighborId = currentId + offset;
        if (neighborId(0) < 0 || neighborId(1) < 0 || neighborId(2) < 0 ||
            neighborId(0) >= size(0) || neighborId(1) >= size(1) ||
            neighborId(2) >= size(2))
        {
          continue;
        }
        if (map.query(neighborId))
        {
          continue;
        }

        const int64_t neighborIdx = linear_index(size, neighborId);
        const double  step        = offset.cast<double>().norm() * scale;
        const double  tentativeG  = gScore.at(current.idx) + step;

        const auto existing = gScore.find(neighborIdx);
        if (existing == gScore.end() || tentativeG < existing->second)
        {
          gScore[neighborIdx] = tentativeG;
          parent[neighborIdx] = current.idx;
          idOf[neighborIdx]   = neighborId;
          open.push({tentativeG + heuristic(neighborId), neighborIdx});
        }
      }
    }

    return false;
  }

}  // namespace planner::core
