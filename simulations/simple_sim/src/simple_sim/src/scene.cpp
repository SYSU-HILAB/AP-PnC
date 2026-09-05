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

#include "simple_sim/scene.hpp"

#include <algorithm>
#include <cmath>

namespace simple_sim
{

  namespace
  {

    // Ray/sphere intersection; returns the first positive hit distance.
    bool RaySphere(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                   const Sphere &sphere, double *t_out)
    {
      const Eigen::Vector3d oc = origin - sphere.center;
      const double          b  = oc.dot(dir);
      const double c    = oc.squaredNorm() - sphere.radius * sphere.radius;
      const double disc = b * b - c;
      if (disc < 0.0)
      {
        return false;
      }
      const double sq = std::sqrt(disc);
      double       t  = -b - sq;
      if (t < 1e-4)
      {
        t = -b + sq;
      }
      if (t < 1e-4)
      {
        return false;
      }
      *t_out = t;
      return true;
    }

    // Ray/AABB (axis-aligned) intersection via the slab method.
    bool RayBox(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                const Box &box, double t_max, double *t_out)
    {
      double t_min     = 0.0;
      double t_max_hit = t_max;
      for (int axis = 0; axis < 3; ++axis)
      {
        const double lo = box.center[axis] - box.half_extent[axis];
        const double hi = box.center[axis] + box.half_extent[axis];
        if (std::abs(dir[axis]) < 1e-9)
        {
          if (origin[axis] < lo || origin[axis] > hi)
          {
            return false;
          }
        }
        else
        {
          const double inv = 1.0 / dir[axis];
          double       t1  = (lo - origin[axis]) * inv;
          double       t2  = (hi - origin[axis]) * inv;
          if (t1 > t2)
          {
            std::swap(t1, t2);
          }
          t_min     = std::max(t_min, t1);
          t_max_hit = std::min(t_max_hit, t2);
          if (t_min > t_max_hit)
          {
            return false;
          }
        }
      }
      if (t_min < 1e-4)
      {
        return false;
      }
      *t_out = t_min;
      return true;
    }

  }  // namespace

  bool Scene::raycast(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                      double max_range, double *distance) const
  {
    double best = max_range;
    bool   hit  = false;

    if (dir.z() < -1e-9)
    {
      const double t = (ground_z - origin.z()) / dir.z();
      if (t > 1e-4 && t < best)
      {
        best = t;
        hit  = true;
      }
    }

    for (const auto &sphere : spheres)
    {
      double t = 0.0;
      if (RaySphere(origin, dir, sphere, &t) && t < best)
      {
        best = t;
        hit  = true;
      }
    }

    for (const auto &box : boxes)
    {
      double t = 0.0;
      if (RayBox(origin, dir, box, best, &t) && t < best)
      {
        best = t;
        hit  = true;
      }
    }

    *distance = best;
    return hit;
  }

}  // namespace simple_sim
