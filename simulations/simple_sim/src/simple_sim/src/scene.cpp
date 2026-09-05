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
#include <random>

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

    // Ray/vertical-cylinder intersection: circle solve in the xy-plane,
    // then keep roots whose hit z lies within [0, height].
    bool RayCylinder(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                     const Cylinder &cyl, double t_max, double *t_out)
    {
      const double ox = origin.x() - cyl.center_xy.x();
      const double oy = origin.y() - cyl.center_xy.y();
      const double a  = dir.x() * dir.x() + dir.y() * dir.y();
      if (a < 1e-12)
      {
        // Vertical ray: only the flat caps can be hit.
        if (ox * ox + oy * oy > cyl.radius * cyl.radius)
        {
          return false;
        }
        const double cap_z = dir.z() > 0.0 ? cyl.height : 0.0;
        if (std::abs(dir.z()) < 1e-12)
        {
          return false;
        }
        const double t = (cap_z - origin.z()) / dir.z();
        if (t < 1e-4 || t > t_max)
        {
          return false;
        }
        *t_out = t;
        return true;
      }

      const double b    = 2.0 * (ox * dir.x() + oy * dir.y());
      const double c    = ox * ox + oy * oy - cyl.radius * cyl.radius;
      const double disc = b * b - 4.0 * a * c;
      if (disc < 0.0)
      {
        return false;
      }
      const double sq = std::sqrt(disc);
      for (double t : {(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)})
      {
        if (t < 1e-4 || t > t_max)
        {
          continue;
        }
        const double z = origin.z() + t * dir.z();
        if (z >= 0.0 && z <= cyl.height)
        {
          *t_out = t;
          return true;
        }
      }
      return false;
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

    for (const auto &cyl : cylinders)
    {
      double t = 0.0;
      if (RayCylinder(origin, dir, cyl, best, &t) && t < best)
      {
        best = t;
        hit  = true;
      }
    }

    *distance = best;
    return hit;
  }

  Scene makePillarCourse(const PillarCourseConfig &cfg)
  {
    Scene scene;

    // Deterministic pillar field: pillars evenly spread along the corridor
    // with jittered x, random side and offset; pillar surfaces never intrude
    // into the |y| < corridor flight channel.
    std::mt19937                           rng(cfg.seed);
    std::uniform_real_distribution<double> jitter(-0.2, 0.2);
    std::uniform_real_distribution<double> radius(cfg.radius_min, cfg.radius_max);
    std::uniform_real_distribution<double> side(0.0, 1.0);

    const int    N      = std::max(cfg.pillar_count, 0);
    const double x_lo   = -cfg.length / 2.0 + cfg.edge_margin;
    const double x_hi   = cfg.length / 2.0 - cfg.edge_margin;
    const double y_lim  = cfg.width / 2.0 - cfg.edge_margin;
    const double span   = (x_hi - x_lo) / std::max(N, 1);
    for (int i = 0; i < N; ++i)
    {
      const double u = (static_cast<double>(i) + 0.5) / N;
      const double x =
          std::clamp(x_lo + (x_hi - x_lo) * u + jitter(rng) * span, x_lo, x_hi);
      const double r = radius(rng);
      // Offset from the corridor edge by the pillar radius + 0.5 m.
      const double y_min = cfg.corridor_half_width + r + 0.5;
      std::uniform_real_distribution<double> y_dist(y_min, std::max(y_lim, y_min));
      const double y = (side(rng) < 0.5 ? -1.0 : 1.0) * y_dist(rng);

      Cylinder cyl;
      cyl.center_xy << x, y;
      cyl.radius = r;
      cyl.height = cfg.height;
      scene.cylinders.push_back(cyl);
    }

    if (cfg.build_walls)
    {
      const double t = 0.2;  // wall thickness [m]
      const auto   wall = [&](double cx, double cy, double hx, double hy)
      {
        Box b;
        b.center << cx, cy, cfg.height / 2.0;
        b.half_extent << hx, hy, cfg.height / 2.0;
        scene.boxes.push_back(b);
      };
      wall(0.0, cfg.width / 2.0 + t, cfg.length / 2.0 + t, t);
      wall(0.0, -cfg.width / 2.0 - t, cfg.length / 2.0 + t, t);
      wall(cfg.length / 2.0 + t, 0.0, t, cfg.width / 2.0 + t);
      wall(-cfg.length / 2.0 - t, 0.0, t, cfg.width / 2.0 + t);
    }
    return scene;
  }

}  // namespace simple_sim
