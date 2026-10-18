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

/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include <Eigen/Eigen>
#include <deque>
#include <memory>

#include "firi.hpp"
#include "geo_utils.hpp"

namespace sfc_gen
{

  // NOTE(AP-PnC): upstream's OMPL/RRT*-based planPath() was intentionally
  // dropped here; this project uses its own dependency-free grid A*
  // (planner_core/grid_astar.hpp) instead, since OMPL is not otherwise a build
  // dependency and these are small, bounded scenes.

  inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                          const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d              &lowCorner,
                          const Eigen::Vector3d              &highCorner,
                          const double &progress, const double &range,
                          std::vector<Eigen::MatrixX4d> &hpolys,
                          const double                   eps = 1.0e-6)
  {
    hpolys.clear();
    const int                   n  = path.size();
    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
    bd(0, 0)                       = 1.0;
    bd(1, 0)                       = -1.0;
    bd(2, 1)                       = 1.0;
    bd(3, 1)                       = -1.0;
    bd(4, 2)                       = 1.0;
    bd(5, 2)                       = -1.0;

    Eigen::MatrixX4d             hp, gap;
    Eigen::Vector3d              a, b = path[0];
    std::vector<Eigen::Vector3d> valid_pc;
    std::vector<Eigen::Vector3d> bs;
    valid_pc.reserve(points.size());
    for (int i = 1; i < n;)
    {
      a = b;
      if ((a - path[i]).norm() > progress)
      {
        b = (path[i] - a).normalized() * progress + a;
      }
      else
      {
        b = path[i];
        i++;
      }
      bs.emplace_back(b);

      bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
      bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
      bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
      bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
      bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
      bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

      valid_pc.clear();
      for (const Eigen::Vector3d &p : points)
      {
        if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
        {
          valid_pc.emplace_back(p);
        }
      }
      if (valid_pc.empty())
      {
        // The global map can be occupied while this local road segment is
        // empty. Its bounded box already is a collision-free corridor; do
        // not dereference valid_pc[0] or ask FIRI to fit absent obstacles.
        hpolys.emplace_back(bd);
        continue;
      }
      Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
          valid_pc[0].data(), 3, valid_pc.size());

      firi::firi(bd, pc, a, b, hp);

      if (hpolys.size() != 0)
      {
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                     ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
        {
          firi::firi(bd, pc, a, a, gap, 1);
          hpolys.emplace_back(gap);
        }
      }

      hpolys.emplace_back(hp);
    }
  }

  inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
  {
    std::vector<Eigen::MatrixX4d> htemp = hpolys;
    if (htemp.size() == 1)
    {
      Eigen::MatrixX4d headPoly = htemp.front();
      htemp.insert(htemp.begin(), headPoly);
    }
    hpolys.clear();

    int              M = htemp.size();
    Eigen::MatrixX4d hPoly;
    bool             overlap;
    std::deque<int>  idices;
    idices.push_front(M - 1);
    for (int i = M - 1; i >= 0; i--)
    {
      for (int j = 0; j < i; j++)
      {
        if (j < i - 1)
        {
          overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
        }
        else
        {
          overlap = true;
        }
        if (overlap)
        {
          idices.push_front(j);
          i = j + 1;
          break;
        }
      }
    }
    for (const auto &ele : idices)
    {
      hpolys.push_back(htemp[ele]);
    }
  }

}  // namespace sfc_gen

#endif
