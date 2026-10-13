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

#include <chrono>
#include <cmath>
#include <gcopter/sfc_gen.hpp>
#include <iostream>
#include <planner_core/grid_astar.hpp>
#include <planner_core/planner.hpp>

namespace planner::core
{

  namespace
  {
    const char *level_name(LogLevel level)
    {
      switch (level)
      {
        case LogLevel::Debug:
          return "debug";
        case LogLevel::Warn:
          return "warn";
        case LogLevel::Error:
          return "error";
        case LogLevel::Info:
        default:
          return "info";
      }
    }
  }  // namespace

  Planner::Planner(ProblemConfig config, LogSink log)
      : config_(std::move(config)), log_(std::move(log))
  {
    init_basic_trajectory_type(config_.problem.basic_traj.type);
  }

  void Planner::log(LogLevel level, const std::string &msg) const
  {
    if (log_)
    {
      log_(level, msg);
    }
    else
    {
      std::cerr << "[planner:" << level_name(level) << "] " << msg << std::endl;
    }
  }

  void Planner::init_basic_trajectory_type(int type)
  {
    if (type == 1)
    {
      basic_traj_ = std::make_unique<basic_trajectories::Lemniscate>(
          config_.problem.basic_traj.radius, config_.problem.basic_traj.rate,
          config_.problem.basic_traj.height,
          config_.problem.basic_traj.curvature_factor);
      return;
    }
    if (type == 2)
    {
      basic_traj_ = std::make_unique<basic_trajectories::Circle>(
          config_.problem.basic_traj.radius, config_.problem.basic_traj.rate,
          config_.problem.basic_traj.height);
      return;
    }
    if (type == 3)
    {
      basic_traj_ = std::make_unique<basic_trajectories::Sin>(
          config_.problem.basic_traj.radius,
          config_.problem.basic_traj.wave_len,
          config_.problem.basic_traj.height);
      config_.problem.basic_traj.rate = basic_traj_->getAngularSpeed();
      return;
    }
    if (type == 4)
    {
      basic_traj_ = std::make_unique<basic_trajectories::Line>(
          config_.problem.basic_traj.radius, config_.problem.basic_traj.rate,
          config_.problem.basic_traj.height);
      return;
    }
    else
    {
      log(LogLevel::Error, "The trajecotry type is not supported!");
    }
  }

  void Planner::create_basic_trajectory_sfc(
      std::vector<Eigen::MatrixX4d> &hPolys)
  {
    Eigen::RowVector4d h1, h2, h3, h4, h5, h6;

    Eigen::MatrixX4d H;
    H.resize(6, 4);

    auto route = basic_traj_->samplingPointArray(config_.problem.piece_n);

    for (size_t i = 1; i < route.size() - 1; i++)
    {
      auto            point = route[i];
      Eigen::Vector3d pc    = point.col(0);  // Center of the box

      // Define the half-lengths of the box in each direction
      double front = config_.problem.box.front;
      double back  = config_.problem.box.back;
      double right = config_.problem.box.right;
      double left  = config_.problem.box.left;
      double up    = config_.problem.box.up;
      double down  = config_.problem.box.down;

      // Create the six planes of the box (axis-aligned)
      // Each plane is defined as [a, b, c, d] where ax + by + cz + d <= 0
      h1 << 1, 0, 0, -pc.x() - front;  // Front (x+)
      h2 << -1, 0, 0, pc.x() - back;   // Back  (x-)
      h3 << 0, 1, 0, -pc.y() - right;  // Right (y+)
      h4 << 0, -1, 0, pc.y() - left;   // Left  (y-)
      h5 << 0, 0, 1, -pc.z() - up;     // Up    (z+)
      h6 << 0, 0, -1, pc.z() - down;   // Down  (z-)

      // Stack the planes into a matrix
      H << h1, h2, h3, h4, h5, h6;
      hPolys.push_back(H);
    }
  }

  std::optional<ReferenceTrajectory> Planner::plan()
  {
    std::vector<Eigen::MatrixX4d> hPolys;
    log(LogLevel::Info, "In Planning");
    create_basic_trajectory_sfc(hPolys);

    Eigen::Matrix3d iniState, finState;
    auto route = basic_traj_->samplingPointArray(config_.problem.piece_n);
    iniState << route.front().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    finState << route.back().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    gcopter::GCOPTER_PolytopeSFC gcopter;

    TrajectoryRepresentation<5> traj;
    Eigen::VectorXd             initTime;
    initTime.resize(config_.problem.piece_n);
    initTime.setConstant(2 * M_PI / config_.problem.basic_traj.rate /
                         config_.problem.piece_n);

    Eigen::Matrix3Xd initPointsPos(3, config_.problem.piece_n - 1);
    for (int i = 0; i < config_.problem.piece_n - 1; i++)
    {
      Eigen::Vector3d v = route[i].col(1);
      v.normalize();
      Eigen::Vector3d u = v.cross(Eigen::Vector3d::UnitZ());
      initPointsPos.col(i) =
          route[i + 1].col(0) + u * config_.problem.box.right * 0.8;
    }

    if (!gcopter.setup_basic_trajectory(iniState, finState, initTime,
                                        initPointsPos, hPolys,
                                        config_.problem.minco.smoothing_eps,
                                        config_.problem.minco.integral_intervs))
    {
      log(LogLevel::Error, "GCOPTER setup failed");
      return std::nullopt;
    }
    auto opt_start = std::chrono::high_resolution_clock::now();

    if (std::isinf(gcopter.optimize(traj, config_.problem.minco.rel_cost_tol)))
    {
      log(LogLevel::Error, "GCOPTER optimization diverged");
      return std::nullopt;
    }
    auto opt_end = std::chrono::high_resolution_clock::now();

    log(LogLevel::Info,
        "PLANNING Duration " +
            std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(opt_end -
                                                                      opt_start)
                    .count()) +
            " ms");

    return ReferenceTrajectory(std::move(traj));
  }

  bool Planner::build_corridor_sfc(const Eigen::Vector3d         &start,
                                   const Eigen::Vector3d         &goal,
                                   const voxel_map::VoxelMap     &map,
                                   std::vector<Eigen::MatrixX4d> &hPolys,
                                   std::vector<Eigen::Vector3d>  &route)
  {
    if (!gridAstar(start, goal, map, route))
    {
      log(LogLevel::Error, "Corridor planning: no collision-free route found");
      return false;
    }

    std::vector<Eigen::Vector3d> surf;
    map.getSurf(surf);
    if (surf.empty())
    {
      // Nothing occupied anywhere: skip FIRI (which indexes surf[0]
      // unconditionally) and use one corridor spanning the whole map.
      Eigen::MatrixX4d box(6, 4);
      box.setZero();
      box(0, 0) = 1.0;
      box(0, 3) = -map.getCorner().x();
      box(1, 0) = -1.0;
      box(1, 3) = map.getOrigin().x();
      box(2, 1) = 1.0;
      box(2, 3) = -map.getCorner().y();
      box(3, 1) = -1.0;
      box(3, 3) = map.getOrigin().y();
      box(4, 2) = 1.0;
      box(4, 3) = -map.getCorner().z();
      box(5, 2) = -1.0;
      box(5, 3) = map.getOrigin().z();
      hPolys    = {box};
      return true;
    }

    sfc_gen::convexCover(route, surf, map.getOrigin(), map.getCorner(),
                         /*progress=*/6.0, /*range=*/3.0, hPolys);
    sfc_gen::shortCut(hPolys);
    if (hPolys.empty())
    {
      log(LogLevel::Error, "Corridor planning: FIRI produced no polytopes");
      return false;
    }
    return true;
  }

  std::optional<ReferenceTrajectory> Planner::plan_corridor(
      const Eigen::Matrix3d &iniState, const Eigen::Matrix3d &finState,
      const voxel_map::VoxelMap &map)
  {
    std::vector<Eigen::MatrixX4d> hPolys;
    std::vector<Eigen::Vector3d>  route;
    if (!build_corridor_sfc(iniState.col(0), finState.col(0), map, hPolys,
                            route))
    {
      return std::nullopt;
    }

    const int pieceN = static_cast<int>(hPolys.size()) + 1;

    double totalLength = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
      totalLength += (route[i] - route[i - 1]).norm();
    }
    const double v_max     = std::max(0.1, config_.problem.cost.v_max);
    const double segLength = std::max(1e-3, totalLength / pieceN);

    Eigen::VectorXd  initTime(pieceN);
    Eigen::Matrix3Xd initPointsPos(3, pieceN - 1);
    double           acc      = 0.0;
    std::size_t      routeIdx = 0;
    for (int i = 0; i < pieceN - 1; i++)
    {
      const double target = segLength * (i + 1);
      while (routeIdx + 1 < route.size() &&
             acc + (route[routeIdx + 1] - route[routeIdx]).norm() < target)
      {
        acc += (route[routeIdx + 1] - route[routeIdx]).norm();
        ++routeIdx;
      }
      initPointsPos.col(i) = route[routeIdx];
      initTime(i)          = segLength / v_max;
    }
    initTime(pieceN - 1) = segLength / v_max;

    gcopter::GCOPTER_PolytopeSFC gcopter;
    TrajectoryRepresentation<5>  traj;
    if (!gcopter.setup_basic_trajectory(iniState, finState, initTime,
                                        initPointsPos, hPolys,
                                        config_.problem.minco.smoothing_eps,
                                        config_.problem.minco.integral_intervs))
    {
      log(LogLevel::Error, "GCOPTER setup failed (corridor)");
      return std::nullopt;
    }

    if (std::isinf(gcopter.optimize(traj, config_.problem.minco.rel_cost_tol)))
    {
      log(LogLevel::Error, "GCOPTER optimization diverged (corridor)");
      return std::nullopt;
    }

    return ReferenceTrajectory(std::move(traj));
  }

}  // namespace planner::core
