/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <chrono>
#include <cmath>
#include <iostream>
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

}  // namespace planner::core
