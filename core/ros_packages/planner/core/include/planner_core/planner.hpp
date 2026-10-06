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
#include <basic_trajectories/circle.hpp>
#include <basic_trajectories/lemniscate.hpp>
#include <basic_trajectories/line.hpp>
#include <basic_trajectories/sin.hpp>
#include <functional>
#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>
#include <memory>
#include <optional>
#include <planner_core/problem_config.hpp>
#include <planner_core/trajectory.hpp>
#include <string>
#include <vector>

namespace planner::core
{

  /**
   * @brief Basic (seed) trajectory family, mirrors the yaml `basic_traj.type`.
   */
  enum class BasicTrajectoryType
  {
    LEMNISCATE = 1,
    CIRCLE     = 2,
    SIN        = 3,
    LINE       = 4
  };

  /// Severity for the optional logging sink.
  enum class LogLevel
  {
    Debug,
    Info,
    Warn,
    Error
  };

  /// Optional logging sink; default prints to stderr.
  using LogSink = std::function<void(LogLevel, const std::string &)>;

  /**
   * @brief Pure trajectory optimizer (no ROS, no files, no clock).
   *
   * Builds the OCP (seed trajectory + polytope SFC), solves it with
   * GCOPTER/MINCO, and returns a continuous ReferenceTrajectory (flatness
   * evaluation at arbitrary time). Consumers:
   *   - the ROS node (`planner_node`), which samples it on the NMPC grid, and
   *   - the Python binding, which exposes it to the benchmark pipeline.
   */
  class Planner
  {
   public:
    /**
     * @brief Constructor
     *
     * @param config Loaded OCP/problem configuration
     * @param log    Optional logging sink (defaults to stderr)
     */
    explicit Planner(ProblemConfig config, LogSink log = {});

    // Disable copy (owns the gcopter cost singleton mirror config)
    Planner(const Planner &)            = delete;
    Planner &operator=(const Planner &) = delete;

    /**
     * @brief Run the (one-shot) optimization.
     *
     * @return the continuous reference trajectory, or std::nullopt on failure
     */
    std::optional<ReferenceTrajectory> plan();

    /// Problem configuration in use.
    const ProblemConfig &config() const { return config_; }

   private:
    void init_basic_trajectory_type(int type);
    void create_basic_trajectory_sfc(std::vector<Eigen::MatrixX4d> &hPolys);
    void log(LogLevel level, const std::string &msg) const;

    ProblemConfig                                   config_;
    LogSink                                         log_;
    std::unique_ptr<basic_trajectories::Trajectory> basic_traj_;
  };

}  // namespace planner::core
