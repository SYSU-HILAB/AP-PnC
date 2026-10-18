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
                         config_.problem.map.sfc_progress_m,
                         config_.problem.map.sfc_range_m, hPolys);
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

    if (config_.problem.map.speed_controlled)
    {
      // Explicit speed-controlled fly-through mode, not a failed-optimizer
      // fallback. Time-optimal GCOPTER does not impose constant cruise speed.
      // Connect full PVA with a quintic and select duration for minimum speed
      // ripple. Accept ONLY a map/SFC-checked, dynamically feasible connection.
      const double speed    = config_.problem.cost.v_max;
      const double distance = (finState.col(0) - iniState.col(0)).norm();
      const bool   cruise =
          iniState.col(1).norm() > 1e-3 && finState.col(1).norm() > 1e-3;
      if (!iniState.allFinite() || !finState.allFinite() || distance < 1e-3 ||
          speed <= 0)
        return std::nullopt;
      double base = 2.0 * distance / speed;
      if (cruise)
      {
        const double angle = std::acos(std::clamp(
            iniState.col(1).normalized().dot(finState.col(1).normalized()),
            -1.0, 1.0));
        base = distance / speed *
               (angle < 1e-6 ? 1.0 : angle / (2 * std::sin(angle / 2)));
      }
      auto connect = [&](double t)
      {
        Eigen::Matrix<double, 3, 6> c;
        const Eigen::Vector3d       dp = finState.col(0) - iniState.col(0) -
                                         t * iniState.col(1) -
                                         0.5 * t * t * iniState.col(2);
        const Eigen::Vector3d       dv =
            finState.col(1) - iniState.col(1) - t * iniState.col(2);
        const Eigen::Vector3d da = finState.col(2) - iniState.col(2);
        c.col(5)                 = iniState.col(0);
        c.col(4)                 = iniState.col(1);
        c.col(3)                 = iniState.col(2) / 2;
        c.col(2) = 10 * dp / std::pow(t, 3) - 4 * dv / (t * t) + da / (2 * t);
        c.col(1) =
            -15 * dp / std::pow(t, 4) + 7 * dv / std::pow(t, 3) - da / (t * t);
        c.col(0) = 6 * dp / std::pow(t, 5) - 3 * dv / std::pow(t, 4) +
                   da / (2 * std::pow(t, 3));
        TrajectoryRepresentation<5> representation;
        representation.emplace_back(t, c);
        return representation;
      };
      double duration   = base,
             best_score = std::numeric_limits<double>::infinity();
      for (int i = 0; i < (cruise ? 101 : 1); ++i)
      {
        const double t         = base * (cruise ? 0.9 + i * 0.0025 : 1.0);
        const auto   candidate = connect(t);
        double       score     = 0;
        for (int j = 0; j <= 100; ++j)
          score = std::max(
              score, std::abs(candidate.getVel(t * j / 100).norm() - speed));
        if (score < best_score)
        {
          best_score = score;
          duration   = t;
        }
      }
      ReferenceTrajectory candidate(connect(duration));
      // The free-space A* chord is not the dynamic arc: certify the actual
      // fly-through curve with FIRI rather than forcing it into chord boxes.
      // Pointwise inflated-map checks below remain mandatory as well.
      const int samples = static_cast<int>(std::ceil(duration / 0.01));
      for (int i = 0; i <= samples; ++i)
        if (map.query(
                candidate.representation().getPos(duration * i / samples)))
        {
          log(LogLevel::Error,
              "Speed-controlled connector intersects inflated occupancy");
          return std::nullopt;
        }
      std::vector<Eigen::Vector3d> surface, curve;
      map.getSurf(surface);
      if (!surface.empty())
      {
        const int knots =
            std::max(2, static_cast<int>(std::ceil(duration * speed / 3.0)));
        for (int i = 0; i <= knots; ++i)
          curve.push_back(candidate.sample(duration * i / knots).p);
        sfc_gen::convexCover(curve, surface, map.getOrigin(), map.getCorner(),
                             config_.problem.map.sfc_progress_m,
                             config_.problem.map.sfc_range_m, hPolys);
        // Do not shortcut this cover: overlap preserves connectivity, NOT
        // containment of the already speed-constrained curved trajectory.
      }
      for (int i = 0; i <= samples; ++i)
      {
        const auto rp          = candidate.sample(duration * i / samples);
        bool       in_corridor = false;
        for (const auto &poly : hPolys)
          in_corridor =
              in_corridor ||
              (poly.leftCols<3>() * rp.p + poly.col(3)).maxCoeff() <= 1e-6;
        if (!in_corridor || map.query(rp.p) || rp.v.norm() > speed * 1.05 ||
            (cruise && rp.v.norm() < speed * 0.95) ||
            rp.a.norm() > config_.problem.cost.acc_max ||
            rp.thrust < config_.problem.cost.thrust_min ||
            rp.thrust > config_.problem.cost.thrust_max)
        {
          log(LogLevel::Error,
              !in_corridor      ? "Speed-controlled connector leaves FIRI cover"
              : map.query(rp.p) ? "Speed-controlled connector intersects map"
                                : "Speed-controlled connector violates "
                                  "speed/acceleration/thrust bounds");
          return std::nullopt;
        }
      }
      return candidate;
    }

    for (const auto &poly : hPolys)
    {
      Eigen::Matrix3Xd vertices;
      if (!geo_utils::enumerateVs(poly, vertices) || vertices.cols() < 4)
      {
        std::cerr << "Degenerate FIRI polytope:\n" << poly << std::endl;
        return std::nullopt;
      }
    }
    const int pieceN = static_cast<int>(hPolys.size()) + 1;

    double totalLength = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
      totalLength += (route[i] - route[i - 1]).norm();
    }
    const double segLength = std::max(1e-3, totalLength / pieceN);

    Eigen::Vector3d tangent = finState.col(0) - iniState.col(0);
    if (tangent.norm() < 1e-6)
    {
      tangent = Eigen::Vector3d::UnitX();
    }
    Eigen::Vector3d lateral =
        tangent.normalized().cross(Eigen::Vector3d::UnitZ());
    if (lateral.norm() < 1e-6)
    {
      lateral = Eigen::Vector3d::UnitY();
    }
    lateral.normalize();

    // GCOPTER's corridor setup can be numerically finicky for some A*
    // route/polytope shapes (exact colinearity of seed points with the
    // boundary states, or a seed time allocation the line search can't
    // reach from): retry a small set of perturbations -- alternating the
    // sideways nudge direction/magnitude and the seed cruise-speed guess --
    // before giving up, the same "try a different starting point" tactic a
    // human would use on a stuck nonlinear solve.
    const struct
    {
      double nudge;
      double speedDivisor;
    } attempts[] = {
        {0.15, 8.0}, {-0.15, 8.0}, {0.3, 8.0}, {-0.3, 4.0}, {0.0, 8.0},
    };

    for (const auto &attempt : attempts)
    {
      // Seed conservatively (a slow cruise-speed guess the optimizer
      // shrinks) rather than near v_max (an optimistic guess it must
      // stretch): shrinking an overestimated time allocation is the
      // direction GCOPTER's line search handles reliably.
      // A rest-to-rest seed can be slow; a fly-through boundary cannot.
      // Seeding a 10 m/s endpoint with 1 m/s segment times induces large
      // polynomial reversals/overshoot before the optimizer even starts.
      const double boundary_speed =
          std::max(iniState.col(1).norm(), finState.col(1).norm());
      const double v_max =
          std::max({0.1, config_.problem.cost.v_max / attempt.speedDivisor,
                    boundary_speed * (attempt.speedDivisor > 4 ? 0.9 : 1.1)});

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
        // Nudge the seed slightly off the A* centerline: exact colinearity
        // of interior seed points with the boundary states is degenerate
        // for GCOPTER's setup, the same reason plan()'s own seed offsets
        // its interior points sideways rather than placing them on the
        // nominal path.
        initPointsPos.col(i) = route[routeIdx] + lateral * attempt.nudge;
        initTime(i)          = segLength / v_max;
      }
      initTime(pieceN - 1) = segLength / v_max;

      gcopter::GCOPTER_PolytopeSFC gcopter;
      TrajectoryRepresentation<5>  traj;
      if (!gcopter.setup_basic_trajectory(
              iniState, finState, initTime, initPointsPos, hPolys,
              config_.problem.minco.smoothing_eps,
              config_.problem.minco.integral_intervs))
      {
        continue;
      }
      try
      {
        if (!std::isfinite(
                gcopter.optimize(traj, config_.problem.minco.rel_cost_tol)))
          continue;
      }
      catch (const std::invalid_argument &)
      {
        // A line-search trial can overflow the flatness input. Treat that
        // numerical seed as failed and use the next existing retry; never
        // publish its non-finite polynomial as a valid reference.
        continue;
      }
      bool finite = traj.getPieceNum() > 0;
      for (const auto &piece : traj)
        finite = finite && std::isfinite(piece.getDuration()) &&
                 piece.getDuration() > 0.0 && piece.getCoeffMat().allFinite();
      if (!finite)
        continue;
      return ReferenceTrajectory(std::move(traj));
    }

    log(LogLevel::Error,
        "GCOPTER optimization diverged (corridor) after all seed retries");
    return std::nullopt;
  }

}  // namespace planner::core
