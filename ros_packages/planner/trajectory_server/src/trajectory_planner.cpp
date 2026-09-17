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

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <tailsitter_df/tailsitter_df.hpp>
#include <trajectory_server/planning_utils.hpp>
#include <trajectory_server/trajectory_planner.hpp>

using std::chrono_literals::operator""s;

namespace trajectory_server
{

  // Static AeroAlphaSolver instance for df_lyu_alpha lookup from force balance
  // equation Solves: S_ap_z*cos(alpha) + S_ap_x*sin(alpha) = F_aero_z/mass
  static auto g_lyu_aerodynamics =
      std::make_shared<aerodynamics::LyuAerodynamics>();
  static aerodynamics::AeroAlphaSolver g_alpha_solver(g_lyu_aerodynamics);

  GlobalPlanner::GlobalPlanner() : Node("global_planning_node")
  {
    // Prior-free planning config: direct yaml read (bringup), no ROS params.
    // Conventional cwd-relative path; containers run from /workspace.
    config.load("bringup/config/planning.yaml");
    tailsitter_df::Model::getInstance().setParameters(
        config.problem.flatness.phi, config.problem.flatness.mass);

    auto reliable_qos =
        rclcpp::QoS(10).transient_local().reliable().keep_last(1);
    trajVizPub = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "traj_viz", reliable_qos);

    trajPub = this->create_publisher<RefTrajectory>(config.io.traj_topic,
                                                    reliable_qos);

    init_basic_trajectory_type(config.problem.basic_traj.type);
    timer = this->create_wall_timer(2s, std::bind(&GlobalPlanner::plan, this));
  }

  void GlobalPlanner::init_basic_trajectory_type(const int type)
  {
    // basic_trajniscate
    if (type == 1)
    {
      basic_traj = std::make_unique<basic_trajectories::Lemniscate>(
          config.problem.basic_traj.radius, config.problem.basic_traj.rate,
          config.problem.basic_traj.height,
          config.problem.basic_traj.curvature_factor);
      return;
    }
    if (type == 2)
    {
      basic_traj = std::make_unique<basic_trajectories::Circle>(
          config.problem.basic_traj.radius, config.problem.basic_traj.rate,
          config.problem.basic_traj.height);
      return;
    }
    if (type == 3)
    {
      basic_traj = std::make_unique<basic_trajectories::Sin>(
          config.problem.basic_traj.radius, config.problem.basic_traj.wave_len,
          config.problem.basic_traj.height);
      config.problem.basic_traj.rate = basic_traj->getAngularSpeed();
      return;
    }
    if (type == 4)
    {
      basic_traj = std::make_unique<basic_trajectories::Line>(
          config.problem.basic_traj.radius, config.problem.basic_traj.rate,
          config.problem.basic_traj.height);
      return;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "The trajecotry type is not supported!");
    }
  }

  void GlobalPlanner::create_basic_trajecotry_sfc(
      std::vector<Eigen::MatrixX4d> &hPolys)
  {
    Eigen::RowVector4d h1, h2, h3, h4, h5, h6;

    Eigen::MatrixX4d H;
    H.resize(6, 4);

    auto route = basic_traj->samplingPointArray(config.problem.piece_n);

    auto traj_vis_msg            = basic_traj->toMsg();
    traj_vis_msg.header.frame_id = config.problem.frame;
    traj_vis_msg.header.stamp    = this->get_clock()->now();
    this->trajVizPub->publish(traj_vis_msg);

    Eigen::Vector3d pc, v_direction, u_direction;
    for (size_t i = 1; i < route.size() - 1; i++)
    {
      auto point = route[i];
      pc         = point.col(0);  // Center of the box

      // Define the half-lengths of the box in each direction
      double front   = config.problem.box.front;
      double back    = config.problem.box.back;
      double outside = config.problem.box.side_outside;
      double inside  = config.problem.box.side_inside;
      double up      = config.problem.box.up;
      double down    = config.problem.box.down;

      // Create the six planes of the box (axis-aligned)
      // Each plane is defined as [a, b, c, d] where ax + by + cz + d <= 0
      // Front (x+)
      h1 << 1, 0, 0, -pc.x() - front;
      // Back (x-)
      h2 << -1, 0, 0, pc.x() - back;
      // Outside (y+)
      h3 << 0, 1, 0, -pc.y() - outside;
      // Inside (y-)
      h4 << 0, -1, 0, pc.y() - inside;
      // Up (z+)
      h5 << 0, 0, 1, -pc.z() - up;
      // Down (z-)
      h6 << 0, 0, -1, pc.z() - down;

      // Stack the planes into a matrix
      H << h1, h2, h3, h4, h5, h6;
      hPolys.push_back(H);
    }
  }

  void GlobalPlanner::plan()
  {
    if (init_setup)
    {
      return;
    }
    std::vector<Eigen::MatrixX4d> hPolys;
    RCLCPP_INFO(this->get_logger(), "In Planning");
    create_basic_trajecotry_sfc(hPolys);

    Eigen::Matrix3d iniState, finState;
    auto route = basic_traj->samplingPointArray(config.problem.piece_n);
    iniState << route.front().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    finState << route.back().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    gcopter::GCOPTER_PolytopeSFC gcopter;

    traj.clear();
    Eigen::VectorXd initTime;
    initTime.resize(config.problem.piece_n);
    initTime.setConstant(2 * M_PI / config.problem.basic_traj.rate /
                         config.problem.piece_n);

    Eigen::Matrix3Xd initPointsPos(3, config.problem.piece_n - 1);
    for (int i = 0; i < config.problem.piece_n - 1; i++)
    {
      Eigen::Vector3d v = route[i].col(1);
      v.normalize();
      Eigen::Vector3d u = v.cross(Eigen::Vector3d::UnitZ());
      initPointsPos.col(i) =
          route[i + 1].col(0) + u * config.problem.box.side_outside * 0.8;
    }

    if (!init_setup)
    {
      // Reset body frame before optimization
      tailsitter_df::Data::getInstance().reset();

      if (!gcopter.setup_basic_trajectory(
              iniState, finState, initTime, initPointsPos, hPolys,
              config.problem.minco.smoothing_eps,
              config.problem.minco.integral_intervs))
      {
        return;
      }
      auto opt_start = std::chrono::high_resolution_clock::now();

      if (std::isinf(gcopter.optimize(traj, config.problem.minco.rel_cost_tol)))
      {
        return;
      }
      auto opt_end = std::chrono::high_resolution_clock::now();

      std::cerr << "PLANNING Duration"
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       opt_end - opt_start)
                       .count()
                << "ms" << std::endl;

      init_setup = true;

      auto route_msg            = basic_traj->toMsg(100);
      route_msg.header.frame_id = config.problem.frame;
      route_msg.header.stamp    = this->get_clock()->now();
      this->trajVizPub->publish(route_msg);

      auto &tailsitter_data = tailsitter_df::Data::getInstance();
      tailsitter_data.reset();

      // Reset Newton-Raphson solver for new trajectory
      // Note: Solver configuration is already read from the yaml config in
      // constructor We just need to reset the temporal coherence state here
      const char *env_p = std::getenv("AP_PNC_DIR");
      if (env_p != nullptr)
      {
        // Prior-based experiment config (Lyu force-balance initial alpha); the
        // prior-free phi-theory path does not depend on this file.
        std::string yaml_path =
            std::string(env_p) + "/benchmark/config/aero_alpha_solver.yaml";
        try
        {
          YAML::Node config = YAML::LoadFile(yaml_path);
          double     initial_alpha_deg =
              config["initial_alpha_deg"].as<double>(90.0);
          g_alpha_solver.resetAlpha(initial_alpha_deg);
          RCLCPP_INFO(this->get_logger(),
                      "AeroAlphaSolver reset with initial alpha: %.6f degrees "
                      "from config/aero_alpha_solver.yaml",
                      initial_alpha_deg);
        }
        catch (const YAML::Exception &e)
        {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to parse config/aero_alpha_solver.yaml: %s. "
                      "Using default initial_alpha_deg = 90.0",
                      e.what());
          g_alpha_solver.resetAlpha(90.0);
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(),
                    "AP_PNC_DIR environment variable not set. Using default "
                    "initial_alpha_deg = 90.0");
        g_alpha_solver.resetAlpha(90.0);
      }

      control_traj_msg.points.clear();
      // Track df_lyu_quat to avoid quaternion sign flips between samples
      Eigen::Quaterniond prev_df_lyu_quat = Eigen::Quaterniond::Identity();
      bool               df_lyu_quat_initialized = false;
      auto              &csv_file = init_planning_csv_file(config);
      RCLCPP_INFO(this->get_logger(), "CSV file opened");
      for (double t = 0.0; t <= traj.getTotalDuration() + DBL_EPSILON;
           t += config.io.traj_pub_rsl)
      {
        Eigen::Vector3d omg;
        double          thrust;

        tailsitter_data.df_backward_auto(traj.getVel(t), traj.getAcc(t),
                                         traj.getJer(t), thrust, omg);

        {
          // Create trajectory sample using factory method
          TrajectorySampleData sample = TrajectorySampleData::from_trajectory(
              traj, t, tailsitter_data, config.problem.flatness.mass,
              g_alpha_solver, prev_df_lyu_quat, df_lyu_quat_initialized);

          // Update continuity state
          if (!df_lyu_quat_initialized)
          {
            df_lyu_quat_initialized = true;
          }
          prev_df_lyu_quat = sample.quat_lyu;

          write_trajectory_sample(csv_file, sample);
        }

        auto p = traj.getPos(t);
        auto v = traj.getVel(t);

        auto yb_df = tailsitter_data.getYb();

        // reference trajectory uses quad FLU body frame, so negate yb
        Eigen::Vector3d YB =
            (Eigen::Vector3d() << -yb_df(0), -yb_df(1), -yb_df(2)).finished();

        control_traj_msg.timestamp = this->get_clock()->now();
        auto traj_point            = TrajectoryPoint();
        traj_point.point[TrajectoryPoint::POS_START_INDEX]     = p[0];
        traj_point.point[TrajectoryPoint::POS_START_INDEX + 1] = p[1];
        traj_point.point[TrajectoryPoint::POS_START_INDEX + 2] = p[2];
        traj_point.point[TrajectoryPoint::VEL_START_INDEX]     = v[0];
        traj_point.point[TrajectoryPoint::VEL_START_INDEX + 1] = v[1];
        traj_point.point[TrajectoryPoint::VEL_START_INDEX + 2] = v[2];
        traj_point.point[TrajectoryPoint::YB_START_INDEX]      = YB[0];
        traj_point.point[TrajectoryPoint::YB_START_INDEX + 1]  = YB[1];
        traj_point.point[TrajectoryPoint::YB_START_INDEX + 2]  = YB[2];
        // default set to zero
        control_traj_msg.points.push_back(traj_point);
      }

      trajPub->publish(control_traj_msg);
      RCLCPP_WARN(this->get_logger(), "Trajectory Published !!!\n");
      std::vector<Eigen::Vector3d> route_path;
      for (int i = 0; i < config.problem.piece_n; i++)
      {
        route_path.emplace_back(route[i].col(0));
      }
      if (traj.getPieceNum() > 0)
      {
        // Record routes to separate CSV file after successful optimization
        auto &routes_csv_file = init_routes_csv_file(config);
        for (size_t i = 0; i < route.size(); i++)
        {
          const auto route_sample =
              RouteSampleData::from_trajectory_point(route[i], i);
          write_route_sample(routes_csv_file, route_sample);
        }
        routes_csv_file.close();
        RCLCPP_INFO(this->get_logger(),
                    "Routes CSV file created, and filled out.");
      }
      csv_file.close();
    }
  }

}  // namespace trajectory_server
