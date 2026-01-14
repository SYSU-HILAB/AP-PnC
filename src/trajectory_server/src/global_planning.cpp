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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <cstdlib>
#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interface/msg/ref_trajectory.hpp>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
using std::chrono_literals::operator""s;
using RefTrajectory   = interface::msg::RefTrajectory;
using TrajectoryPoint = interface::msg::TrajectoryPoint;

#include <basic_trajectories/circle.hpp>
#include <basic_trajectories/lemniscate.hpp>
#include <basic_trajectories/line.hpp>
#include <basic_trajectories/sin.hpp>
#include <basic_trajectories/vertical_loop.hpp>

#include <aerodynamics/aero_alpha_solver.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

// Forward declarations
std::string format_float_1f(double value);
std::string get_trajectory_type_name(int type);
std::filesystem::path get_artifacts_temp_dir();

// Static AeroAlphaSolver instance for df_lyu_alpha lookup from force balance equation
// Solves: S_ap_z*cos(alpha) + S_ap_x*sin(alpha) = F_aero_z/mass
static auto g_lyu_aerodynamics = std::make_shared<aerodynamics::LyuAerodynamics>();
static aerodynamics::AeroAlphaSolver g_alpha_solver(g_lyu_aerodynamics);

// create a enum class for the basic trajectory type
enum class BasicTrajectoryType
{
  LEMNISCATE = 1,
  CIRCLE     = 2,
  SIN        = 3,
  LINE       = 4
};

struct Config
{
  double smoothingEps;
  int    integralIntervs;
  double relCostTol;
  double uavScale;
  // trajectory publish time resolution;
  double              traj_pub_rsl = 0.02;
  std::string         trajTopic    = "control_traj";
  std::vector<double> finVel;
  int pieceN = 20;  // 20 piece meaning 21 points including start and end, and
                    // that is to say, only pieceN - 1 control points
  int controlPoints = pieceN - 1;

  double box_side_inside_length  = 1;
  double box_side_outside_length = 1;
  double box_up_length           = 1;
  double box_down_length         = 0.1;
  double box_front_length        = 1;
  double box_back_length         = 1;

  std::string frame = "world";
  // basic trajectory set
  double basic_trajectory_rate             = 0.8;
  double basic_trajectory_radius           = 10;
  double basic_trajectory_height           = 5;
  int    basic_trajectory_type             = 1;
  double basic_trajectory_curvature_factor = 0.0;
  double basic_traj_wave_len               = 10.0;

  double trajecotry_cirled = true;

  double mass = 2.0;  // Vehicle mass in kg (default from tailsitter_df config)

  gcopter::CostConfig &cost_config;

  Config(rclcpp::Node *node) : cost_config(gcopter::CostConfig::getInstance())
  {
    frame = node->declare_parameter("Frame", "world");

    mass = node->declare_parameter("mass", 2.0);

    uavScale = node->declare_parameter("UavScale", 1.0);

    smoothingEps    = node->declare_parameter("SmoothingEps", 1e-2);
    integralIntervs = node->declare_parameter("IntegralIntervs", 16);
    relCostTol      = node->declare_parameter("RelCostTol", 1e-4);
    pieceN          = node->declare_parameter("PieceN", 10);

    box_side_inside_length =
        node->declare_parameter("BoxSideInsideLength", 0.1);
    box_side_outside_length =
        node->declare_parameter("BoxSideOutsideLength", 1.0);
    box_down_length  = node->declare_parameter("BoxDownLength", 0.1);
    box_up_length    = node->declare_parameter("BoxUpLength", 0.5);
    box_front_length = node->declare_parameter("BoxFrontLength", 1.5);
    box_back_length  = node->declare_parameter("BoxBackLength", 1.5);

    basic_trajectory_rate = node->declare_parameter("BasicTrajectoryRate", 0.8);
    basic_trajectory_radius =
        node->declare_parameter("BasicTrajectoryRadius", 10.0);
    basic_trajectory_height =
        node->declare_parameter("BasicTrajectoryHeight", 5.0);
    basic_trajectory_type = node->declare_parameter("BasicTrajectoryType", 1);
    basic_trajectory_curvature_factor =
        node->declare_parameter("BasicTrajectoryCurvatureFactor", 0.0);
    basic_traj_wave_len =
        node->declare_parameter("BasicTrajectoryWaveLen", 10.0);

    trajecotry_cirled = node->declare_parameter("BasicTrajectoryCirled", true);

    // pub config
    traj_pub_rsl = node->declare_parameter("traj_pub_rsl", 0.02);
    trajTopic    = node->declare_parameter("trajTopic", "control_traj");
    // cost_config
    cost_config.v_max     = node->declare_parameter("v_max", 10.0);
    cost_config.omg_x_max = node->declare_parameter("omg_x_max", 3.0);
    cost_config.omg_y_max = node->declare_parameter("omg_y_max", 3.0);
    cost_config.omg_z_max = node->declare_parameter("omg_z_max", 3.0);

    cost_config.thrust_max = node->declare_parameter("thrust_max", 20.0);
    cost_config.thrust_min = node->declare_parameter("thrust_min", 1.0);
    cost_config.acc_max    = node->declare_parameter("acc_max", 1.0);

    cost_config.thrust_weight = node->declare_parameter("thrust_weight", 1e5);
    cost_config.vel_weight    = node->declare_parameter("vel_weight", 1e4);
    cost_config.acc_weight    = node->declare_parameter("acc_weight", 1e4);
    cost_config.omg_x_weight  = node->declare_parameter("omg_x_weight", 1e3);
    cost_config.omg_y_weight  = node->declare_parameter("omg_y_weight", 1e3);
    cost_config.omg_z_weight  = node->declare_parameter("omg_z_weight", 1e3);
    cost_config.time_weight   = node->declare_parameter("time_weight", 1e2 * 5);
    cost_config.omg_consistent_weight =
        node->declare_parameter("omg_consistent_weight", 1e3);
  }

  void UpdateConfig(const rclcpp::Node *node)
  {
    node->get_parameter("SmoothingEps", smoothingEps);
    node->get_parameter("IntegralIntervs", integralIntervs);
    node->get_parameter("RelCostTol", relCostTol);
    node->get_parameter("UavScale", uavScale);

    node->get_parameter("FinVel", finVel);
    node->get_parameter("PieceN", pieceN);

    node->get_parameter("BoxSideInsideLength", box_side_inside_length);
    node->get_parameter("BoxSideOutsideLength", box_side_outside_length);
    node->get_parameter("BoxUpLength", box_up_length);
    node->get_parameter("BoxDownLength", box_down_length);
    node->get_parameter("BoxFrontLength", box_front_length);
    node->get_parameter("BoxBackLength", box_back_length);

    node->get_parameter("Frame", frame);

    node->get_parameter("mass", mass);

    node->get_parameter("BasicTrajectoryRate", basic_trajectory_rate);
    node->get_parameter("BasicTrajectoryRadius", basic_trajectory_radius);
    node->get_parameter("BasicTrajectoryHeight", basic_trajectory_height);
    node->get_parameter("BasicTrajectoryType", basic_trajectory_type);
    node->get_parameter("BasicTrajectoryCurvatureFactor",
                        basic_trajectory_curvature_factor);
    node->get_parameter("BasicTrajectoryWaveLen", basic_traj_wave_len);
    node->get_parameter("TrajectoryCirled", trajecotry_cirled);
    // update cost_config
    node->get_parameter("v_max", cost_config.v_max);
    node->get_parameter("omg_x_max", cost_config.omg_x_max);
    node->get_parameter("omg_y_max", cost_config.omg_y_max);
    node->get_parameter("omg_z_max", cost_config.omg_z_max);
    node->get_parameter("thrust_max", cost_config.thrust_max);
    node->get_parameter("thrust_min", cost_config.thrust_min);
    node->get_parameter("acc_max", cost_config.acc_max);
    node->get_parameter("thrust_weight", cost_config.thrust_weight);
    node->get_parameter("vel_weight", cost_config.vel_weight);
    node->get_parameter("omg_x_weight", cost_config.omg_x_weight);
    node->get_parameter("omg_y_weight", cost_config.omg_y_weight);
    node->get_parameter("omg_z_weight", cost_config.omg_z_weight);
    node->get_parameter("time_weight", cost_config.time_weight);
    node->get_parameter("acc_weight", cost_config.acc_weight);
    node->get_parameter("omg_consistent_weight",
                        cost_config.omg_consistent_weight);

    node->get_parameter("traj_pub_rsl", traj_pub_rsl);
    node->get_parameter("trajTopic", trajTopic);
  }
};

/**
 * Stateless helper to compute df_lyu_quat from velocity, acceleration, and lyu_alpha.
 * Follows the same algorithm as df_forward_nominal but uses provided alpha value.
 * Handles velocity singularity (v < vThreshold) similar to df_forward_singularity_vel.
 */
Eigen::Quaterniond compute_df_lyu_quat(const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &acc,
                                       double lyu_alpha)
{
  const Eigen::Vector3d gravity(0, 0, -9.81);
  constexpr double vThreshold = 0.5;

  // Singularity handling for low velocity
  if (vel.norm() < vThreshold)
  {
    // df_forward_singularity_vel logic
    Eigen::Vector3d eta = acc - gravity;
    Eigen::Vector3d xb = eta.normalized();

    // Interpolate yb based on velocity scaling
    double scaling = vel.norm() / vThreshold;
    Eigen::Vector3d yb_from_v = vel.cross(eta).normalized();
    // For cirled trajectories, zb_fixed is North direction (0, 1, 0) in ENU
    Eigen::Vector3d zb_fixed(0, 1, 0);
    Eigen::Vector3d yb_from_fixed_frame = zb_fixed.cross(xb).normalized();

    Eigen::Vector3d yb = ((1.0 - scaling) * yb_from_fixed_frame + scaling * yb_from_v).normalized();
    Eigen::Vector3d zb = xb.cross(yb).normalized();

    Eigen::Matrix3d rot;
    rot << xb, yb, zb;
    return Eigen::Quaterniond(rot);
  }
  else
  {
    // df_forward_nominal logic with lyu_alpha
    Eigen::Vector3d x_s = vel / vel.norm();

    // Compute z_s (direction of thrust, perpendicular to velocity)
    Eigen::Vector3d temp = -(Eigen::Matrix3d::Identity() - x_s * x_s.transpose()) * (acc - gravity);
    Eigen::Vector3d z_s = temp / temp.norm();

    // Compute y_s (completes the right-handed coordinate system)
    Eigen::Vector3d y_s = z_s.cross(x_s);

    // Use lyu_alpha instead of df_phi_alpha
    double alpha = lyu_alpha;

    // Compute body frame axes with alpha rotation
    Eigen::Vector3d xb = std::cos(alpha) * x_s - std::sin(alpha) * z_s;
    Eigen::Vector3d yb = y_s;
    Eigen::Vector3d zb = std::cos(alpha) * z_s + std::sin(alpha) * x_s;

    // Build rotation matrix and convert to quaternion
    Eigen::Matrix3d rot;
    rot << xb, yb, zb;
    return Eigen::Quaterniond(rot);
  }
}

std::ofstream &init_routes_csv_file(const Config &config);
std::ofstream &init_planning_csv_file(const Config &config);

class GlobalPlanner : public rclcpp::Node
{
 private:
  Config                                                           config;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  // Visualizer visualizer;
  RefTrajectory                                               control_traj_msg;
  Trajectory<5>                                               traj;
  double                                                      trajStamp;
  std::unique_ptr<basic_trajectories::Trajectory>             basic_traj;
  rclcpp::TimerBase::SharedPtr                                timer;
  rclcpp::Publisher<RefTrajectory>::SharedPtr                 trajPub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajVizPub;

  bool init_setup = false;

 public:
  GlobalPlanner() : Node("global_planning_node"), config(this)
  {
    config.UpdateConfig(this);
    auto reliable_qos =
        rclcpp::QoS(10).transient_local().reliable().keep_last(1);
    trajVizPub = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "traj_viz", reliable_qos);

    trajPub =
        this->create_publisher<RefTrajectory>(config.trajTopic, reliable_qos);

    init_basic_trajectory_type(config.basic_trajectory_type);
    timer = this->create_wall_timer(2s, std::bind(&GlobalPlanner::plan, this));

    RCLCPP_INFO(this->get_logger(), "Planning");
  }

  void init_basic_trajectory_type(const int type)
  {
    // basic_trajniscate
    if (type == 1)
    {
      basic_traj = std::make_unique<basic_trajectories::Lemniscate>(
          config.basic_trajectory_radius, config.basic_trajectory_rate,
          config.basic_trajectory_height,
          config.basic_trajectory_curvature_factor);
      return;
    }
    if (type == 2)
    {
      basic_traj = std::make_unique<basic_trajectories::Circle>(
          config.basic_trajectory_radius, config.basic_trajectory_rate,
          config.basic_trajectory_height);
      return;
    }
    if (type == 3)
    {
      basic_traj = std::make_unique<basic_trajectories::Sin>(
          config.basic_trajectory_radius, config.basic_traj_wave_len,
          config.basic_trajectory_height);
      config.basic_trajectory_rate = basic_traj->getAngularSpeed();
      return;
    }
    if (type == 4)
    {
      if (!config.trajecotry_cirled)
      {
        basic_traj = std::make_unique<basic_trajectories::Line>(
            config.basic_trajectory_radius, config.basic_trajectory_rate,
            config.basic_trajectory_height);
        return;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),
                     "The line trajectory must not be cirled!");
        abort();
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "The trajecotry type is not supported!");
    }
  }
  void create_basic_trajecotry_sfc(std::vector<Eigen::MatrixX4d> &hPolys)
  {
    Eigen::RowVector4d h1, h2, h3, h4, h5, h6;

    Eigen::MatrixX4d H;
    H.resize(6, 4);

    std::cerr << "Init H\n" << H << std::endl;
    std::cerr << "Set up\n" << std::endl;
    auto route = basic_traj->samplingPointArray(config.pieceN);

    for (size_t i = 0; i < route.size(); i++)
    {
      std::cerr << "route[" << i << "]\n" << route[i] << std::endl;
    }

    auto traj_vis_msg            = basic_traj->toMsg();
    traj_vis_msg.header.frame_id = config.frame;
    traj_vis_msg.header.stamp    = this->get_clock()->now();
    this->trajVizPub->publish(traj_vis_msg);

    Eigen::Vector3d pc, v_direction, u_direction;
    for (size_t i = 1; i < route.size() - 1; i++)
    {
      auto point = route[i];
      pc         = point.col(0);  // Center of the box

      // Define the half-lengths of the box in each direction
      double front   = config.box_front_length;
      double back    = config.box_back_length;
      double outside = config.box_side_outside_length;
      double inside  = config.box_side_inside_length;
      double up      = config.box_up_length;
      double down    = config.box_down_length;

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

  inline void plan()
  {
    if (init_setup)
    {
      return;
    }
    std::vector<Eigen::MatrixX4d> hPolys;
    RCLCPP_INFO(this->get_logger(), "Planning");
    create_basic_trajecotry_sfc(hPolys);

    Eigen::Matrix3d iniState, finState;
    auto            route = basic_traj->samplingPointArray(config.pieceN);
    iniState << route.front().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    finState << route.back().col(0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    gcopter::GCOPTER_PolytopeSFC gcopter;

    traj.clear();
    Eigen::VectorXd initTime;
    initTime.resize(config.pieceN);
    initTime.setConstant(2 * M_PI / config.basic_trajectory_rate /
                         config.pieceN);

    Eigen::Matrix3Xd initPointsPos(3, config.pieceN - 1);
    for (int i = 0; i < config.pieceN - 1; i++)
    {
      Eigen::Vector3d v = route[i].col(1);
      v.normalize();
      Eigen::Vector3d u = v.cross(Eigen::Vector3d::UnitZ());
      initPointsPos.col(i) =
          route[i + 1].col(0) + u * config.box_side_outside_length * 0.8;
    }

    // print initTime and init Pos
    std::cerr << "initTime: " << initTime.transpose() << std::endl;
    std::cerr << "initPointsPos: \n" << initPointsPos << std::endl;
    // print size of time and pos
    std::cerr << "initTime size: " << initTime.size() << std::endl;
    std::cerr << "initPointsPos size: " << initPointsPos.size() << std::endl;
    // print end point
    std::cerr << "finState: \n" << finState << std::endl;
    if (!init_setup)
    {
      if (!gcopter.setup_basic_trajectory(
              iniState, finState, initTime, initPointsPos, hPolys,
              config.smoothingEps, config.integralIntervs))
      {
        return;
      }
      auto opt_start = std::chrono::high_resolution_clock::now();

      if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
      {
        return;
      }
      auto opt_end = std::chrono::high_resolution_clock::now();

      std::cerr << "planning duration"
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       opt_end - opt_start)
                       .count()
                << "ms" << std::endl;

      init_setup = true;

      auto route_msg            = basic_traj->toMsg(100);
      route_msg.header.frame_id = config.frame;
      route_msg.header.stamp    = this->get_clock()->now();
      this->trajVizPub->publish(route_msg);

      auto &tailsitter_data = tailsitter_df::Data::getInstance();
      tailsitter_data.reset(config.trajecotry_cirled);

      // Initialize Newton-Raphson solver with initial alpha = 90 degrees at t=0
      constexpr double INITIAL_ALPHA_DEG = 90.0;
      g_alpha_solver.resetAlpha(INITIAL_ALPHA_DEG);

      control_traj_msg.points.clear();
      // Track df_lyu_quat to avoid quaternion sign flips between samples
      Eigen::Quaterniond prev_df_lyu_quat = Eigen::Quaterniond::Identity();
      bool                df_lyu_quat_initialized = false;
      auto &csv_file = init_planning_csv_file(config);
      RCLCPP_INFO(this->get_logger(), "CSV file opened");
      for (double t = 0.0; t <= traj.getTotalDuration() + DBL_EPSILON;
           t += config.traj_pub_rsl)
      {
        Eigen::Vector3d omg;
        double          thrust;

        tailsitter_data.df_backward_auto(traj.getVel(t), traj.getAcc(t),
                                         traj.getJer(t), thrust, omg);

        {
          // Get all required data from tailsitter model
          const auto &pos = traj.getPos(t);
          const auto &vel = traj.getVel(t);
          const auto &acc = traj.getAcc(t);
          const auto &yb  = tailsitter_data.getYb();

          // Calculate acceleration without gravity
          const Eigen::Vector3d gravity(0, 0, 9.81);
          const Eigen::Vector3d av = acc - gravity;

          // Get aerodynamic data
          const double df_phi_alpha  = tailsitter_data.getAlpha();  // Alpha from differential flat
          const double alpha_dot     = tailsitter_data.getAlphaDot();
          const double aero_force_z  = tailsitter_data.getBAeroForce()(2);
          const double B_aero_force_z = tailsitter_data.getBAeroForceZ();
          const double S_ap_x        = tailsitter_data.getSApX();
          const double S_ap_z        = tailsitter_data.getSApZ();
          const double B_Omega_b_y   = tailsitter_data.getBOmegaY();

          // Solve force balance equation for df_lyu_alpha:
          // S_ap_z*cos(alpha) + S_ap_x*sin(alpha) = F_aero_z/mass
          // This uses Newton-Raphson with temporal coherence
          const double V = vel.norm();
          const double df_lyu_alpha = g_alpha_solver.solve(V, S_ap_x, S_ap_z, config.mass);

          const auto quat_df = tailsitter_data.getQuaternion();

          // Compute df_lyu_quat using the helper function
          // Note: acc is raw acceleration, the helper function handles gravity subtraction internally
          Eigen::Quaterniond df_lyu_quat = compute_df_lyu_quat(vel, acc, df_lyu_alpha);

          // Enforce temporal continuity: flip sign if quaternion jumped by 180 degrees
          if (df_lyu_quat_initialized)
          {
            if (df_lyu_quat.coeffs().dot(prev_df_lyu_quat.coeffs()) < 0.0)
            {
              df_lyu_quat.coeffs() *= -1.0;
            }
          }
          else
          {
            df_lyu_quat_initialized = true;
          }
          prev_df_lyu_quat = df_lyu_quat;

          // Write data to CSV in consistent order
          csv_file << t << "," << pos(0) << "," << pos(1) << "," << pos(2)
                   << "," << vel(0) << "," << vel(1) << "," << vel(2) << ","
                   << thrust << "," << df_phi_alpha << "," << df_lyu_alpha << "," << aero_force_z << ","
                   << -yb(0) << "," << -yb(1) << "," << -yb(2) << ","
                   << quat_df.w() << "," << quat_df.x() << "," << quat_df.y() << "," << quat_df.z() << ","
                   << df_lyu_quat.w() << "," << df_lyu_quat.x() << "," << df_lyu_quat.y() << "," << df_lyu_quat.z() << ","
                   << av(0)
                   << "," << av(1) << "," << av(2) << "," << S_ap_x << ","
                   << S_ap_z << "," << B_Omega_b_y << "," << alpha_dot << ","
                   << B_aero_force_z << "\n";
        }

        auto p = traj.getPos(t);
        auto v = traj.getVel(t);

        auto            yb_df = tailsitter_data.getYb();
        // Eigen::Vector3d YB =
        //     (Eigen::Vector3d() << -yb_df(0), -yb_df(1), yb_df(2)).finished();
        // Eigen::Vector3d YB =
        //     (Eigen::Vector3d() << -yb_df(0), -yb_df(1), yb_df(2)).finished();
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
      for (int i = 0; i < config.pieceN; i++)
      {
        route_path.emplace_back(route[i].col(0));
      }
      if (traj.getPieceNum() > 0)
      {
        trajStamp = this->get_clock()->now().seconds();

        // Record routes to separate CSV file after successful optimization
        auto &routes_csv_file = init_routes_csv_file(config);
        for (size_t i = 0; i < route.size(); i++)
        {
          const auto &point = route[i];
          // Extract position, velocity, acceleration, and jerk
          const auto &pos = point.col(0);  // position (x, y, z)
          const auto &vel = point.col(1);  // velocity (vx, vy, vz)
          const auto &acc = point.col(2);  // acceleration (ax, ay, az)
          const auto &jer = point.col(3);  // jerk (jx, jy, jz)

          routes_csv_file << i << ","
                         << pos(0) << "," << pos(1) << "," << pos(2) << ","
                         << vel(0) << "," << vel(1) << "," << vel(2) << ","
                         << acc(0) << "," << acc(1) << "," << acc(2) << ","
                         << jer(0) << "," << jer(1) << "," << jer(2) << "\n";
        }
        routes_csv_file.close();
        RCLCPP_INFO(this->get_logger(), "Routes CSV file created");
      }
      csv_file.close();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto global_planner = std::make_shared<GlobalPlanner>();
  rclcpp::spin(global_planner);
  rclcpp::shutdown();

  return 0;
}

std::filesystem::path get_artifacts_temp_dir()
{
  if (const char *ap_pnc_dir = std::getenv("AP_PNC_DIR"))
  {
    return std::filesystem::path(ap_pnc_dir) / "artifacts" / "simulations" / "temp_data";
  }
  return std::filesystem::path("artifacts/simulations/temp_data");
}

std::ofstream &init_planning_csv_file(const Config &config)
{
  static std::ofstream csv_file;
  try
  {
    std::filesystem::path dir = get_artifacts_temp_dir();
    if (!std::filesystem::exists(dir))
    {
      std::filesystem::create_directories(dir);
    }

    // Create filename based on config parameters to match routes naming convention
    std::string filename = "planning_" +
                          get_trajectory_type_name(config.basic_trajectory_type) +
                          "_vel" + format_float_1f(config.cost_config.v_max) +
                          "_thrust" + format_float_1f(config.cost_config.thrust_min) +
                          "-" + format_float_1f(config.cost_config.thrust_max) +
                          ".csv";

    csv_file.open((dir / filename).string(), std::ios::out | std::ios::trunc);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to create/open planning CSV file: " << e.what()
              << std::endl;
    // try fallback to current dir
    csv_file.open("planning.csv", std::ios::out | std::ios::trunc);
  }

  if (!csv_file.is_open())
  {
    std::cerr << "Unable to open planning CSV file for writing." << std::endl;
    return csv_file;
  }

  // header t, p, v, thrust, df_phi_alpha, df_lyu_alpha, aero_force_z  with header name
  csv_file << "time,pos_x,pos_y,pos_z,"
           << "vel_x,vel_y,vel_z,"
           << "thrust,df_phi_alpha,df_lyu_alpha,aero_force_z,"
           << "yb_x,yb_y,yb_z,"
           << "df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,"
           << "df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,"
           << "ap_x,ap_y,ap_z,"
           << "S_ap_x,S_ap_z,B_Omega_b_y,"
           << "alpha_dot,B_aero_force_z\n";
  return csv_file;
}

std::string format_float_1f(double value)
{
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.1f", value);
  return std::string(buffer);
}

std::string get_trajectory_type_name(int type)
{
  switch(type)
  {
    case 1: return "lemniscate";
    case 2: return "circle";
    case 3: return "sin";
    case 4: return "line";
    default: return "unknown";
  }
}

std::ofstream &init_routes_csv_file(const Config &config)
{
  static std::ofstream routes_csv_file;
  try
  {
    std::filesystem::path dir = get_artifacts_temp_dir();
    if (!std::filesystem::exists(dir))
    {
      std::filesystem::create_directories(dir);
    }

    // Create filename based on config parameters as specified in task2.md
    std::string filename = "routes_" +
                          get_trajectory_type_name(config.basic_trajectory_type) +
                          "_vel" + format_float_1f(config.cost_config.v_max) +
                          "_thrust" + format_float_1f(config.cost_config.thrust_min) +
                          "-" + format_float_1f(config.cost_config.thrust_max) +
                          ".csv";

    routes_csv_file.open((dir / filename).string(), std::ios::out | std::ios::trunc);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to create/open routes CSV file: " << e.what() << std::endl;
    // try fallback to current dir
    routes_csv_file.open("routes.csv", std::ios::out | std::ios::trunc);
  }

  if (!routes_csv_file.is_open())
  {
    std::cerr << "Unable to open routes CSV file for writing." << std::endl;
    return routes_csv_file;
  }

  // Write header
  routes_csv_file << "index,"
                  << "pos_x,pos_y,pos_z,"
                  << "vel_x,vel_y,vel_z,"
                  << "acc_x,acc_y,acc_z,"
                  << "jerk_x,jerk_y,jerk_z\n";
  return routes_csv_file;
}
