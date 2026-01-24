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

#include <trajectory_server/planning_utils.hpp>

#include <trajectory_server/attitude_computer.hpp>

#include <cstdio>
#include <iostream>

namespace trajectory_server
{

TrajectorySampleData TrajectorySampleData::from_trajectory(
    const Trajectory<5>& traj,
    double t,
    tailsitter_df::Data& tailsitter_data,
    double mass,
    aerodynamics::AeroAlphaSolver& alpha_solver,
    const Eigen::Quaterniond& prev_df_lyu_quat,
    bool df_lyu_quat_initialized)
{
  TrajectorySampleData data;

  // Get all required data from tailsitter model
  data.t   = t;
  data.pos = traj.getPos(t);
  data.vel = traj.getVel(t);

  Eigen::Vector3d omg;
  tailsitter_data.df_backward_auto(traj.getVel(t), traj.getAcc(t),
                                   traj.getJer(t), data.thrust, omg);

  const auto& yb  = tailsitter_data.getYb();
  const auto& acc = traj.getAcc(t);

  // Calculate acceleration without gravity
  const Eigen::Vector3d gravity(0, 0, 9.81);
  data.Wap = acc - gravity;

  // Get aerodynamic data
  data.df_phi_alpha = tailsitter_data.getAlpha();  // Alpha from differential flat
  data.alpha_dot    = tailsitter_data.getAlphaDot();
  data.aero_force_z = tailsitter_data.getBAeroForce()(2);
  data.B_aero_force_z = tailsitter_data.getBAeroForceZ();
  data.S_ap_x        = tailsitter_data.getSApX();
  data.S_ap_z        = tailsitter_data.getSApZ();
  data.B_Omega_b_y   = tailsitter_data.getBOmegaY();

  // Solve force balance equation for df_lyu_alpha:
  // S_ap_z*cos(alpha) + S_ap_x*sin(alpha) = F_aero_z/mass
  const double V          = data.vel.norm();
  data.df_lyu_alpha       = alpha_solver.solve(V, data.S_ap_x, data.S_ap_z, mass);

  data.quat_df = tailsitter_data.getQuaternion();

  // Compute df_lyu_quat using the helper function
  data.quat_lyu = compute_df_lyu_quat(data.vel, acc, data.df_lyu_alpha);

  // Enforce temporal continuity: flip sign if quaternion jumped by 180 degrees
  if (df_lyu_quat_initialized)
  {
    if (data.quat_lyu.coeffs().dot(prev_df_lyu_quat.coeffs()) < 0.0)
    {
      data.quat_lyu.coeffs() *= -1.0;
    }
  }

  // Negate yb for FLU frame
  data.yb = -yb;

  return data;
}


std::filesystem::path get_artifacts_temp_dir()
{
  if (const char* ap_pnc_dir = std::getenv("AP_PNC_DIR"))
  {
    return std::filesystem::path(ap_pnc_dir) / "artifacts" / "simulations" /
           "temp_data";
  }
  return std::filesystem::path("artifacts/simulations/temp_data");
}

std::string format_float_1f(double value)
{
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.1f", value);
  return std::string(buffer);
}

std::string get_trajectory_type_name(int type)
{
  switch (type)
  {
    case 1:
      return "lemniscate";
    case 2:
      return "circle";
    case 3:
      return "sin";
    case 4:
      return "line";
    default:
      return "unknown";
  }
}

std::ofstream& init_planning_csv_file(const Config& config)
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
  catch (const std::exception& e)
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

  // header t, p, v, thrust, df_phi_alpha, df_lyu_alpha, aero_force_z  with header
  // name
  csv_file << "time,pos_x,pos_y,pos_z,"
           << "vel_x,vel_y,vel_z,"
           << "thrust,df_phi_alpha,df_lyu_alpha,aero_force_z,"
           << "yb_x,yb_y,yb_z,"
           << "df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,"
           << "df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,"
           << "Wap_x,Wap_y,Wap_z,"
           << "S_ap_x,S_ap_z,B_Omega_b_y,"
           << "alpha_dot,B_aero_force_z\n";
  return csv_file;
}

std::ofstream& init_routes_csv_file(const Config& config)
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
  catch (const std::exception& e)
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

void write_trajectory_sample(std::ofstream& csv_file, const TrajectorySampleData& data)
{
  csv_file << data.t << ","
           << data.pos(0) << "," << data.pos(1) << "," << data.pos(2) << ","
           << data.vel(0) << "," << data.vel(1) << "," << data.vel(2) << ","
           << data.thrust << ","
           << data.df_phi_alpha << "," << data.df_lyu_alpha << ","
           << data.aero_force_z << ","
           << data.yb(0) << "," << data.yb(1) << "," << data.yb(2) << ","
           << data.quat_df.w() << "," << data.quat_df.x() << ","
           << data.quat_df.y() << "," << data.quat_df.z() << ","
           << data.quat_lyu.w() << "," << data.quat_lyu.x() << ","
           << data.quat_lyu.y() << "," << data.quat_lyu.z() << ","
           << data.Wap(0) << "," << data.Wap(1) << "," << data.Wap(2) << ","
           << data.S_ap_x << "," << data.S_ap_z << ","
           << data.B_Omega_b_y << ","
           << data.alpha_dot << "," << data.B_aero_force_z << "\n";
}

RouteSampleData RouteSampleData::from_trajectory_point(const Eigen::MatrixXd& point, size_t i)
{
  RouteSampleData data;
  data.index = i;
  data.pos  = point.col(0);
  data.vel  = point.col(1);
  data.acc  = point.col(2);
  data.jerk = point.col(3);
  return data;
}

void write_route_sample(std::ofstream& csv_file, const RouteSampleData& data)
{
  csv_file << data.index << ","
           << data.pos(0) << "," << data.pos(1) << "," << data.pos(2) << ","
           << data.vel(0) << "," << data.vel(1) << "," << data.vel(2) << ","
           << data.acc(0) << "," << data.acc(1) << "," << data.acc(2) << ","
           << data.jerk(0) << "," << data.jerk(1) << "," << data.jerk(2) << "\n";
}

}  // namespace trajectory_server
