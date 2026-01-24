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

#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <string>

#include <gcopter/trajectory.hpp>
#include <aerodynamics/aero_alpha_solver.hpp>
#include <tailsitter_df/tailsitter_df.hpp>

#include <trajectory_server/planning_config.hpp>

namespace trajectory_server
{

/**
 * @brief Trajectory sample data for CSV logging
 *
 * Contains all relevant data from a single trajectory sample point.
 * Used for logging planning results to CSV files.
 */
struct TrajectorySampleData
{
  // Time and kinematics
  double t;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;

  // Forces and angles
  double B_ap_x_df_lyu;  //update using  S_ap_x cos alpha - S_ap_z sin alpha upon lyu alpha.   
  double df_phi_alpha;   ///< Alpha from differential flat
  double df_lyu_alpha;   ///< Alpha from Lyu aerodynamics solver

  // Body frame Y-axis (negated for FLU frame)
  Eigen::Vector3d yb;

  // Quaternions
  Eigen::Quaterniond quat_df;    ///< Quaternion from differential flat
  Eigen::Quaterniond quat_lyu;   ///< Quaternion from Lyu aerodynamics

  // Acceleration without gravity (Wap = a - g)
  Eigen::Vector3d W_ap;

  // Aerodynamic specific force components
  double S_ap_x;
  double S_ap_z;

  // Spatial frame angular velocities
  double S_Omega_x;
  double S_Omega_y;
  double S_Omega_z;

  double B_Omega_x_df_lyu; // S_Omega_x cos alpha - S_Omega_z sin alpha; upon lyu_alpha
  double B_Omega_y_df_lyu; // set to zero.
  double B_Omega_z_df_lyu; // S_Omega_x sin alpha - S_Omega_z cos alpha ; upon lyu_alpha

  double B_fa_z_df_lyu;
  // Body frame angular velocity and rate
  double alpha_dot;

  /**
   * @brief Factory method: Create sample from trajectory and tailsitter data
   *
   * Extracts all required data from the trajectory, tailsitter model,
   * and aerodynamics solver at time t.
   *
   * @param traj Trajectory object
   * @param t Time to sample
   * @param tailsitter_data Tailsitter data model
   * @param mass Vehicle mass (kg)
   * @param alpha_solver Aerodynamic alpha solver
   * @param prev_df_lyu_quat Previous quaternion for continuity (optional)
   * @return Populated TrajectorySampleData
   */
  static TrajectorySampleData from_trajectory(
      const Trajectory<5>& traj,
      double t,
      tailsitter_df::Data& tailsitter_data,
      double mass,
      aerodynamics::AeroAlphaSolver& alpha_solver,
      const Eigen::Quaterniond& prev_df_lyu_quat = Eigen::Quaterniond::Identity(),
      bool df_lyu_quat_initialized = false);
};

/**
 * @brief Route sample data for CSV logging
 *
 * Contains kinematic data from a single route waypoint (position, velocity,
 * acceleration, jerk). Used for logging optimized routes to CSV files.
 */
struct RouteSampleData
{
  size_t index;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Vector3d jerk;

  /**
   * @brief Factory method: Create route sample from trajectory matrix point
   *
   * Extracts position, velocity, acceleration, and jerk from a trajectory point.
   * The point is a 4x3 matrix where each column contains [pos, vel, acc, jerk].
   *
   * @param point Trajectory matrix point (4x3: pos, vel, acc, jerk)
   * @param i Index of this waypoint in the route
   * @return Populated RouteSampleData
   */
  static RouteSampleData from_trajectory_point(const Eigen::MatrixXd& point, size_t i);
};

/**
 * @brief Write trajectory sample data to CSV file
 *
 * @param csv_file Output file stream
 * @param data Trajectory sample data to write
 */
void write_trajectory_sample(std::ofstream& csv_file, const TrajectorySampleData& data);

/**
 * @brief Write route sample data to CSV file
 *
 * @param csv_file Output file stream
 * @param data Route sample data to write
 */
void write_route_sample(std::ofstream& csv_file, const RouteSampleData& data);

/**
 * @brief Get the artifacts temporary directory path
 *
 * Resolves the directory for storing temporary simulation data.
 * Uses AP_PNC_DIR environment variable if set, otherwise uses default path.
 *
 * @return Path to artifacts/simulations/temp_data directory
 */
std::filesystem::path get_artifacts_temp_dir();

/**
 * @brief Format a float value with 1 decimal place
 *
 * @param value Float value to format
 * @return Formatted string (e.g., "10.0")
 */
std::string format_float_1f(double value);

/**
 * @brief Get trajectory type name from integer type code
 *
 * @param type Integer type code (1=lemniscate, 2=circle, 3=sin, 4=line)
 * @return String name of trajectory type
 */
std::string get_trajectory_type_name(int type);

/**
 * @brief Initialize and open planning CSV file for writing
 *
 * Creates a new CSV file with planning data including position, velocity,
 * thrust, alpha angles, quaternions, and aerodynamic forces.
 *
 * Filename format: planning_{type}_vel{v_max}_thrust{min}-{max}.csv
 *
 * @param config Configuration object for filename generation
 * @return Reference to opened CSV file stream
 */
std::ofstream& init_planning_csv_file(const Config& config);

/**
 * @brief Initialize and open routes CSV file for writing
 *
 * Creates a new CSV file with route waypoints including position,
 * velocity, acceleration, and jerk data.
 *
 * Filename format: routes_{type}_vel{v_max}_thrust{min}-{max}.csv
 *
 * @param config Configuration object for filename generation
 * @return Reference to opened CSV file stream
 */
std::ofstream& init_routes_csv_file(const Config& config);

}  // namespace trajectory_server
