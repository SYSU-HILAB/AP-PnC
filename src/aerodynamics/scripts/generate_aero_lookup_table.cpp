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

/**
 * @file generate_aero_lookup_table.cpp
 * @brief Generate aerodynamic lookup table CSV file using LyuAerodynamics C++ wrapper
 *
 * This utility generates a fine-grained alpha->cz lookup table by calling
 * the LyuAerodynamics C++ wrapper over a range of angle of attack values.
 *
 * Usage:
 *   ./generate_aero_lookup_table <output_csv_file> [velocity] [mass]
 *
 * Arguments:
 *   output_csv_file - Path to output CSV file (default: aero_lookup_table.csv)
 *   velocity        - Flight velocity in m/s (default: 10.0)
 *   mass            - Vehicle mass in kg (default: 2.0)
 */

#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

// Convert degrees to radians
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr double DEG_TO_RAD = M_PI / 180.0;

int main(int argc, char* argv[])
{
  // Parse command line arguments
  std::string output_file = "aero_lookup_table.csv";
  double      velocity    = 10.0;  // Default velocity in m/s
  double      mass        = 2.0;   // Default mass in kg

  if (argc > 1)
  {
    output_file = argv[1];
  }
  if (argc > 2)
  {
    velocity = std::stod(argv[2]);
  }
  if (argc > 3)
  {
    mass = std::stod(argv[3]);
  }

  std::cout << "Generating aerodynamic lookup table..." << std::endl;
  std::cout << "Output file: " << output_file << std::endl;
  std::cout << "Velocity: " << velocity << " m/s" << std::endl;
  std::cout << "Mass: " << mass << " kg" << std::endl;

  // Open output file
  std::ofstream csv(output_file);
  if (!csv.is_open())
  {
    std::cerr << "Error: Unable to open output file: " << output_file
              << std::endl;
    return 1;
  }

  // Write CSV header
  csv << "alpha_deg,cx,cz\n";
  csv << std::fixed << std::setprecision(6);

  // Create LyuAerodynamics instance
  aerodynamics::LyuAerodynamics aero;

  // Define alpha range: -180 to +180 degrees with 0.1 degree granularity
  // For a tailsitter, typical alpha range is approximately -180 to +180 degrees
  // due to the ability to fly in both forward and backward directions.
  constexpr double alpha_min = -180.0;
  constexpr double alpha_max = 180.0;
  constexpr double alpha_step = 0.1;  // 0.1 degree granularity

  // Generate lookup table
  for (double alpha_deg = alpha_min; alpha_deg <= alpha_max; alpha_deg += alpha_step)
  {
    // Convert alpha to radians
    double alpha_rad = alpha_deg * DEG_TO_RAD;

    // Construct body velocity vector for this alpha
    // In FRD convention (Forward-Right-Down):
    // - V_b[0] = forward velocity component = V * cos(alpha)
    // - V_b[1] = right velocity component = 0 (no sideslip)
    // - V_b[2] = down velocity component = V * sin(alpha)
    Eigen::Vector3d V_b(velocity * std::cos(alpha_rad), 0.0,
                        velocity * std::sin(alpha_rad));

    // Call LyuAerodynamics C++ wrapper
    Eigen::Vector3d aero_force_b;
    Eigen::Vector3d aero_moment_b;
    double          alpha_out;
    double          beta_out;

    bool success = aero.getAeroWrench(V_b, aero_force_b, aero_moment_b, alpha_out,
                                      beta_out);

    if (!success)
    {
      std::cerr << "Error: Aerodynamics computation failed for alpha = "
                << alpha_deg << " deg" << std::endl;
      return 1;
    }

    // Extract force components
    double fx = aero_force_b(0);  // Drag force (negative x direction)
    double fz = aero_force_b(2);  // Lift force (negative z direction in FRD)

    // Compute force coefficients
    // cz = -fz / (mass * V^2), where mass normalizes the force to acceleration
    double cx = -fx / (mass * velocity * velocity);  // Drag coefficient
    double cz = -fz / (mass * velocity * velocity);  // Lift coefficient

    // Write to CSV
    csv << alpha_deg << "," << cx << "," << cz << "\n";

    // Progress indicator (every 30 degrees = 300 points at 0.1 degree step)
    if (static_cast<int>(alpha_deg * 10) % 300 == 0)
    {
      std::cout << "  alpha = " << std::setw(6) << alpha_deg
                << " deg: cx = " << std::setw(8) << cx << ", cz = " << std::setw(8)
                << cz << std::endl;
    }
  }

  csv.close();

  std::cout << "Lookup table generation complete!" << std::endl;
  std::cout << "Output saved to: " << output_file << std::endl;

  return 0;
}
