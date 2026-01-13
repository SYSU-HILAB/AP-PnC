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
 * @brief Generate aerodynamic lookup table CSV file using LyuAerodynamics
 *
 * This utility generates a fine-grained alpha->cz lookup table by calling
 * the MATLAB-generated Aerodynamics function over a range of angle of attack values.
 *
 * Usage:
 *   ./generate_aero_lookup_table <output_csv_file> [velocity]
 *
 * Arguments:
 *   output_csv_file - Path to output CSV file (default: aero_lookup_table.csv)
 *   velocity        - Flight velocity in m/s (default: 10.0)
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "Zhang/Aerodynamics.h"

// Convert degrees to radians
constexpr double DEG_TO_RAD = M_PI / 180.0;

int main(int argc, char* argv[])
{
  // Parse command line arguments
  std::string output_file = "aero_lookup_table.csv";
  double      velocity    = 10.0;  // Default velocity in m/s

  if (argc > 1)
  {
    output_file = argv[1];
  }
  if (argc > 2)
  {
    velocity = std::stod(argv[2]);
  }

  std::cout << "Generating aerodynamic lookup table..." << std::endl;
  std::cout << "Output file: " << output_file << std::endl;
  std::cout << "Velocity: " << velocity << " m/s" << std::endl;

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

  // Define alpha range: -180 to +180 degrees with 1 degree granularity
  // For a tailsitter, typical alpha range is approximately -180 to +180 degrees
  // due to the ability to fly in both forward and backward directions.
  constexpr double alpha_min = -180.0;
  constexpr double alpha_max = 180.0;
  constexpr double alpha_step = 1.0;  // 1 degree granularity

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
    double V_b[3] = {velocity * std::cos(alpha_rad), 0.0,
                     velocity * std::sin(alpha_rad)};

    // Call MATLAB-generated Aerodynamics function
    double F_b[3];     // Aerodynamic force in body frame (N)
    double M_b[3];     // Aerodynamic moment in body frame (N·m)
    double alpha_out;  // Angle of attack from model (rad)
    double beta_out;   // Sideslip angle from model (rad)

    Aerodynamics(V_b, F_b, M_b, &alpha_out, &beta_out);

    // Extract force components
    double fx = F_b[0];  // Drag force (negative x direction)
    double fy = F_b[1];  // Side force
    double fz = F_b[2];  // Lift force (negative z direction in FRD)

    // Compute dynamic pressure: q = 0.5 * rho * V^2
    // Using standard air density rho = 1.225 kg/m^3
    constexpr double rho    = 1.225;
    double              q      = 0.5 * rho * velocity * velocity;

    // Assume reference area S_ref = 1.0 for coefficient calculation
    // (adjust as needed for your aircraft)
    constexpr double S_ref = 0.1;  // Reference area in m^2

    // Compute force coefficients
    double cx = -fx / (q * S_ref);  // Drag coefficient (positive for drag)
    double cz = -fz / (q * S_ref);  // Lift coefficient (positive for lift up)

    // Write to CSV
    csv << alpha_deg << "," << cx << "," << cz << "\n";

    // Progress indicator
    if (static_cast<int>(alpha_deg) % 30 == 0)
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
