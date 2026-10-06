/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Description:
 * Generate CSV data for ML training from all aerodynamic types.
 * Outputs: lyu_across_AoAs.csv, bspline_across_AoAs.csv, phi_across_AoAs.csv,
 * advanced_across_AoAs.csv Format: alpha(rad),cx,cz
 */

#include <spdlog/spdlog.h>

#include <aerodynamics/advanced_lift_drag.hpp>
#include <aerodynamics/bspline_aerodynamics.hpp>
#include <aerodynamics/phi_aerodynamics.hpp>
#include <aerodynamics/project_paths.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>
#include <fstream>
#include <iomanip>

using namespace aerodynamics;

/**
 * @brief Generate CSV data for a specific aerodynamics implementation
 *
 * @param creator Factory to create the aerodynamics implementation
 * @param filename Output CSV filename (e.g., "lyu_across_AoAs.csv")
 * @param name Name of the implementation for debug output
 * @return true if generation succeeded
 */
bool generate_aero_data(std::unique_ptr<AerodynamicsCreator> creator,
                        const std::string& filename, const std::string& name)
{
  spdlog::info("Generating data for {}", name);

  if (!creator->createAerodynamics())
  {
    spdlog::error("Failed to create {}", name);
    return false;
  }

  // Airspeed magnitude
  constexpr double speed_magnitude = 12.0;

  // High resolution for smooth ML training: 3601 points (~0.1 degree steps)
  constexpr int num_points = 3601;

  // Alpha range: -pi to +pi radians
  constexpr double alpha_min  = -M_PI;
  constexpr double alpha_max  = M_PI;
  constexpr double alpha_step = (alpha_max - alpha_min) / (num_points - 1);

  // Create output file
  const auto    output_path = paths::output(filename);
  std::ofstream outfile(output_path);
  if (!outfile.is_open())
  {
    spdlog::error("Failed to open file: {}", output_path.string());
    return false;
  }

  // CSV header: alpha in radians, cx, cz
  outfile << "alpha(rad),cx,cz\n";

  try
  {
    for (int i = 0; i < num_points; ++i)
    {
      // Input alpha for velocity calculation
      const double alpha_input = alpha_min + i * alpha_step;

      // Compute body velocity components based on alpha_input
      // For positive alpha (pitch up), velocity comes from below
      Eigen::Vector3d body_speed;
      body_speed.x() = speed_magnitude * std::cos(alpha_input);
      body_speed.y() = 0.0;  // No sideslip
      body_speed.z() = speed_magnitude * std::sin(alpha_input);

      Eigen::Vector3d aero_force_b;
      Eigen::Vector3d aero_moment_b;

      // Output parameters (computed by aerodynamics model)
      double alpha_out = 0.0;
      double beta_out  = 0.0;

      creator->getAeroWrench(body_speed, aero_force_b, aero_moment_b, alpha_out,
                             beta_out);

      // Normalize coefficients: force / (speed^2)
      const double cx = aero_force_b.x() / (speed_magnitude * speed_magnitude);
      const double cz = aero_force_b.z() / (speed_magnitude * speed_magnitude);

      // Write: alpha_input(rad), cx, cz
      outfile << std::setprecision(std::numeric_limits<double>::max_digits10)
              << alpha_input << "," << cx << "," << cz << "\n";
    }

    outfile.close();
    spdlog::info("Data written to {} ({} points)", output_path.string(),
                 num_points);
    return true;
  }
  catch (const std::exception& e)
  {
    spdlog::error("Exception in {}: {}", name, e.what());
    outfile.close();
    return false;
  }
}

int main()
{
  spdlog::set_level(spdlog::level::info);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

  spdlog::info("=== Aerodynamics ML Data Generation ===");
  spdlog::info("Alpha range: [-pi, +pi] radians");
  spdlog::info("Resolution: 3601 points (~0.1 degree steps)");

  bool all_passed = true;

  // Generate data for each aerodynamic type
  all_passed &= generate_aero_data(std::make_unique<LyuAerodynamicsCreator>(),
                                   "lyu_across_AoAs.csv", "LyuAerodynamics");

  all_passed &=
      generate_aero_data(std::make_unique<BsplineAerodynamicsCreator>(),
                         "bspline_across_AoAs.csv", "BsplineAerodynamics");

  all_passed &= generate_aero_data(std::make_unique<PhiAerodynamicsCreator>(),
                                   "phi_across_AoAs.csv", "PhiAerodynamics");

  all_passed &=
      generate_aero_data(std::make_unique<AdvancedLiftDragCreator>(),
                         "advanced_across_AoAs.csv", "AdvancedLiftDrag");

  if (all_passed)
  {
    spdlog::info("=== All aerodynamic data generated successfully! ===");
    return 0;
  }
  else
  {
    spdlog::error("=== Some data generation failed! ===");
    return 1;
  }
}
