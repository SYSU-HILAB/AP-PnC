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

#include <spdlog/spdlog.h>

#include <ScopeProfiler/ScopeProfiler.hpp>
#include <aerodynamics/advanced_lift_drag.hpp>
#include <aerodynamics/bspline_aerodynamics.hpp>
#include <aerodynamics/non_aerodynamics.hpp>
#include <aerodynamics/phi_aerodynamics.hpp>
#include <aerodynamics/project_paths.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>
#include <fstream>
using namespace aerodynamics;

/**
 * @brief Test a specific aerodynamics implementation
 *
 * @param creator Factory to create the aerodynamics implementation
 * @param name Name of the implementation for debug output
 * @return true if test passed
 */
bool test_aerodynamics(std::unique_ptr<AerodynamicsCreator> creator,
                       const std::string&                   name)
{
  spdlog::debug("\nTesting {}", name);

  if (!creator->createAerodynamics())
  {
    spdlog::error("Failed to create {}", name);
    return false;
  }

  // Set constant airspeed magnitude of 12
  constexpr double speed_magnitude = 12.0;
  constexpr int    num_points      = 361;
  constexpr double deg_to_rad      = M_PI / 180.0;

  // Create output file for this aerodynamics model
  const auto    output_path = paths::output("z" + name + ".csv");
  std::ofstream outfile(output_path);
  if (!outfile)
    throw std::runtime_error("cannot write " + output_path.string());
  outfile << "alpha_deg,cx,cz\n";  // CSV header

  try
  {
    for (int i = 0; i < num_points; ++i)
    {
      // Sweep alpha from -180 to 180 degrees, 1 degree increments
      double alpha_deg = -180.0 + i;
      double alpha     = alpha_deg * deg_to_rad;
      double beta      = 0.0;  // Zero sideslip angle

      // Compute body velocity components based on alpha
      Eigen::Vector3d body_speed;
      body_speed.x() = speed_magnitude * cos(alpha);
      body_speed.y() = 0.0;  // No sideslip
      body_speed.z() = speed_magnitude * sin(alpha);

      Eigen::Vector3d aero_force_b;
      Eigen::Vector3d aero_moment_b;

      creator->getAeroWrench(body_speed, aero_force_b, aero_moment_b, alpha,
                             beta);

      // Record angle of attack and force components
      outfile << alpha_deg << ","
              << aero_force_b.x() / (speed_magnitude * speed_magnitude) << ","
              << aero_force_b.z() / (speed_magnitude * speed_magnitude) << "\n";
    }

    outfile.close();
    spdlog::info("Data written to {}", output_path.string());
    return true;
  }
  catch (const std::exception& e)
  {
    spdlog::error("Exception in {}: {}", name, e.what());
    return false;
  }
}

int main()
{
  // Initialize spdlog with debug level and console sink
  spdlog::set_level(spdlog::level::err);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  bool all_passed = true;

  {
    ScopeProfiler _("PhiAero");

    all_passed &= test_aerodynamics(std::make_unique<PhiAerodynamicsCreator>(),
                                    "PhiAero");
  }
  {
    ScopeProfiler _("BSplineAero");
    all_passed &= test_aerodynamics(
        std::make_unique<BsplineAerodynamicsCreator>(), "BSplineAero");
  }
  {
    ScopeProfiler _("AdvancedLDAero");
    all_passed &= test_aerodynamics(std::make_unique<AdvancedLiftDragCreator>(),
                                    "AdvancedLDAero");
  }
  {
    ScopeProfiler _("ZhangLyuAero");
    all_passed &= test_aerodynamics(std::make_unique<LyuAerodynamicsCreator>(),
                                    "ZhangLyuAero");
  }
  {
    ScopeProfiler _("NonAero");
    all_passed &= test_aerodynamics(std::make_unique<NonAerodynamicsCreator>(),
                                    "NonAero");
  }
  printScopeProfiler();
  // Test Phi Aerodynamics

  if (all_passed)
  {
    spdlog::info("\nAll tests passed!");
    return 0;
  }
  else
  {
    spdlog::error("\nSome tests failed!");
    return 1;
  }
}
