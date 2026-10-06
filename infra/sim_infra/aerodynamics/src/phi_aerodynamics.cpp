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

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <aerodynamics/phi_aerodynamics.hpp>
#include <aerodynamics/project_paths.hpp>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <stdexcept>

namespace aerodynamics
{

  bool PhiAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                      Eigen::Vector3d&       aero_force_b,
                                      Eigen::Vector3d&       aero_moment_b,
                                      double& alpha, double& beta)
  {
    // Calculate alpha and beta from body velocity
    constexpr double vThreshold = 5e-1;
    if (body_speed.norm() < vThreshold)
    {
      alpha = 0.0;
      beta  = 0.0;
    }
    else
    {
      alpha = std::atan2(body_speed(2), body_speed(0));
      beta  = std::asin(body_speed(1) / body_speed.norm());
    }

    // Calculate aerodynamic forces using phi matrix
    aero_force_b  = _phi_matrix * body_speed * body_speed.norm();
    aero_moment_b = Eigen::Vector3d::Zero();
    return true;
  }

  bool PhiAerodynamicsCreator::createAerodynamics()
  {
    // Load the phi matrix from config/aero/phi.yaml (3x3, row-major)
    Eigen::Matrix3d phi_matrix = Eigen::Matrix3d::Zero();
    try
    {
      const YAML::Node config =
          YAML::LoadFile(paths::aero_config("phi").string());
      const YAML::Node rows    = config["phi_coefs"];
      if (!rows || !rows.IsSequence() || rows.size() != 3)
        throw std::runtime_error("phi_coefs must be a 3x3 matrix");
      for (std::size_t r = 0; r < 3; ++r)
      {
        const YAML::Node row = rows[r];
        if (!row.IsSequence() || row.size() != 3)
          throw std::runtime_error("phi_coefs must be a 3x3 matrix");
        for (std::size_t c = 0; c < 3; ++c)
          phi_matrix(r, c) = row[c].as<double>();
      }
    }
    catch (const std::exception &e)
    {
      spdlog::error("phi aerodynamics: failed to load {}: {}",
                    paths::aero_config("phi").string(), e.what());
      return false;
    }

    std::stringstream ss;
    ss << phi_matrix;
    spdlog::debug("phi_matrix:\n{}", ss.str());

    // Create and initialize PhiAerodynamics instance
    auto instance = std::make_unique<PhiAerodynamics>();
    instance->setPhi(phi_matrix);

    _instance = std::move(instance);
    return true;
  }

}  // namespace aerodynamics
