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
#include <yaml-cpp/yaml.h>

#include <aerodynamics/bspline_aerodynamics.hpp>
#include <aerodynamics/project_paths.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace aerodynamics
{
  BsplineAerodynamics::BsplineAerodynamics(const std::vector<double> &cx_coefs,
                                           const std::vector<double> &cx_knots,
                                           const std::vector<double> &cz_coefs,
                                           const std::vector<double> &cz_knots,
                                           const double scale_factor)
      : cx_knots_(cx_knots),
        cz_knots_(cz_knots),
        cx_coefficients_(cx_coefs),
        cz_coefficients_(cz_coefs),
        scale_factor_(scale_factor)
  {
  }

  bool BsplineAerodynamics::getAeroWrench(const Eigen::Vector3d &body_speed,
                                          Eigen::Vector3d       &aero_force_b,
                                          Eigen::Vector3d       &aero_moment_b,
                                          double &alpha, double &beta)
  {
    std::tie(alpha, beta) = air_angles(body_speed);
    // Preserve the fitted model's low-speed force law without changing the
    // reported kinematic angles. Coefficient axes are body FLU.
    const bool                low_speed = body_speed.norm() < 0.5;
    const double              fit_alpha = low_speed ? 0.0 : alpha;
    const double              fit_beta  = low_speed ? 0.0 : beta;
    const std::vector<double> cx_basis =
        bspline_basis(SPLINE_ORDER, cx_knots_, fit_alpha);
    const std::vector<double> cz_basis =
        bspline_basis(SPLINE_ORDER, cz_knots_, fit_alpha);

    double v_squared = body_speed.squaredNorm();

    double           aeroForceBx = 0.0;
    constexpr double aeroForceBy = 0.0;
    double           aeroForceBz = 0.0;

    for (size_t i = 0; i < cx_basis.size() && i < cx_coefficients_.size(); ++i)
    {
      aeroForceBx += cx_coefficients_[i] * cx_basis[i];
    }
    for (size_t i = 0; i < cz_basis.size() && i < cz_coefficients_.size(); ++i)
    {
      aeroForceBz += cz_coefficients_[i] * cz_basis[i];
    }

    aero_force_b  = scale_factor_ *
                    Eigen::Vector3d(aeroForceBx, aeroForceBy, aeroForceBz) *
                    v_squared * cos(fit_beta);
    aero_moment_b = Eigen::Vector3d(0.0, 0.0, 0.0);

    return true;
  }

  bool BsplineAerodynamicsCreator::createAerodynamics()
  {
    // Load the fitted coefficients from config/aero/ma.yaml
    std::vector<double> cx_coefs, cx_knots, cz_coefs, cz_knots;
    double              scale_factor = 0.0;
    try
    {
      const YAML::Node config =
          YAML::LoadFile(paths::aero_config("ma").string());
      if (config["body_frame"].as<std::string>() != "FLU")
        throw std::runtime_error("ma parameters require body_frame: FLU");
      const auto read_vector =
          [&config](const char *key, std::vector<double> &out)
      {
        const YAML::Node node = config[key];
        if (!node || !node.IsSequence() || node.size() == 0)
          throw std::runtime_error(std::string("ma.yaml: '") + key +
                                   "' must be a non-empty list");
        out.clear();
        out.reserve(node.size());
        for (const auto &item : node)
          out.push_back(item.as<double>());
      };
      read_vector("cx_coefs", cx_coefs);
      read_vector("cx_knots", cx_knots);
      read_vector("cz_coefs", cz_coefs);
      read_vector("cz_knots", cz_knots);
      if (!config["scale_factor"])
        throw std::runtime_error("ma.yaml: 'scale_factor' missing");
      scale_factor = config["scale_factor"].as<double>();
    }
    catch (const std::exception &e)
    {
      spdlog::error("ma aerodynamics: failed to load {}: {}",
                    paths::aero_config("ma").string(), e.what());
      return false;
    }

    spdlog::debug("Loaded bspline parameters ({} cx coefs, {} cz coefs)",
                  cx_coefs.size(), cz_coefs.size());

    // Create and initialize BsplineAerodynamics instance
    auto instance = std::make_unique<BsplineAerodynamics>(
        cx_coefs, cx_knots, cz_coefs, cz_knots, scale_factor);

    _instance = std::move(instance);
    return true;
  }

};  // namespace aerodynamics
