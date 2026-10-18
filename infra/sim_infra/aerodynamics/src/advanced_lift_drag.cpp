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

// The original code is from
// https://github.com/gazebosim/gz-sim/blob/gz-sim9/src/systems/advanced_lift_drag/AdvancedLiftDrag.cc
// But we rewrite in the project primarily for
// 1. fair comparison with other aerodynamics (remove aerodynamic moment and
// aerodynamic force caused by angular velocity)
// 2. consistent API for more advanced application
// 3. consise invocation in our application
// 4. simplify the to the minimal parameters for the aerodynamics invocatoin
// The original parameters is from
// https://github.com/PX4/PX4-gazebo-models/blob/230450cc817dd7675612ed5ec72ee59b6989d367/models/quadtailsitter/model.sdf
// Authors:
// Erchao Rong: rongerch@outlook.com
// Zihao Liu: liuzh297@gmail.com
// Junning Liang: gordonliang27@foxmail.com
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <aerodynamics/advanced_lift_drag.hpp>
#include <aerodynamics/project_paths.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace aerodynamics
{
  namespace
  {
    // Struct to encapsulate all aerodynamic parameters
    struct AeroParams
    {
      static constexpr double CD_fp_k1 = -0.224;
      static constexpr double CD_fp_k2 = -0.115;
      // Blending parameters
      double sigmoid_blend;  // Sigmoid blending parameter
      // Basic aerodynamic coefficients
      double CL_alpha_0;
      double CL_alpha;     // Lift curve slope (per radian)
      double alpha_stall;  // Stall angle (radians)

      // Add wing efficiency (Oswald efficiency factor for a 3D wing)
      // this parameter incorporates Aspect ratio and PI.
      double eff;

      double CD_0;           // Zero-lift drag coefficient
      double CD_flat_plate;  // Flat plate model parameters

      // Side force parameters
      double CY_beta;       // Side force derivative
      double CL_beta_loss;  // Lift loss factor with sideslip
      double scale_factor;  // Scale factor for the aerodynamics
    };

    // Function to load parameters from config/aero/advanced.yaml
    bool loadAeroParams(AeroParams &params)
    {
      try
      {
        const YAML::Node config =
            YAML::LoadFile(paths::aero_config("advanced").string());
        const auto read_scalar = [&config](const char *key)
        {
          const YAML::Node node = config[key];
          if (!node)
            throw std::runtime_error(std::string("advanced.yaml: '") + key +
                                     "' missing");
          return node.as<double>();
        };
        params.sigmoid_blend = read_scalar("sigmoid_blend");
        params.CL_alpha_0    = read_scalar("cl_alpha_0");
        params.CL_alpha      = read_scalar("cl_alpha");
        params.alpha_stall   = read_scalar("alpha_stall");
        params.eff           = read_scalar("eff");
        params.CD_0          = read_scalar("cd_0");
        params.CD_flat_plate = read_scalar("cd_flat_plate");
        params.CY_beta       = read_scalar("cy_beta");
        params.CL_beta_loss  = read_scalar("cl_beta_loss");
        params.scale_factor  = read_scalar("scale_factor");
      }
      catch (const std::exception &e)
      {
        spdlog::error("advanced aerodynamics: failed to load {}: {}",
                      paths::aero_config("advanced").string(), e.what());
        return false;
      }

      return true;
    }

    // Create a single instance of AeroParams
    AeroParams aero_params{};

    Eigen::Vector3d body_force_coefficients(double alpha, double beta)
    {
      // Calculate sigmoid blending function for pre/post stall
      double sigma =
          (1 +
           exp(-aero_params.sigmoid_blend * (alpha - aero_params.alpha_stall)) +
           exp(aero_params.sigmoid_blend * (alpha + aero_params.alpha_stall))) /
          ((1 + exp(-aero_params.sigmoid_blend *
                    (alpha - aero_params.alpha_stall))) *
           (1 + exp(aero_params.sigmoid_blend *
                    (alpha + aero_params.alpha_stall))));

      // Calculate lift coefficient (CL)
      double sinAlpha = sin(alpha);
      double cosAlpha = cos(alpha);

      // Pre-stall: linear model
      double CL_prestall = (1 - sigma) * (aero_params.CL_alpha * alpha);
      spdlog::debug("CL_prestall: {}", CL_prestall);
      // Post-stall: nonlinear model based on flat plate theory.
      // sign(alpha) with a defined value at alpha == 0 (0/0 would be NaN
      // and poison the product even though sigma is small there).
      const double alpha_sign = (alpha > 0.0) - (alpha < 0.0);
      double       CL_poststall =
          sigma * 2.0 * alpha_sign * sinAlpha * sinAlpha * cosAlpha;
      spdlog::debug("CL_poststall: {}", CL_poststall);

      // here we don't comply to the original model, but scale the CL by
      // the
      double CL = CL_prestall + CL_poststall;
      CL        = CL * cos(beta);
      spdlog::debug("CL: {}", CL);

      // Calculate drag coefficient (CD)
      // Estimate flat plate drag coefficient
      // Here we don't use the original model but choose more simple one
      double CD_fp = abs(aero_params.CD_flat_plate * abs(sin(alpha)));

      // Pre-stall: quadratic drag polar
      // Post-stall: flat plate drag model
      double CD_pre  = aero_params.CD_0 + (CL * CL) / (aero_params.eff);
      double CD_post = CD_fp;
      double CD      = (1 - sigma) * CD_pre + sigma * CD_post;

      // Calculate side force coefficient (CY)
      // Using a simple linear model with respect to sideslip angle
      double CY = aero_params.CY_beta * beta;

      // Resolve drag and lift directly onto the FLU body axes.
      return {-CD * sinAlpha - CL * cosAlpha, CY,
              -CD * cosAlpha + CL * sinAlpha};
    }
  }  // namespace

  class AdvancedLiftDrag::AdvancedLiftDragDataPrivate
  {
   public:
    AdvancedLiftDragDataPrivate() = default;

    bool getAeroWrench(const Eigen::Vector3d &body_speed,
                       Eigen::Vector3d       &aero_force_b,
                       Eigen::Vector3d &aero_moment_b, double &alpha,
                       double &beta)
    {
      std::tie(alpha, beta) = air_angles(body_speed);
      const double vNorm    = body_speed.norm();
      // corner case
      if (vNorm < 1e-6)
      {
        aero_force_b.setZero();
        aero_moment_b.setZero();
        return true;
      }

      aero_force_b = body_force_coefficients(alpha, beta) *
                     body_speed.squaredNorm() * aero_params.scale_factor;
      aero_moment_b.setZero();
      return true;
    }
  };

  // Constructor implementation
  AdvancedLiftDrag::AdvancedLiftDrag()
      : pimpl_(std::make_unique<AdvancedLiftDragDataPrivate>())
  {
  }

  // Destructor implementation
  AdvancedLiftDrag::~AdvancedLiftDrag() = default;

  // Move constructor implementation
  AdvancedLiftDrag::AdvancedLiftDrag(AdvancedLiftDrag &&) noexcept = default;

  // Move assignment implementation
  AdvancedLiftDrag &AdvancedLiftDrag::operator=(AdvancedLiftDrag &&) noexcept =
      default;

  // Forward the getAeroWrench call to implementation
  bool AdvancedLiftDrag::getAeroWrench(const Eigen::Vector3d &body_speed,
                                       Eigen::Vector3d       &aero_force_b,
                                       Eigen::Vector3d       &aero_moment_b,
                                       double &alpha, double &beta)
  {
    return pimpl_->getAeroWrench(body_speed, aero_force_b, aero_moment_b, alpha,
                                 beta);
  }

  bool AdvancedLiftDragCreator::createAerodynamics()
  {
    // Load parameters from config reader
    if (!loadAeroParams(aero_params))
    {
      return false;
    }
    // print aero_params
    spdlog::info("aero_params.sigmoid_blend: {}", aero_params.sigmoid_blend);
    spdlog::info("aero_params.alpha_stall: {}", aero_params.alpha_stall);
    spdlog::info("aero_params.scale_factor: {}", aero_params.scale_factor);

    _instance = std::make_unique<AdvancedLiftDrag>();
    return true;
  }

}  // namespace aerodynamics
