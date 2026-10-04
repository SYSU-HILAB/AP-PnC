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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <yaml-cpp/yaml.h>
#include <aerodynamics/project_paths.hpp>

#include <aerodynamics/aero_alpha_solver.hpp>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aerodynamics
{

  // Constants for conversion
  constexpr double DEG_TO_RAD = M_PI / 180.0;

  AeroAlphaSolver::AeroAlphaSolver(
      std::shared_ptr<AerodynamicsInterface> aero_interface,
      const std::string                     &toml_path)
      : aero_interface_(std::move(aero_interface)),
        max_iterations_(50),
        tolerance_(1e-8),
        damping_factor_(1.0),
        derivative_epsilon_(1e-6),
        initial_alpha_deg_(90.0),
        prev_alpha_rad_(90.0 * DEG_TO_RAD),
        alpha_offset_rad_(0.0),
        unwrapping_initialized_(false),
        prev_wrapped_alpha_(0.0),
        last_iterations_(0)
  {
    if (!aero_interface_)
    {
      throw std::invalid_argument(
          "AeroAlphaSolver: aerodynamics interface cannot be null");
    }

    // Load configuration from YAML
    std::string config_path = toml_path;
    if (config_path.empty())
    {
      // Prior-based experiment config (Lyu force-balance); the prior-free
      // phi-theory path does not depend on this file.
      const char *env_p = std::getenv("AP_PNC_DIR");
      if (env_p != nullptr)
      {
        config_path = (paths::root() /
            "infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml").string();
      }
    }

    if (!config_path.empty())
    {
      config_path = paths::input(config_path).string();
      try
      {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config)
        {
          max_iterations_     = config["max_iterations"].as<int>(50);
          tolerance_          = config["tolerance"].as<double>(1e-8);
          damping_factor_     = config["damping_factor"].as<double>(1.0);
          derivative_epsilon_ = config["derivative_epsilon"].as<double>(1e-6);
          initial_alpha_deg_  = config["initial_alpha_deg"].as<double>(90.0);

          // Update prev_alpha_rad_ to match loaded config
          prev_alpha_rad_ = initial_alpha_deg_ * DEG_TO_RAD;
        }
      }
      catch (const YAML::Exception &e)
      {
        std::cerr << "Warning: Failed to parse " << config_path << ": "
                  << e.what() << ". Using default values." << std::endl;
      }
    }

    // Print initialization info for user identification
    std::cerr << "AeroAlphaSolver initialized with Newton-Raphson solver"
              << std::endl;
    std::cerr << "  - Max iterations: " << max_iterations_ << std::endl;
    std::cerr << "  - Tolerance: " << std::scientific << tolerance_
              << std::fixed << std::endl;
    std::cerr << "  - Initial alpha: " << initial_alpha_deg_ << " degrees"
              << std::endl;
    if (!config_path.empty())
    {
      std::cerr << "  - Config file: " << config_path << std::endl;
    }
    else
    {
      std::cerr << "  - Config: Using default values (AP_PNC_DIR not set)"
                << std::endl;
    }
  }

  AlphaSolverResult AeroAlphaSolver::solve(double V, double S_ap_x,
                                           double S_ap_z, double mass)
  {
    if (mass <= 0.0)
    {
      throw std::invalid_argument(
          "AeroAlphaSolver::solve: mass must be positive");
    }

    // Use previous alpha as initial guess (temporal coherence)
    double alpha = prev_alpha_rad_;

    // Compute initial residual for alpha_dot estimation
    double f_initial = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);

    for (int i = 0; i < max_iterations_; ++i)
    {
      // Compute residual: f(alpha) = LHS - RHS
      double f = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);

      // Check convergence
      if (std::abs(f) < tolerance_)
      {
        // Store previous alpha before updating for next call
        double prev_alpha = prev_alpha_rad_;
        prev_alpha_rad_   = alpha;
        last_iterations_  = i + 1;

        // alpha_dot is set to 0.0 for this version
        // Use the overloaded solve method with time derivatives for accurate
        // alpha_dot
        double alpha_dot = 0.0;

        return AlphaSolverResult{alpha, std::abs(f), alpha_dot};
      }

      // Compute derivative df/d(alpha)
      double df = computeDerivative(alpha, V, S_ap_x, S_ap_z, mass);

      // Avoid division by zero
      if (std::abs(df) < 1e-12)
      {
        std::stringstream ss;
        ss << "AeroAlphaSolver::solve: Zero derivative encountered at alpha="
           << alpha << " (iteration " << i << ")";
        throw std::runtime_error(ss.str());
      }

      // Newton-Raphson update with damping
      double delta = damping_factor_ * f / df;
      alpha -= delta;

      // Store current wrapped alpha before modification
      double wrapped_alpha = alpha;
      while (wrapped_alpha < -M_PI)
      {
        wrapped_alpha += 2 * M_PI;
      }
      while (wrapped_alpha > M_PI)
      {
        wrapped_alpha -= 2 * M_PI;
      }

      // Detect and compensate for wrap events on first iteration
      if (i == 0 && unwrapping_initialized_)
      {
        double delta_wrapped = wrapped_alpha - prev_wrapped_alpha_;

        // If jump is larger than π, it's a wrap event
        if (delta_wrapped > M_PI)
        {
          alpha_offset_rad_ -= 2 * M_PI;  // Wrapped from positive to negative
        }
        else if (delta_wrapped < -M_PI)
        {
          alpha_offset_rad_ += 2 * M_PI;  // Wrapped from negative to positive
        }
      }

      // Apply unwrapping offset
      alpha = wrapped_alpha + alpha_offset_rad_;

      // Update tracking variables
      if (i == 0)
      {
        prev_wrapped_alpha_     = wrapped_alpha;
        unwrapping_initialized_ = true;
      }
    }

    // Convergence failed - throw with residual information
    std::stringstream ss;
    double final_residual = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);
    ss << "AeroAlphaSolver::solve: Failed to converge after " << max_iterations_
       << " iterations. Final residual: " << std::scientific << final_residual
       << ", alpha=" << alpha;
    throw std::runtime_error(ss.str());
  }

  void AeroAlphaSolver::resetAlpha(double initial_alpha_deg)
  {
    resetUnwrapping(initial_alpha_deg);
  }

  void AeroAlphaSolver::resetUnwrapping(double initial_alpha_deg)
  {
    // Reset all unwrapping state
    alpha_offset_rad_       = 0.0;
    unwrapping_initialized_ = false;
    prev_wrapped_alpha_     = 0.0;

    // Also reset the temporal coherence state
    prev_alpha_rad_ = initial_alpha_deg * DEG_TO_RAD;
  }

  int AeroAlphaSolver::getLastIterationCount() const
  {
    return last_iterations_;
  }

  double AeroAlphaSolver::computeResidual(double alpha_rad, double V,
                                          double S_ap_x, double S_ap_z,
                                          double mass) const
  {
    // LHS: S_ap_z * cos(alpha) + S_ap_x * sin(alpha)
    double lhs = S_ap_z * std::cos(alpha_rad) + S_ap_x * std::sin(alpha_rad);

    // RHS: F_aero_z / mass
    // Construct body velocity: V_b = [V*cos(alpha), 0, V*sin(alpha)]
    Eigen::Vector3d V_b(V * std::cos(alpha_rad), 0.0, V * std::sin(alpha_rad));

    Eigen::Vector3d aero_force_b;
    Eigen::Vector3d aero_moment_b;
    double          alpha_out, beta_out;

    // Call aerodynamics model to compute force
    aero_interface_->getAeroWrench(V_b, aero_force_b, aero_moment_b, alpha_out,
                                   beta_out);

    double rhs = aero_force_b(2) / mass;

    // Residual: LHS - RHS (should be zero at solution)
    return lhs - rhs;
  }

  double AeroAlphaSolver::computeDerivative(double alpha_rad, double V,
                                            double S_ap_x, double S_ap_z,
                                            double mass) const
  {
    double epsilon = derivative_epsilon_;

    // Central difference for better accuracy
    double f_plus =
        computeResidual(alpha_rad + epsilon, V, S_ap_x, S_ap_z, mass);
    double f_minus =
        computeResidual(alpha_rad - epsilon, V, S_ap_x, S_ap_z, mass);

    return (f_plus - f_minus) / (2 * epsilon);
  }

  double AeroAlphaSolver::computeDerivativeS_ap_x(double alpha_rad) const
  {
    // df/dS_ap_x = sin(alpha)
    // From: f = S_ap_z*cos(alpha) + S_ap_x*sin(alpha) - F_aero_z/mass
    return std::sin(alpha_rad);
  }

  double AeroAlphaSolver::computeDerivativeS_ap_z(double alpha_rad) const
  {
    // df/dS_ap_z = cos(alpha)
    // From: f = S_ap_z*cos(alpha) + S_ap_x*sin(alpha) - F_aero_z/mass
    return std::cos(alpha_rad);
  }

  double AeroAlphaSolver::computeDerivativeV(double alpha_rad, double V,
                                             double mass) const
  {
    double epsilon = derivative_epsilon_;

    // Central difference: df/dV
    // Note: This computes the partial derivative with respect to V, treating
    // alpha as constant The residual depends on V through the aerodynamic force
    // F_aero_z(V_b)
    double f_plus  = computeResidual(alpha_rad, V + epsilon, 0.0, 0.0, mass);
    double f_minus = computeResidual(alpha_rad, V - epsilon, 0.0, 0.0, mass);

    return (f_plus - f_minus) / (2 * epsilon);
  }

  double AeroAlphaSolver::computeAlphaDot(double alpha_rad, double V,
                                          double S_ap_x_dot, double S_ap_z_dot,
                                          double mass) const
  {
    // Using implicit function theorem:
    // df/dt = ∂f/∂alpha * alpha_dot + ∂f/∂S_ap_x * S_ap_x_dot + ∂f/∂S_ap_z *
    // S_ap_z_dot + ∂f/∂V * V_dot = 0
    //
    // Therefore:
    // alpha_dot = -(∂f/∂S_ap_x * S_ap_x_dot + ∂f/∂S_ap_z * S_ap_z_dot + ∂f/∂V *
    // V_dot) / (∂f/∂alpha)
    //
    // Note: We assume V_dot = 0 for trajectory planning (velocity is
    // independent variable) If V_dot is needed, it should be added to the
    // computation.

    double df_dalpha  = computeDerivative(alpha_rad, V, 0.0, 0.0, mass);
    double df_dS_ap_x = computeDerivativeS_ap_x(alpha_rad);
    double df_dS_ap_z = computeDerivativeS_ap_z(alpha_rad);
    // df_dV is multiplied by V_dot which we assume to be 0, so we skip it

    // Avoid division by zero
    if (std::abs(df_dalpha) < 1e-12)
    {
      return 0.0;
    }

    // Compute alpha_dot using implicit function theorem
    double alpha_dot =
        -(df_dS_ap_x * S_ap_x_dot + df_dS_ap_z * S_ap_z_dot) / df_dalpha;

    return alpha_dot;
  }

  // Overloaded solve method with time derivatives for accurate alpha_dot
  // computation
  AlphaSolverResult AeroAlphaSolver::solve(double V, double S_ap_x,
                                           double S_ap_z, double S_ap_x_dot,
                                           double S_ap_z_dot, double mass)
  {
    if (mass <= 0.0)
    {
      throw std::invalid_argument(
          "AeroAlphaSolver::solve: mass must be positive");
    }

    // Use previous alpha as initial guess (temporal coherence)
    double alpha = prev_alpha_rad_;

    for (int i = 0; i < max_iterations_; ++i)
    {
      // Compute residual: f(alpha) = LHS - RHS
      double f = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);

      // Check convergence
      if (std::abs(f) < tolerance_)
      {
        // Update state for next call
        prev_alpha_rad_  = alpha;
        last_iterations_ = i + 1;

        // Compute alpha_dot using implicit function theorem (mid difference)
        double alpha_dot_val =
            computeAlphaDot(alpha, V, S_ap_x_dot, S_ap_z_dot, mass);

        return AlphaSolverResult{alpha, std::abs(f), alpha_dot_val};
      }

      // Compute derivative df/d(alpha)
      double df = computeDerivative(alpha, V, S_ap_x, S_ap_z, mass);

      // Avoid division by zero
      if (std::abs(df) < 1e-12)
      {
        std::stringstream ss;
        ss << "AeroAlphaSolver::solve: Zero derivative encountered at alpha="
           << alpha << " (iteration " << i << ")";
        throw std::runtime_error(ss.str());
      }

      // Newton-Raphson update with damping
      double delta = damping_factor_ * f / df;
      alpha -= delta;

      // Store current wrapped alpha before modification
      double wrapped_alpha = alpha;
      while (wrapped_alpha < -M_PI)
      {
        wrapped_alpha += 2 * M_PI;
      }
      while (wrapped_alpha > M_PI)
      {
        wrapped_alpha -= 2 * M_PI;
      }

      // Detect and compensate for wrap events on first iteration
      if (i == 0 && unwrapping_initialized_)
      {
        double delta_wrapped = wrapped_alpha - prev_wrapped_alpha_;

        // If jump is larger than π, it's a wrap event
        if (delta_wrapped > M_PI)
        {
          alpha_offset_rad_ -= 2 * M_PI;  // Wrapped from positive to negative
        }
        else if (delta_wrapped < -M_PI)
        {
          alpha_offset_rad_ += 2 * M_PI;  // Wrapped from negative to positive
        }
      }

      // Apply unwrapping offset
      alpha = wrapped_alpha + alpha_offset_rad_;

      // Update tracking variables
      if (i == 0)
      {
        prev_wrapped_alpha_     = wrapped_alpha;
        unwrapping_initialized_ = true;
      }
    }

    // Convergence failed - throw with residual information
    std::stringstream ss;
    double final_residual = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);
    ss << "AeroAlphaSolver::solve: Failed to converge after " << max_iterations_
       << " iterations. Final residual: " << std::scientific << final_residual
       << ", alpha=" << alpha;
    throw std::runtime_error(ss.str());
  }

}  // namespace aerodynamics
