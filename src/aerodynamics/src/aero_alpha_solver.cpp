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

#include <aerodynamics/aero_alpha_solver.hpp>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <toml++/toml.hpp>

namespace aerodynamics
{

// Constants for conversion
constexpr double DEG_TO_RAD = M_PI / 180.0;

AeroAlphaSolver::AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface,
                                 const std::string& toml_path)
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
    throw std::invalid_argument("AeroAlphaSolver: aerodynamics interface cannot be null");
  }

  // Load configuration from TOML
  std::string config_path = toml_path;
  if (config_path.empty())
  {
    const char* env_p = std::getenv("AP_PNC_DIR");
    if (env_p != nullptr)
    {
      config_path = std::string(env_p) + "/global.toml";
    }
  }

  if (!config_path.empty())
  {
    try
    {
      toml::table tbl = toml::parse_file(config_path);
      auto solver_config = tbl["aero_alpha_solver"];

      if (solver_config)
      {
        max_iterations_ = solver_config["max_iterations"].value_or(50);
        tolerance_ = solver_config["tolerance"].value_or(1e-8);
        damping_factor_ = solver_config["damping_factor"].value_or(1.0);
        derivative_epsilon_ = solver_config["derivative_epsilon"].value_or(1e-6);
        initial_alpha_deg_ = solver_config["initial_alpha_deg"].value_or(90.0);

        // Update prev_alpha_rad_ to match loaded config
        prev_alpha_rad_ = initial_alpha_deg_ * DEG_TO_RAD;
      }
    }
    catch (const toml::parse_error& e)
    {
      std::cerr << "Warning: Failed to parse " << config_path << ": " << e.what()
                << ". Using default values." << std::endl;
    }
  }

  // Print initialization info for user identification
  std::cerr << "AeroAlphaSolver initialized with Newton-Raphson solver" << std::endl;
  std::cerr << "  - Max iterations: " << max_iterations_ << std::endl;
  std::cerr << "  - Tolerance: " << std::scientific << tolerance_ << std::fixed << std::endl;
  std::cerr << "  - Initial alpha: " << initial_alpha_deg_ << " degrees" << std::endl;
  if (!config_path.empty())
  {
    std::cerr << "  - Config file: " << config_path << std::endl;
  }
  else
  {
    std::cerr << "  - Config: Using default values (AP_PNC_DIR not set)" << std::endl;
  }
}

double AeroAlphaSolver::solve(double V, double S_ap_x, double S_ap_z, double mass)
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("AeroAlphaSolver::solve: mass must be positive");
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
      prev_alpha_rad_ = alpha;
      last_iterations_ = i + 1;
      return alpha;
    }

    // Compute derivative df/d(alpha)
    double df = computeDerivative(alpha, V, S_ap_x, S_ap_z, mass);

    // Avoid division by zero
    if (std::abs(df) < 1e-12)
    {
      std::stringstream ss;
      ss << "AeroAlphaSolver::solve: Zero derivative encountered at alpha=" << alpha
         << " (iteration " << i << ")";
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
      prev_wrapped_alpha_ = wrapped_alpha;
      unwrapping_initialized_ = true;
    }
  }

  // Convergence failed
  std::stringstream ss;
  ss << "AeroAlphaSolver::solve: Failed to converge after " << max_iterations_
     << " iterations. Final residual: " << std::scientific
     << computeResidual(alpha, V, S_ap_x, S_ap_z, mass) << ", alpha=" << alpha;
  throw std::runtime_error(ss.str());
}

void AeroAlphaSolver::resetAlpha(double initial_alpha_deg)
{
  resetUnwrapping(initial_alpha_deg);
}

void AeroAlphaSolver::resetUnwrapping(double initial_alpha_deg)
{
  // Reset all unwrapping state
  alpha_offset_rad_ = 0.0;
  unwrapping_initialized_ = false;
  prev_wrapped_alpha_ = 0.0;

  // Also reset the temporal coherence state
  prev_alpha_rad_ = initial_alpha_deg * DEG_TO_RAD;
}

int AeroAlphaSolver::getLastIterationCount() const
{
  return last_iterations_;
}

double AeroAlphaSolver::computeResidual(double alpha_rad, double V, double S_ap_x, double S_ap_z,
                                       double mass) const
{
  // LHS: S_ap_z * cos(alpha) + S_ap_x * sin(alpha)
  double lhs = S_ap_z * std::cos(alpha_rad) + S_ap_x * std::sin(alpha_rad);

  // RHS: F_aero_z / mass
  // Construct body velocity: V_b = [V*cos(alpha), 0, V*sin(alpha)]
  Eigen::Vector3d V_b(V * std::cos(alpha_rad), 0.0, V * std::sin(alpha_rad));

  Eigen::Vector3d aero_force_b;
  Eigen::Vector3d aero_moment_b;
  double alpha_out, beta_out;

  // Call aerodynamics model to compute force
  aero_interface_->getAeroWrench(V_b, aero_force_b, aero_moment_b, alpha_out, beta_out);

  double rhs = aero_force_b(2) / mass;

  // Residual: LHS - RHS (should be zero at solution)
  return lhs - rhs;
}

double AeroAlphaSolver::computeDerivative(double alpha_rad, double V, double S_ap_x,
                                          double S_ap_z, double mass) const
{
  double epsilon = derivative_epsilon_;

  // Central difference for better accuracy
  double f_plus = computeResidual(alpha_rad + epsilon, V, S_ap_x, S_ap_z, mass);
  double f_minus = computeResidual(alpha_rad - epsilon, V, S_ap_x, S_ap_z, mass);

  return (f_plus - f_minus) / (2 * epsilon);
}

}  // namespace aerodynamics
