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
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>

namespace aerodynamics
{

// Constants for conversion
constexpr double DEG_TO_RAD = M_PI / 180.0;

AeroAlphaSolver::AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface)
    : aero_interface_(std::move(aero_interface)),
      config_(),
      prev_alpha_rad_(config_.initial_alpha_deg * DEG_TO_RAD),
      last_iterations_(0)
{
  if (!aero_interface_)
  {
    throw std::invalid_argument("AeroAlphaSolver: aerodynamics interface cannot be null");
  }

  // Print initialization info for user identification
  std::cout << "AeroAlphaSolver initialized with Newton-Raphson solver" << std::endl;
  std::cout << "  - Max iterations: " << config_.max_iterations << std::endl;
  std::cout << "  - Tolerance: " << std::scientific << config_.tolerance << std::fixed
            << std::endl;
  std::cout << "  - Initial alpha: " << config_.initial_alpha_deg << " degrees" << std::endl;
}

AeroAlphaSolver::AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface,
                                 const Config& config)
    : aero_interface_(std::move(aero_interface)),
      config_(config),
      prev_alpha_rad_(config_.initial_alpha_deg * DEG_TO_RAD),
      last_iterations_(0)
{
  if (!aero_interface_)
  {
    throw std::invalid_argument("AeroAlphaSolver: aerodynamics interface cannot be null");
  }

  // Print initialization info for user identification
  std::cout << "AeroAlphaSolver initialized with Newton-Raphson solver (custom config)"
            << std::endl;
  std::cout << "  - Max iterations: " << config_.max_iterations << std::endl;
  std::cout << "  - Tolerance: " << std::scientific << config_.tolerance << std::fixed
            << std::endl;
  std::cout << "  - Initial alpha: " << config_.initial_alpha_deg << " degrees" << std::endl;
}

double AeroAlphaSolver::solve(double V, double S_ap_x, double S_ap_z, double mass)
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("AeroAlphaSolver::solve: mass must be positive");
  }

  // Use previous alpha as initial guess (temporal coherence)
  double alpha = prev_alpha_rad_;

  for (int i = 0; i < config_.max_iterations; ++i)
  {
    // Compute residual: f(alpha) = LHS - RHS
    double f = computeResidual(alpha, V, S_ap_x, S_ap_z, mass);

    // Check convergence
    if (std::abs(f) < config_.tolerance)
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
    double delta = config_.damping_factor * f / df;
    alpha -= delta;

    // Wrap alpha to [-pi, pi] for numerical stability
    while (alpha < -M_PI)
    {
      alpha += 2 * M_PI;
    }
    while (alpha > M_PI)
    {
      alpha -= 2 * M_PI;
    }
  }

  // Convergence failed
  std::stringstream ss;
  ss << "AeroAlphaSolver::solve: Failed to converge after " << config_.max_iterations
     << " iterations. Final residual: " << std::scientific
     << computeResidual(alpha, V, S_ap_x, S_ap_z, mass) << ", alpha=" << alpha;
  throw std::runtime_error(ss.str());
}

void AeroAlphaSolver::resetAlpha(double initial_alpha_deg)
{
  prev_alpha_rad_ = initial_alpha_deg * DEG_TO_RAD;
}

AeroAlphaSolver::Config AeroAlphaSolver::getConfig() const
{
  return config_;
}

void AeroAlphaSolver::setConfig(const Config& config)
{
  config_ = config;
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
  double epsilon = config_.derivative_epsilon;

  // Central difference for better accuracy
  double f_plus = computeResidual(alpha_rad + epsilon, V, S_ap_x, S_ap_z, mass);
  double f_minus = computeResidual(alpha_rad - epsilon, V, S_ap_x, S_ap_z, mass);

  return (f_plus - f_minus) / (2 * epsilon);
}

}  // namespace aerodynamics
