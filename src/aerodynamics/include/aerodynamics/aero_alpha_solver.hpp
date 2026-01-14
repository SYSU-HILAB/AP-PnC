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

#pragma once

#include <memory>

#include <Eigen/Dense>

#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{

/**
 * @brief Newton-Raphson solver for angle of attack from force balance equation
 *
 * Solves the aerodynamic force balance equation:
 * S_ap_z * cos(alpha) + S_ap_x * sin(alpha) = F_aero_z([V*cos(alpha), 0, V*sin(alpha)]) / mass
 *
 * This is a direct aerodynamic force balance approach without lookup tables.
 * The solver uses Newton-Raphson iteration with temporal coherence for fast convergence.
 *
 * @note The solver is stateful and maintains the previous alpha value for temporal coherence.
 * This makes it non-thread-safe by design - each trajectory planning instance should have
 * its own solver.
 */
class AeroAlphaSolver
{
 public:
  /**
   * @brief Configuration for Newton-Raphson solver
   */
  struct Config
  {
    int max_iterations = 50;        ///< Maximum Newton-Raphson iterations
    double tolerance = 1e-8;        ///< Convergence tolerance for |f(alpha)|
    double damping_factor = 1.0;    ///< Damping factor (0 < d <= 1) for stability
    double derivative_epsilon = 1e-6; ///< Epsilon for numerical derivative
    double initial_alpha_deg = 90.0; ///< Initial alpha guess (degrees)
  };

  /**
   * @brief Constructor with default configuration
   *
   * @param aero_interface Aerodynamics model for computing forces
   */
  explicit AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface);

  /**
   * @brief Constructor with custom configuration
   *
   * @param aero_interface Aerodynamics model for computing forces
   * @param config Solver configuration
   */
  AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface,
                  const Config& config);

  ~AeroAlphaSolver() = default;

  /**
   * @brief Solve for alpha given force balance parameters
   *
   * Solves the force balance equation:
   * S_ap_z * cos(alpha) + S_ap_x * sin(alpha) = F_aero_z(V_b) / mass
   *
   * where V_b = [V*cos(alpha), 0, V*sin(alpha)] is the body velocity vector.
   *
   * @param V Velocity magnitude (m/s)
   * @param S_ap_x Specific force x-component (m/s²), projection of (a-g) onto velocity direction
   * @param S_ap_z Specific force z-component (m/s²), projection of (a-g) onto perpendicular direction
   * @param mass Vehicle mass (kg)
   * @return Alpha in radians
   *
   * @throws std::runtime_error if solver fails to converge
   */
  double solve(double V, double S_ap_x, double S_ap_z, double mass);

  /**
   * @brief Reset temporal coherence state
   *
   * Resets the previous alpha value used as the initial guess for Newton-Raphson.
   * Typically called at the start of a new trajectory.
   *
   * @param initial_alpha_deg Initial alpha value in degrees (typically 90.0)
   */
  void resetAlpha(double initial_alpha_deg);

  /**
   * @brief Reset angle unwrapping state
   *
   * Resets the cumulative angle offset used for continuity. Call this when starting
   * a new trajectory to prevent accumulated drift.
   *
   * @param initial_alpha_deg Initial alpha value in degrees
   */
  void resetUnwrapping(double initial_alpha_deg);

  /**
   * @brief Get current solver configuration
   *
   * @return Current configuration
   */
  Config getConfig() const;

  /**
   * @brief Update solver configuration
   *
   * @param config New configuration
   */
  void setConfig(const Config& config);

  /**
   * @brief Get number of iterations from last solve
   *
   * @return Number of iterations used in the last solve call
   */
  int getLastIterationCount() const;

 private:
  /// Aerodynamics model for computing forces
  std::shared_ptr<AerodynamicsInterface> aero_interface_;

  /// Solver configuration
  Config config_;

  /// Previous alpha for temporal coherence (radians)
  mutable double prev_alpha_rad_;

  /// Cumulative angle offset for unwrapping (radians)
  mutable double alpha_offset_rad_;

  /// Flag to track if unwrapping is initialized
  mutable bool unwrapping_initialized_;

  /// Previous wrapped alpha for detecting wrap events
  mutable double prev_wrapped_alpha_;

  /// Iterations used in last solve
  mutable int last_iterations_;

  /**
   * @brief Compute residual function f(alpha) = LHS - RHS
   *
   * LHS = S_ap_z * cos(alpha) + S_ap_x * sin(alpha)
   * RHS = F_aero_z(V_b) / mass
   * where V_b = [V*cos(alpha), 0, V*sin(alpha)]
   *
   * @param alpha_rad Angle of attack in radians
   * @param V Velocity magnitude (m/s)
   * @param S_ap_x Specific force x-component (m/s²)
   * @param S_ap_z Specific force z-component (m/s²)
   * @param mass Vehicle mass (kg)
   * @return Residual value (should be zero at solution)
   */
  double computeResidual(double alpha_rad, double V, double S_ap_x, double S_ap_z,
                         double mass) const;

  /**
   * @brief Compute numerical derivative df/d(alpha) using central difference
   *
   * @param alpha_rad Angle of attack in radians
   * @param V Velocity magnitude (m/s)
   * @param S_ap_x Specific force x-component (m/s²)
   * @param S_ap_z Specific force z-component (m/s²)
   * @param mass Vehicle mass (kg)
   * @return Numerical derivative
   */
  double computeDerivative(double alpha_rad, double V, double S_ap_x, double S_ap_z,
                           double mass) const;
};

}  // namespace aerodynamics
