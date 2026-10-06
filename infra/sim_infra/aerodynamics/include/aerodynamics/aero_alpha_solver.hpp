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
#include <string>

#include <Eigen/Dense>

#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{

/**
 * @brief Result structure for alpha solver
 *
 * Contains the solved alpha value along with diagnostic information
 * about the solution quality and dynamics.
 */
struct AlphaSolverResult
{
  double alpha;       ///< Solved angle of attack (radians)
  double residual;    ///< Final residual value (|f(alpha)|) at solution
  double alpha_dot;   ///< Estimated rate of change of alpha (rad/s)

  /**
   * @brief Factory method: Create result from just alpha value (legacy compatibility)
   *
   * @param alpha_val Solved angle of attack (radians)
   * @return AlphaSolverResult with zero residual and alpha_dot
   */
  static AlphaSolverResult from_alpha(double alpha_val)
  {
    return AlphaSolverResult{ alpha_val, 0.0, 0.0 };
  }
};

/**
 * @brief Newton-Raphson solver for angle of attack from force balance equation
 *
 * Solves the aerodynamic force balance equation:
 * S_ap_z * cos(alpha) + S_ap_x * sin(alpha) = F_aero_z([V*cos(alpha), 0, V*sin(alpha)]) / mass
 *
 * This is a direct aerodynamic force balance approach without lookup tables.
 * The solver uses Newton-Raphson iteration with temporal coherence for fast convergence.
 *
 * Configuration is read directly from $AP_PNC_DIR/infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml:
 * - max_iterations: Maximum Newton-Raphson iterations (default: 50)
 * - tolerance: Convergence tolerance for |f(alpha)| (default: 1e-8)
 * - damping_factor: Damping factor (0 < d <= 1) for stability (default: 1.0)
 * - derivative_epsilon: Epsilon for numerical derivative (default: 1e-6)
 * - initial_alpha_deg: Initial alpha guess in degrees (default: 90.0)
 *
 * @note The solver is stateful and maintains the previous alpha value for temporal coherence.
 * This makes it non-thread-safe by design - each trajectory planning instance should have
 * its own solver.
 */
class AeroAlphaSolver
{
 public:
  /**
   * @brief Constructor - reads configuration from config/aero_alpha_solver.yaml
   *
   * Reads configuration from $AP_PNC_DIR/infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml.
   * If AP_PNC_DIR is not set or the file cannot be parsed, uses default values.
   *
   * @param aero_interface Aerodynamics model for computing forces
   * @param toml_path Optional custom path to YAML configuration file
   */
  explicit AeroAlphaSolver(std::shared_ptr<AerodynamicsInterface> aero_interface,
                           const std::string& toml_path = "");

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
   * @return AlphaSolverResult containing alpha, residual, and alpha_dot (set to 0)
   *
   * @throws std::runtime_error if solver fails to converge
   *
   * @note This version does not compute alpha_dot. Use the overloaded version with time derivatives
   *       for accurate alpha_dot computation using implicit function theorem.
   */
  AlphaSolverResult solve(double V, double S_ap_x, double S_ap_z, double mass);

  /**
   * @brief Solve for alpha with time derivatives for accurate alpha_dot computation
   *
   * Solves the force balance equation and computes alpha_dot using implicit function theorem:
   * alpha_dot = -(df/dS_ap_x * S_ap_x_dot + df/dS_ap_z * S_ap_z_dot + df/dV * V_dot) / (df/dalpha)
   *
   * @param V Velocity magnitude (m/s)
   * @param S_ap_x Specific force x-component (m/s²)
   * @param S_ap_z Specific force z-component (m/s²)
   * @param S_ap_x_dot Time derivative of S_ap_x (m/s³)
   * @param S_ap_z_dot Time derivative of S_ap_z (m/s³)
   * @param mass Vehicle mass (kg)
   * @return AlphaSolverResult containing alpha, residual, and alpha_dot
   *
   * @throws std::runtime_error if solver fails to converge
   */
  AlphaSolverResult solve(double V, double S_ap_x, double S_ap_z, double S_ap_x_dot,
                          double S_ap_z_dot, double mass);

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
   * @brief Get number of iterations from last solve
   *
   * @return Number of iterations used in the last solve call
   */
  int getLastIterationCount() const;

 private:
  /// Aerodynamics model for computing forces
  std::shared_ptr<AerodynamicsInterface> aero_interface_;

  /// Solver configuration parameters (read from config/aero_alpha_solver.yaml)
  int max_iterations_;              ///< Maximum Newton-Raphson iterations
  double tolerance_;                ///< Convergence tolerance for |f(alpha)|
  double damping_factor_;           ///< Damping factor (0 < d <= 1) for stability
  double derivative_epsilon_;       ///< Epsilon for numerical derivative
  double initial_alpha_deg_;        ///< Initial alpha guess (degrees)

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

  /**
   * @brief Compute partial derivative df/dS_ap_x (mid difference)
   *
   * @param alpha_rad Angle of attack in radians
   * @return sin(alpha)
   */
  double computeDerivativeS_ap_x(double alpha_rad) const;

  /**
   * @brief Compute partial derivative df/dS_ap_z (mid difference)
   *
   * @param alpha_rad Angle of attack in radians
   * @return cos(alpha)
   */
  double computeDerivativeS_ap_z(double alpha_rad) const;

  /**
   * @brief Compute partial derivative df/dV using central difference
   *
   * @param alpha_rad Angle of attack in radians
   * @param V Velocity magnitude (m/s)
   * @param mass Vehicle mass (kg)
   * @return Numerical derivative of residual with respect to V
   */
  double computeDerivativeV(double alpha_rad, double V, double mass) const;

  /**
   * @brief Compute alpha_dot using implicit function theorem
   *
   * From f(alpha, S_ap_x, S_ap_z, V) = 0:
   * alpha_dot = -(df/dS_ap_x * S_ap_x_dot + df/dS_ap_z * S_ap_z_dot + df/dV * V_dot) / (df/dalpha)
   *
   * @param alpha_rad Solved angle of attack (radians)
   * @param V Velocity magnitude (m/s)
   * @param S_ap_x_dot Time derivative of S_ap_x (m/s³)
   * @param S_ap_z_dot Time derivative of S_ap_z (m/s³)
   * @param mass Vehicle mass (kg)
   * @return alpha_dot in rad/s
   */
  double computeAlphaDot(double alpha_rad, double V, double S_ap_x_dot, double S_ap_z_dot,
                         double mass) const;
};

}  // namespace aerodynamics
