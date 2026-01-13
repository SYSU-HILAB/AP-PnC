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

#include <string>
#include <utility>
#include <vector>

namespace aerodynamics
{

/**
 * @brief Aerodynamic lift coefficient (cz) lookup table class
 *
 * This class provides a lookup table for the lift coefficient (cz) as a function
 * of angle of attack (alpha), and also supports inverse lookup (cz -> alpha)
 * using temporal coherence.
 */
class CzCalculate
{
 public:
  /**
   * @brief Result structure containing cz value and its slope
   */
  struct CzResult
  {
    double cz_value; ///< Lift coefficient value
    double slope;    ///< Derivative of cz with respect to alpha (d(cz)/d(alpha))
  };

  /**
   * @brief Constructor that loads aerodynamic data from a CSV file
   *
   * @param csv_file Path to the CSV file containing aerodynamic data
   *
   * The CSV file should have the format:
   * alpha_deg,cx,cz
   * -180.0,0.0,-0.5
   * ...
   */
  explicit CzCalculate(const std::string& csv_file);

  /**
   * @brief Get cz value for a given angle of attack
   *
   * @param alpha_deg Angle of attack in degrees
   * @return Lift coefficient value
   */
  double getCz(double alpha_deg) const;

  /**
   * @brief Get alpha value from cz using temporal coherence
   *
   * Finds all alpha values matching the given cz (within tolerance),
   * then selects the one closest to the previous alpha value.
   *
   * @param cz Lift coefficient value
   * @return Angle of attack in degrees
   */
  double getAlpha(double cz) const;

  /**
   * @brief Reset the previous alpha value for temporal coherence
   *
   * @param initial_alpha Initial alpha value in degrees
   */
  void resetAlpha(double initial_alpha);

 private:
  /// Vector of (alpha_deg, cz) pairs, sorted by alpha_deg
  std::vector<std::pair<double, double>> data_;

  /// Previous alpha value for temporal coherence
  mutable double prev_alpha_;

  /**
   * @brief Load aerodynamic data from CSV file
   */
  void loadData(const std::string& csv_file);
};

}  // namespace aerodynamics
