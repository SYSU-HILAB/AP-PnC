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

#include <utility>
#include <vector>

namespace aerodynamics
{

/**
 * @brief Aerodynamic lift coefficient (cz) lookup table class
 *
 * This class provides a lookup table for the lift coefficient (cz) as a function
 * of angle of attack (alpha). The table is loaded from a CSV file with columns:
 * alpha_deg, cx, cz
 *
 * The class uses binary search for efficient lookup and provides both the cz value
 * and its derivative (slope) at the queried angle of attack.
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
   *
   * @throws std::runtime_error if the file cannot be opened
   */
  explicit CzCalculate(const std::string& csv_file);

  /**
   * @brief Get cz value and slope for a given angle of attack
   *
   * Performs binary search to find the interval containing the query point,
   * then linearly interpolates to get the cz value and computes the slope.
   *
   * @param alpha_deg Angle of attack in degrees
   * @return CzResult containing cz_value and slope
   *
   * @note Returns boundary values if alpha_deg is outside the loaded range
   */
  CzResult getCzWithSlope(double alpha_deg) const;

  /**
   * @brief Get only the cz value for a given angle of attack
   *
   * Convenience wrapper around getCzWithSlope() that returns only the value.
   *
   * @param alpha_deg Angle of attack in degrees
   * @return Lift coefficient value
   */
  double getCz(double alpha_deg) const;

 private:
  /// Vector of (alpha_deg, cz) pairs, sorted by alpha_deg
  std::vector<std::pair<double, double>> data_;

  /**
   * @brief Load aerodynamic data from CSV file
   *
   * @param csv_file Path to the CSV file
   */
  void loadData(const std::string& csv_file);
};

}  // namespace aerodynamics
