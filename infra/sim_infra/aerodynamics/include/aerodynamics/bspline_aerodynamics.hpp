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

#pragma once
#include <aerodynamics/aero_interface.hpp>
#include <aerodynamics/bspline_basis.hpp>

namespace aerodynamics
{

  /**
   * @brief B-spline based aerodynamics model
   *
   * This class implements aerodynamic forces using B-spline curves to model
   * the force coefficients as functions of angle of attack and sideslip
   * angle.
   */
  class BsplineAerodynamics : public AerodynamicsInterface
  {
   private:
    static constexpr size_t SPLINE_ORDER = 3;

    /// Knot vectors for x-axis coefficient splines
    std::vector<double> cx_knots_;
    /// Knot vectors for z-axis coefficient splines
    std::vector<double> cz_knots_;
    /// Control points for x-axis coefficient splines
    std::vector<double> cx_coefficients_;
    /// Control points for z-axis coefficient splines
    std::vector<double> cz_coefficients_;

    /// Default scale factor based on typical values
    constexpr static double default_scale_factor_ = .5 * 1.184 * .24 / 2.0;
    /// Scale factor for force calculations (1/2 * rho * S / m)
    double scale_factor_;

   public:
    /**
     * @brief Construct a new Bspline Aerodynamics object
     *
     * @param cx_coefs Control points for x-axis force coefficient splines
     * @param cx_knots Knot vector for x-axis force coefficient splines
     * @param cz_coefs Control points for z-axis force coefficient splines
     * @param cz_knots Knot vector for z-axis force coefficient splines
     * @param scale_factor Scale factor for force calculations (default
     * based on typical values)
     */
    explicit BsplineAerodynamics(
        const std::vector<double>& cx_coefs,
        const std::vector<double>& cx_knots,
        const std::vector<double>& cz_coefs,
        const std::vector<double>& cz_knots,
        const double               scale_factor = default_scale_factor_);

    /**
     * @brief Compute aerodynamic forces using B-spline interpolation
     *
     * Forces are computed by evaluating B-spline curves that represent
     * force coefficients as functions of angle of attack and sideslip
     * angle.
     */
    bool getAeroWrench(const Eigen::Vector3d& body_speed,
                       Eigen::Vector3d&       aero_force_b,
                       Eigen::Vector3d& aero_moment_b, double& alpha,
                       double& beta) override;

    ~BsplineAerodynamics() = default;
  };

  /**
   * @brief Factory for creating BsplineAerodynamics instances
   *
   * Creates and initializes BsplineAerodynamics objects with parameters
   * loaded from configuration.
   */
  class BsplineAerodynamicsCreator : public AerodynamicsCreator
  {
   public:
    bool createAerodynamics() override;
    ~BsplineAerodynamicsCreator() = default;
  };

}  // namespace aerodynamics
