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

/**
 * @file bspline_basis.hpp
 * @brief Self-contained B-spline basis evaluation (Cox-de Boor / NURBS Book).
 *
 * Only the evaluation of B-spline basis functions and curves is needed by the
 * aerodynamics models; knot vectors and control points are produced offline
 * (Python/scipy) and loaded from the aero database. This header replaces the
 * third-party B-spline dependency with a small, dependency-free implementation.
 *
 * References: Piegl & Tiller, "The NURBS Book", algorithms A2.2 (FindSpan) and
 * A2.3 (BasisFuns).
 */

#pragma once

#include <cstddef>
#include <vector>

namespace aerodynamics
{

/**
 * @brief Number of B-spline basis functions for a knot vector and degree.
 *
 * n_basis = m - (p + 1), where m = knots.size() and p = degree.
 *
 * @param[in] n_knots Number of knots m
 * @param[in] degree  Spline degree p
 * @return Number of basis functions (0 if the knot vector is too short)
 */
inline std::size_t bspline_num_basis(std::size_t n_knots, std::size_t degree)
{
  return n_knots > degree + 1 ? n_knots - (degree + 1) : 0;
}

namespace detail
{

  /**
   * @brief Find the knot span containing x (The NURBS Book, A2.2).
   */
  inline std::size_t find_span(std::size_t n_basis, std::size_t degree,
                               double x, const std::vector<double> &knots)
  {
    const std::size_t n = n_basis - 1;  // index of the last basis function
    if (x >= knots[n + 1])
    {
      return n;
    }
    if (x <= knots[degree])
    {
      return degree;
    }
    std::size_t low = degree;
    std::size_t high = n + 1;
    std::size_t mid = (low + high) / 2;
    while (x < knots[mid] || x >= knots[mid + 1])
    {
      if (x < knots[mid])
      {
        high = mid;
      }
      else
      {
        low = mid;
      }
      mid = (low + high) / 2;
    }
    return mid;
  }

  /**
   * @brief Nonzero basis values N_{span-p, .., span} at x (The NURBS Book, A2.3).
   */
  inline std::vector<double> basis_funs(std::size_t span, double x,
                                        std::size_t degree,
                                        const std::vector<double> &knots)
  {
    std::vector<double> N(degree + 1, 0.0);
    std::vector<double> left(degree + 1, 0.0);
    std::vector<double> right(degree + 1, 0.0);
    N[0] = 1.0;
    for (std::size_t j = 1; j <= degree; j++)
    {
      left[j]  = x - knots[span + 1 - j];
      right[j] = knots[span + j] - x;
      double saved = 0.0;
      for (std::size_t r = 0; r < j; r++)
      {
        const double denom = right[r + 1] + left[j - r];
        const double temp  = denom != 0.0 ? N[r] / denom : 0.0;
        N[r]               = saved + right[r + 1] * temp;
        saved              = left[j - r] * temp;
      }
      N[j] = saved;
    }
    return N;
  }

}  // namespace detail

/**
 * @brief Evaluate all B-spline basis functions of `degree` at `x`.
 *
 * @param[in] degree Spline degree p
 * @param[in] knots  Knot vector (size m)
 * @param[in] x      Evaluation point
 * @return Basis values, length bspline_num_basis(knots.size(), degree)
 */
inline std::vector<double> bspline_basis(std::size_t degree,
                                         const std::vector<double> &knots,
                                         double x)
{
  const std::size_t   n_basis = bspline_num_basis(knots.size(), degree);
  std::vector<double> basis(n_basis, 0.0);
  if (n_basis == 0)
  {
    return basis;
  }
  const std::size_t         span = detail::find_span(n_basis, degree, x, knots);
  const std::vector<double> N    = detail::basis_funs(span, x, degree, knots);
  for (std::size_t r = 0; r <= degree; r++)
  {
    basis[span - degree + r] = N[r];
  }
  return basis;
}

/**
 * @brief Evaluate a B-spline curve sum_i c_i * N_{i,degree}(x).
 *
 * @param[in] coeffs Control points (length should equal the basis count)
 * @param[in] knots  Knot vector (size m)
 * @param[in] degree Spline degree p
 * @param[in] x      Evaluation point
 * @return Curve value
 */
inline double evaluate_bspline(const std::vector<double> &coeffs,
                               const std::vector<double> &knots,
                               std::size_t degree, double x)
{
  const std::size_t n_basis = bspline_num_basis(knots.size(), degree);
  if (n_basis == 0)
  {
    return 0.0;
  }
  const std::size_t         span = detail::find_span(n_basis, degree, x, knots);
  const std::vector<double> N    = detail::basis_funs(span, x, degree, knots);
  double                    value = 0.0;
  for (std::size_t r = 0; r <= degree; r++)
  {
    const std::size_t i = span - degree + r;
    if (i < coeffs.size())
    {
      value += coeffs[i] * N[r];
    }
  }
  return value;
}

}  // namespace aerodynamics
