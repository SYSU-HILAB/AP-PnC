/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_DERIVATIVE_H
#define BSPLINE_OPERATORS_DERIVATIVE_H

#include <bspline/internal/misc.h>
#include <bspline/operators/GenericOperators.h>

/*!
 * Operator definitions.
 */
namespace bspline::operators {
namespace internal = bspline::internal;

/*!
 * @brief Derivative operator.
 *
 * Represents the derivative operator \f$\mathrm{d}^n/\mathrm{d}x^n\f$.
 *
 * @tparam n Order of the derivative.
 */
template <size_t n>
class Derivative final : public Operator {
 public:
  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return std::max(n, inputOrder) - n;
  }

  /*!
   * @brief Applies operator to one interval.
   *
   * Applies the operator to a set of coefficients (representing a polynomial on
   * one interval).
   *
   * @param input The polynomial coefficients.
   * @param grid The global grid with respect to which the splines are defined.
   * @param intervalIndex The index of the begin of the interval with respect to
   * the global grid.
   * @tparam T The datatype of the coefficients.
   * @tparam size The size of the input array, i. e. the number of coefficients.
   * @returns The polyomial coefficients arising from the application of this
   * operator to the input coefficients.
   */
  template <typename T, size_t size>
  std::array<T, outputOrder(size - 1) + 1> transform(
      const std::array<T, size> &input,
      [[maybe_unused]] const support::Grid<T> &grid,
      [[maybe_unused]] size_t intervalIndex) const {
    static_assert(size >= 1, "Arrays of size zero not supported.");
    // The order of the input spline.
    constexpr size_t SPLINE_ORDER = size - 1;
    // The size of the output array.
    constexpr size_t OUTPUT_SIZE = outputOrder(SPLINE_ORDER) + 1;

    if constexpr (n > SPLINE_ORDER) {
      return {static_cast<T>(0)};
    } else {
      std::array<T, OUTPUT_SIZE> retVal;
      for (size_t i = 0; i < OUTPUT_SIZE; i++) {
        retVal[i] = internal::facultyRatio<T>(i + n, i) * input[i + n];
      }
      return retVal;
    }
  }
};

/*!
 * @brief Convenient alias for derivative operator.
 *
 * Alias for the derivative operator \f$\mathrm{d}^n/\mathrm{d}x^n\f$.
 *
 * @tparam n Order of the derivative.
 */
template <size_t n>
using Dx = Derivative<n>;

}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_DERIVATIVE_H
