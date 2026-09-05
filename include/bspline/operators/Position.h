/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_POSITION_H
#define BSPLINE_OPERATORS_POSITION_H

#include <bspline/internal/misc.h>
#include <bspline/operators/GenericOperators.h>

/*!
 * Operator definitions.
 */
namespace bspline::operators {
namespace internal = bspline::internal;

/*!
 * @brief Position operator.
 *
 * Represents a power of the position operator \f$x^n\f$.
 *
 * @tparam n Order of the power.
 */
template <size_t n>
class Position final : public Operator {
 private:
  /*!
   * Expands \f$(x + x_m)^n\f$.
   *
   * @param xm The middle point of the interval.
   * @tparam T The datatype of xm and the returned coefficients.
   * @returns The polynomial coefficients.
   */
  template <typename T>
  static std::array<T, n + 1> expandPower(const T &xm) {
    std::array<T, n + 1> retVal;
    T power_of_xm = static_cast<T>(1);
    for (size_t i = 0; i < n + 1; i++) {
      retVal[n - i] = internal::binomialCoefficient<T>(n, i) * power_of_xm;
      power_of_xm *= xm;
    }
    return retVal;
  }

 public:
  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return inputOrder + n;
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
      const std::array<T, size> &input, const support::Grid<T> &grid,
      size_t intervalIndex) const {
    constexpr size_t OUTPUT_SIZE = size + n;

    const T xm =
        (grid[intervalIndex] + grid[intervalIndex + 1]) / static_cast<T>(2);

    const std::array<T, n + 1> expanded = expandPower<T>(xm);

    std::array<T, OUTPUT_SIZE> retVal;
    retVal.fill(static_cast<T>(0));

    for (size_t i = 0; i < input.size(); i++) {
      for (size_t j = 0; j < expanded.size(); j++) {
        retVal[i + j] += expanded[j] * input[i];
      }
    }
    return retVal;
  }
};

/*!
 * @brief Alias for Position operator.
 *
 * Alias for the power \f$x^n\f$ of the position operator.
 *
 * @tparam n Order of the power of the position operator.
 */
template <size_t n>
using X = Position<n>;

}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_POSITION_H
