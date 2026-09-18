/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_SPLINEOPERATOR_H
#define BSPLINE_OPERATORS_SPLINEOPERATOR_H

#include <bspline/Spline.h>
#include <bspline/exceptions/BSplineException.h>
#include <bspline/internal/misc.h>
#include <bspline/operators/GenericOperators.h>

namespace bspline::operators {

/*!
 * @brief Operator representation of a Spline.
 *
 * @tparam T The data type of the spline.
 * @tparam order The order of the spline.
 */
template <typename T, size_t order>
class SplineOperator final : public Operator {
 private:
  /*! The spline which this operator represents. */
  Spline<T, order> _s;

 public:
  /*!
   * @brief Constructor constructing a SplineOperator from a Spline.
   *
   * @param s The spline.
   */
  SplineOperator(Spline<T, order> s) : _s(std::move(s)){};

  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return inputOrder + order;
  }

  /*!
   * @brief Applies operator to a Spline.
   *
   * Applies the operator to a set of coefficients (representing a polynomial on
   * one interval).
   *
   * @param input The polynomial coefficients.
   * @param grid The global grid with respect to which the splines are defined.
   * @param intervalIndex The index of the begin of the interval with respect to
   * the global grid.
   * @tparam T The datatype of the spline.
   * @tparam size The size of the array, i. e. the number of coefficients.
   * @throws BSplineException If the spline _s is defined on a grid that is
   * (logically) different from grid.
   * @returns The polyomial coefficients arising from the application of this
   * operator to the input coefficients.
   */
  template <size_t size>
  auto transform(const std::array<T, size> &input, const support::Grid<T> &grid,
                 size_t intervalIndex) const {
    static_assert(size >= 1);
    constexpr size_t OUTPUT_SIZE = outputOrder(size - 1) + 1;

    if (_s.getSupport().getGrid() != grid) {
      throw exceptions::BSplineException(ErrorCode::DIFFERING_GRIDS);
    }

    const auto relativeIndex =
        _s.getSupport().relativeFromAbsolute(intervalIndex);

    auto retVal = internal::make_array<T, OUTPUT_SIZE>(static_cast<T>(0));

    if (relativeIndex) {
      // The interval is part of the Spline's support.
      const auto &coeffs = _s.getCoefficients()[*relativeIndex];

      for (size_t i = 0; i < input.size(); i++) {
        for (size_t j = 0; j < coeffs.size(); j++) {
          retVal[i + j] += input[i] * coeffs[j];
        }
      }
    }
    return retVal;
  }
};
}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_SPLINEOPERATOR_H
