/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_COMPOUNDOPERATORS_H
#define BSPLINE_OPERATORS_COMPOUNDOPERATORS_H

#include <bspline/operators/GenericOperators.h>

namespace bspline::operators {
/*!
 * @brief Represents the product of two operators.
 *
 * @tparam O1 The type of the first (left) operator.
 * @tparam O2 The type of the second (right) operator.
 */
template <typename O1, typename O2,
          std::enable_if_t<are_operators_v<O1, O2>, bool> = true>
class OperatorProduct final : public Operator {
 private:
  /*! The first (left) operator.*/
  O1 _o1;
  /*! The second (right) operator.*/
  O2 _o2;

 public:
  /*!
   * @brief Creates an OperatorProduct from two operators.
   *
   * @param o1 The first (left) operator.
   * @param o2 The second (right) ooperator.
   */
  OperatorProduct(O1 o1, O2 o2) : _o1(std::move(o1)), _o2(std::move(o2)){};

  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return O1::outputOrder(O2::outputOrder(inputOrder));
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
    return _o1.transform(_o2.transform(input, grid, intervalIndex), grid,
                         intervalIndex);
  }
};

/*!
 * @brief Operator multiplication operator.
 *
 * The multiplication operator for two operators, returning an OperatorProduct.
 *
 * @param o1 The first (left) operator.
 * @param o2 The second (right) operator.
 * @tparam O1 The type of the first (left) operator.
 * @tparam O2 The type of the second (right) operator.
 */
template <typename O1, typename O2,
          std::enable_if_t<are_operators_v<O1, O2>, bool> = true>
OperatorProduct<O1, O2> operator*(O1 &&o1, O2 &&o2) {
  return OperatorProduct(std::forward<O1>(o1), std::forward<O2>(o2));
}

// ######################### OperatorProduct #############################
// #######################################################################

// #######################################################################
// ########################### OperatorSum ###############################

/*! Indicates which operation the OperatorSum is supposed to perform. */
enum class AdditionOperation {
  /*! Perform addition. */
  ADDITION,
  /*! Perform subtraction. */
  SUBTRACTION,
};

/*!
 * @brief Validates operation.
 *
 * Checks whether the operation is a valid oeration for OperatorSum.
 *
 * @tparam operation The operation type to be checked.
 */
template <AdditionOperation operation>
inline constexpr bool is_valid_operation_v = (operation ==
                                              AdditionOperation::ADDITION) ||
                                             (operation ==
                                              AdditionOperation::SUBTRACTION);

/*!
 * @brief Operator sum.
 * Represents the sum or difference of two operators.
 *
 * @tparam O1 The type of the first operator.
 * @tparam O2 The type of the second operator.
 * @tparam operation Indicates whether the operators shall be added or
 * subtracted.
 */
template <
    typename O1, typename O2, AdditionOperation operation,
    std::enable_if_t<are_operators_v<O1, O2> && is_valid_operation_v<operation>,
                     bool> = true>
class OperatorSum final : public Operator {
 private:
  /*! The first operator.*/
  O1 _o1;
  /*! The second operator.*/
  O2 _o2;

  /*!
   * @brief Adds arrays of differing size.
   *
   * Adds the smaller array to the larger array in place and returns the larger
   * array.
   *
   * @param a The first array.
   * @param b The second array.
   * @tparam T The datatype of the arrays.
   * @tparam sizea The size of a.
   * @tparam sizeb The size of b.
   * @returns The sum of the two input arrays.
   */
  template <typename T, size_t sizea, size_t sizeb>
  static std::array<T, std::max(sizea, sizeb)> &add(std::array<T, sizea> &a,
                                                    std::array<T, sizeb> &b) {
    if constexpr (sizeb > sizea) {
      return add(b, a);
    } else {
      for (size_t i = 0; i < sizeb; i++) {
        a[i] += b[i];
      }
      return a;
    }
  }

 public:
  /*!
   * @brief Creates an OperatorSum from two operators.
   *
   * @param o1 The first operator.
   * @param o2 The second ooperator.
   */
  OperatorSum(O1 o1, O2 o2) : _o1(std::move(o1)), _o2(std::move(o2)){};

  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return std::max(O1::outputOrder(inputOrder), O2::outputOrder(inputOrder));
  }

  /*!
   * @brief Applies opertor to interval.
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
    auto a = _o1.transform(input, grid, intervalIndex);
    auto b = _o2.transform(input, grid, intervalIndex);

    // Negate b if subtraction is requested.
    if constexpr (operation == AdditionOperation::SUBTRACTION) {
      for (T &el : b) {
        el *= static_cast<T>(-1);
      }
    }

    return add(a, b);
  }
};

/*!
 * @brief Operator sum.
 *
 * The addition operator for two operators, returning an OperatorSum.
 *
 * @param o1 The first operator.
 * @param o2 The second operator.
 * @tparam O1 The type of the first operator.
 * @tparam O2 The type of the second operator.
 * @returns The sum of the two input operators.
 */
template <typename O1, typename O2,
          std::enable_if_t<are_operators_v<O1, O2>, bool> = true>
auto operator+(O1 &&o1, O2 &&o2) {
  return OperatorSum<O1, O2, AdditionOperation::ADDITION>(std::forward<O1>(o1),
                                                          std::forward<O2>(o2));
}

/*!
 * @brief Operator difference.
 *
 * The subtraction operator for two operators, returning an OperatorSum.
 *
 * @param o1 The first operator.
 * @param o2 The second operator.
 * @tparam O1 The type of the first operator.
 * @tparam O2 The type of the second operator.
 * @returns the difference of the two input operators.
 */
template <typename O1, typename O2,
          std::enable_if_t<are_operators_v<O1, O2>, bool> = true>
auto operator-(O1 &&o1, O2 &&o2) {
  return OperatorSum<O1, O2, AdditionOperation::SUBTRACTION>(
      std::forward<O1>(o1), std::forward<O2>(o2));
}

}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_COMPOUNDOPERATORS_H
