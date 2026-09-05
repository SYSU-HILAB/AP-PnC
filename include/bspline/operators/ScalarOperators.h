/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_SCALAROPERATORS_H
#define BSPLINE_OPERATORS_SCALAROPERATORS_H

#include <bspline/Spline.h>
#include <bspline/operators/CompoundOperators.h>

/*!
 * Operator definitions.
 */
namespace bspline::operators {

/*!
 * @brief Validates ScalarOperator template parameters.
 *
 * Indicates whether the argument types may be arguments of a scalar
 * multiplication (i.e. a scalar multiplied with an operator).
 *
 * @tparam T Scalar type.
 * @tparam O Operator type.
 */
template <typename T, typename O>
inline constexpr bool are_scalar_multiplication_types_v =
    !is_spline_v<T> && !is_operator_v<T> && is_operator_v<O>;

/*!
 * @brief Multiplication operator for Operator and scalar.
 *
 * Represents the multiplication of an operator with a scalar.
 *
 * @tparam S type of the scalar.
 * @tparam O type of the operator to be multiplied with the scalar.
 */
template <typename S, typename O,
          std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> =
              true>
class ScalarMultiplication final : public Operator {
 private:
  /*! The scalar to multiply the operator with. */
  S _s;
  /*! The operator to be multiplied. */
  O _o;

 public:
  /*!
   * @brief Multiplication of scalar and Operator.
   *
   * Constructor constructing an ScalarMultiplication from a scalar and an
   * operator.
   *
   * @param s The scalar to be multiplied.
   * @param o The operator to be multiplied.
   */
  ScalarMultiplication(S s, O o) : _s(std::move(s)), _o(std::move(o)){};

  /*!
   * @brief Multiplication of scalar and a default constructed Operator.
   *
   * Constructor constructing an ScalarMultiplication from a scalar alone.
   *
   * @param s The scalar to be multiplied.
   */
  ScalarMultiplication(S s) : _s(s), _o(O{}){};

  /*!
   * @brief Returns the order of the output spline for a given input order.
   *
   * @param inputOrder the order of the input spline.
   * @returns The output spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) {
    return O::outputOrder(inputOrder);
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
   * @tparam T The datatype of the spline.
   * @tparam size The size of the array, i. e. the number of coefficients.
   * @returns The polyomial coefficients arising from the application of this
   * operator to the input coefficients.
   */
  template <typename T, size_t size>
  auto transform(const std::array<T, size> &input, const support::Grid<T> &grid,
                 size_t intervalIndex) const {
    auto a = _o.transform(input, grid, intervalIndex);

    // Multiply a.
    for (T &el : a) {
      el *= static_cast<T>(_s);
    }
    return a;
  }
};

/*!
 * @brief ScalarMultiplication with default constructed operator.
 *
 * Deduction guide for a ScalarMultiplication constructed from a scalar alone.
 *
 * @tparam S The type of the scalar.
 */
template <typename S>
ScalarMultiplication(S s) -> ScalarMultiplication<S, IdentityOperator>;

/*!
 * @brief Scalar-Opertator multiplication.
 *
 * Deduction guide for a ScalarMultiplication of a scalar and an Operator.
 *
 * @param s The scalar to be multiplied.
 * @param o The operator to be multiplied.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator to be multiplied.
 * @returns The scaled operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
ScalarMultiplication<S, O> operator*(const S &s, O &&o) {
  return ScalarMultiplication(s, std::forward<O>(o));
}

/*!
 * @brief The scalar multiplication operator for an operator.
 *
 * @param o The operator to be multiplied.
 * @param s The scalar to be multiplied.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator to be multiplied.
 * @returns The scaled operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
ScalarMultiplication<S, O> operator*(O &&o, const S &s) {
  return ScalarMultiplication(s, std::forward<O>(o));
}

/*!
 * @brief The scalar division operator for an operator.
 *
 * @param o The operator to be divided.
 * @param s The divisor.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator to be divided.
 * @returns The scaled operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
ScalarMultiplication<S, O> operator/(O &&o, const S &s) {
  return ScalarMultiplication(static_cast<S>(1) / s, std::forward<O>(o));
}

/*!
 * The scalar addition operator for an operator.
 *
 * @param o The operator to be added.
 * @param s The scalar to be added.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator.
 * @returns The shifted operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
auto operator+(O &&o, const S &s) {
  return std::forward<O>(o) + ScalarMultiplication{s};
}

/*!
 * @brief The scalar addition operator for an operator.
 *
 * @param s The scalar to be added.
 * @param o The operator to be added.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator.
 * @returns The shifted operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
auto operator+(const S &s, O &&o) {
  return ScalarMultiplication{s} + std::forward<O>(o);
}

/*!
 * @brief The scalar subtraction operator for an operator.
 *
 * @param o The operator.
 * @param s The scalar to be subtracted.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator.
 * @returns The shifted operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
auto operator-(O &&o, const S &s) {
  return std::forward<O>(o) - ScalarMultiplication{s};
}

/*!
 * The scalar subtraction operator for an operator.
 *
 * @param s The scalar.
 * @param o The operator to be subtracted.
 * @tparam S The type of the scalar.
 * @tparam O The type of the operator.
 * @returns The shifted and inverted operator.
 */
template <
    typename S, typename O,
    std::enable_if_t<are_scalar_multiplication_types_v<S, O>, bool> = true>
auto operator-(const S &s, O &&o) {
  return ScalarMultiplication{s} - std::forward<O>(o);
}

/*!
 * @brief Unitary minus operator.
 *
 * The unitary minus operator for an operator. Returns an
 * ScalarMultiplication<int, O>.
 *
 * @param o The operator to be negated.
 * @tparam O The type of the operator to be negated.
 * @returns The inverted operator.
 */
template <typename O, std::enable_if_t<is_operator_v<O>, bool> = true>
ScalarMultiplication<int, O> operator-(O &&o) {
  return ScalarMultiplication<int, O>(-1, std::forward<O>(o));
}
}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_SCALAROPERATORS_H
