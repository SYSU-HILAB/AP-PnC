/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_OPERATORS_GENERICOPERATORS_H
#define BSPLINE_OPERATORS_GENERICOPERATORS_H

#include <bspline/Spline.h>

/*!
 * @brief Operator definitions.
 */
namespace bspline::operators {

/*!
 * @brief Marker interface for operators.
 *
 * All proper operators must derive from this interface.
 */
class Operator {};

/*!
 * @brief Checks whether O is an operator.
 *
 * Indicates whether the template parameter is an operator
 * type.
 *
 * @tparam O Template parameter.
 */
template <typename O>
inline constexpr bool is_operator_v =
    std::is_base_of_v<Operator, std::remove_cv_t<std::remove_reference_t<O>>>;

/*!
 * @brief Checks whether O1 and O2 are operators.
 *
 * Indicates whether both template parameters are operator
 * types.
 *
 * @tparam O1 First template parameter.
 * @tparam O2 Second template parameter.
 */
template <typename O1, typename O2>
inline constexpr bool are_operators_v = is_operator_v<O1> &&is_operator_v<O2>;

/*!
 * @brief Applies operator to spline.
 *
 * Helper method that applies an operator to a spline based on the
 * transformation of the coefficients on a single interval.
 *
 * @param op The operator to apply to the spline.
 * @param spline The spline to apply the operator to.
 * @tparam T The datatype of the input and output splines.
 * @tparam order The order of the input spline.
 * @tparam O The type of the operator.
 * @returns The spline resulting from the application of this operator to the
 * spline.
 */
template <typename T, size_t order, typename O,
          std::enable_if_t<is_operator_v<O>, bool> = true>
auto transformSpline(const O &op, const Spline<T, order> &spline) {
  constexpr size_t OUTPUT_SIZE = O::outputOrder(order) + 1;

  const auto &oldCoefficients = spline.getCoefficients();

  std::vector<std::array<T, OUTPUT_SIZE>> newCoefficients;
  newCoefficients.reserve(oldCoefficients.size());

  const auto &support = spline.getSupport();

  for (size_t i = 0; i < oldCoefficients.size(); i++) {
    const size_t absIndex = support.absoluteFromRelative(i);
    newCoefficients.push_back(
        op.transform(oldCoefficients[i], support.getGrid(), absIndex));
  }

  return Spline(spline.getSupport(), std::move(newCoefficients));
}

// ################## Generic Operator Definitions #######################
// #######################################################################

// #######################################################################
// ########################## IdentityOperator ##############################

/*!
 * @brief Multiplicative identity operator.
 *
 * Represents the identity operator.
 */
class IdentityOperator final : public Operator {
 public:
  /*!
   * @brief Order of the resulting Spline.
   *
   * Returns the order of the output Spline for a given input order.
   *
   * @param inputOrder the order of the input Spline.
   * @returns The output Spline-order for a given input input order.
   */
  static constexpr size_t outputOrder(size_t inputOrder) { return inputOrder; }

  /*!
   * @brief Applies operator on one interval.
   *
   * Applies the operator to a set of coefficients (representing a polynomial on
   * one interval).
   *
   * @param input The polynomial coefficients.
   * @param grid The global grid with respect to which the splines are defined.
   * @param intervalIndex The index of the begin of the interval with respect to
   * the global grid.
   * @tparam T The datatype of the coefficients.
   * @tparam size The size of the array, i. e. the number of coefficients.
   * @returns The polyomial coefficients arising from the application of the
   * operator to the input coefficients.
   */
  template <typename T, size_t size>
  std::array<T, size> transform(const std::array<T, size> &input,
                                [[maybe_unused]] const support::Grid<T> &grid,
                                [[maybe_unused]] size_t intervalIndex) const {
    return input;
  }
};

/*!
 * @brief Applies an operator to a spline.
 *
 * @param o The operator.
 * @param s The spline.
 * @tparam O The type of the operator.
 * @tparam S The type of the spline.
 * @returns The spline resulting from the application of the operator to the
 * spline.
 */
template <typename O, typename S,
          std::enable_if_t<is_operator_v<O> && is_spline_v<S>, bool> = true>
auto operator*(const O &o, const S &s) {
  return transformSpline(o, s);
}
}  // namespace bspline::operators
#endif  // BSPLINE_OPERATORS_GENERICOPERATORS_H
