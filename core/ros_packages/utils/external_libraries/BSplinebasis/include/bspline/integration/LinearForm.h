/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_INTEGRATION_LINEARFORM_H
#define BSPLINE_INTEGRATION_LINEARFORM_H

#include <bspline/Spline.h>
#include <bspline/operators/GenericOperators.h>

namespace bspline::integration {

/*!
 * Represents the linear form \f[\left\langle
 * a\right\rangle=\int\limits_{-\infty}^{\infty}\mathrm{d}x~\left[\hat{O}\,\,
 * a(x)\right]\f] with the operator \f$\hat{O}\f$ applied to the spline.
 *
 * @tparam O The type of the operator applied to the spline.
 */
template <typename O,
          std::enable_if_t<operators::is_operator_v<O>, bool> = true>
class LinearForm final {
 private:
  /*! Operator applied to the spline.*/
  O _o;

  /*!
   * Evaluates the linear form on one interval.
   *
   * @param a The coefficients  of the polynomial.
   * @param dxhalf The half width of the interval.
   * @tparam T The datatype of the polynomials.
   * @tparam size The number of coefficients of the polynomial.
   * @returns The value of the linear form on the one interval.
   */
  template <typename T, size_t size>
  static T evaluateInterval(const std::array<T, size> &a, const T &dxhalf) {
    // Use Horner's scheme to evaluate.
    constexpr int endIndex =
        static_cast<int>(size) - static_cast<int>(size % 2 == 0 ? 2 : 1);
    static_assert(endIndex >= 0);

    T result = a.at(endIndex) / static_cast<T>(endIndex + 1);
    const T dxhalf_squared = dxhalf * dxhalf;

    for (int i = endIndex - 2; i >= 0; i -= 2) {
      result = dxhalf_squared * result + a[i] / static_cast<T>(i + 1);
    }
    return static_cast<T>(2) * dxhalf * result;
  }

 public:
  /*!
   * Constructor constructing a LinearForm from an operator.
   *
   * @param o The operator acting on the spline.
   */
  explicit LinearForm(O o) : _o(std::move(o)){};

  /*!
   * Default constructor constructing a LinearForm.
   */
  LinearForm() : _o(O{}){};

  /*!
   * Evaluates the linear form for a particular spline.
   *
   * @param a The  spline.
   * @tparam T The datatype of the splines.
   * @tparam order The order of the spline.
   * @returns The value of the linear form for the given spline.
   */
  template <typename T, size_t order>
  T evaluate(const Spline<T, order> &a) const {
    const size_t nintervals = a.getSupport().numberOfIntervals();

    T result = static_cast<T>(0);

    for (size_t i = 0; i < nintervals; i++) {
      const size_t absIndex = a.getSupport().absoluteFromRelative(i);
      const T dxhalf =
          (a.getSupport()[i + 1] - a.getSupport()[i]) / static_cast<T>(2);

      result +=
          evaluateInterval(_o.transform(a.getCoefficients()[i],
                                        a.getSupport().getGrid(), absIndex),
                           dxhalf);
    }
    return result;
  }

  /*!
   * <b>Alias for LinearForm::evaluate().</b>
   * @copydoc LinearForm::evaluate()
   */
  template <typename T, size_t order>
  T operator()(const Spline<T, order> &a) const {
    return evaluate(a);
  }
};

/*!
 * Deduction guide for a linear form which represents the integral over the
 * spline \f[\left\langle
 * a\right\rangle=\int\limits_{-\infty}^{\infty}\mathrm{d}x~a(x).\f]
 */
LinearForm()->LinearForm<operators::IdentityOperator>;

}  // namespace bspline::integration
#endif  // BSPLINE_INTEGRATION_LINEARFORM_H
