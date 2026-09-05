#ifndef BSPLINE_INTEGRATION_ANALYTICAL_H
#define BSPLINE_INTEGRATION_ANALYTICAL_H
/*
 * This file contains additional integration routines computing a few special
 * integrals for the splines defined in spline-template.h. The integrals are
 * computed from analytical formulas and introduce no additional dependencies.
 *
 * All methods accessing two splines assume that these splines are defined on
 * the same grid (i.e. that both splines have the same interval boundaries
 * within the intersection of their respective supports).
 *
 * @deprecated All contents of this file should be considered as deprecated.
 *
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/Spline.h>
#include <bspline/exceptions/BSplineException.h>

/*!
 * Namespace containing the code for numerical and analytical integration of
 * splines.
 */
namespace bspline::integration {

using bspline::Spline;
using namespace bspline::exceptions;

namespace internal {
/*!
 * Efficient integer power for arbitrary type.
 *
 * @param a Basis.
 * @param n Integer exponent.
 * @tparam T Datatype.
 * @deprecated Obsolete.
 */
template <typename T>
[[deprecated]] T pow(T a, size_t n) {
  size_t power_of_2 = 1;
  T ret = static_cast<T>(1);
  while (power_of_2 <= n) {
    // If the bit corresponding to power_of_2 is set in n,
    // Multiply return value by a = a0^power_of_2 (a0 being the input a)
    if ((power_of_2 & n) == power_of_2) {
      ret *= a;
    }
    power_of_2 *= 2;
    a *= a;
  }
  return ret;
}

/*!
 * Performs an integral over splines m1 and m2 on one interval. The type of the
 * integral is defined by the integration function f.
 *
 * @param f Function defining the type of integral
 * @param coeffsa First spline's coefficients on the interval of interest.
 * @param coeffsb Second spline's coefficients on the interval of interest.
 * @param x0 Beginning of the interval.
 * @param x1 End of the interval.
 * @tparam T Datatype of both splines.
 * @tparam F Type of Function object f.
 * @tparam sizea Number of coefficients per interval for the first spline.
 * @tparam sizeb Number of coefficients per interval for the second spline.
 * @deprecated Obsolete.
 */
template <typename T, typename F, size_t sizea, size_t sizeb>
[[deprecated]] T integrateIntervalAnalytically(
    F f, const std::array<T, sizea> &coeffsa,
    const std::array<T, sizeb> &coeffsb, const T &x0, const T &x1) {
  T result = static_cast<T>(0);
  const T dxhalf = (x1 - x0) / static_cast<T>(2);
  const T xm = (x1 + x0) / static_cast<T>(2);
  for (size_t i = 0; i < sizea; i++) {
    for (size_t j = 0; j < sizeb; j++) {
      result += f(i, j, coeffsa[i], coeffsb[j], dxhalf, xm);
    }
  }
  return result;
}

/*!
 * Performs an integral over the common support of splines m1 and m2. The type
 * of the integral is defined by the integration function f.
 *
 * @param f Function defining the type of the integral.
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline.
 * @tparam order2 Order of the second spline.
 * @deprecated Obsolete.
 */
template <typename T, typename F, size_t order1, size_t order2>
[[deprecated]] T helperAnalyticIntegration(
    F f, const bspline::Spline<T, order1> &m1,
    const bspline::Spline<T, order2> &m2) {
  if (!m1.getSupport().hasSameGrid(m2.getSupport())) {
    throw BSplineException(ErrorCode::DIFFERING_GRIDS);
  }

  Support integrandSupport = m1.getSupport().calcIntersection(m2.getSupport());
  const size_t nintervals = integrandSupport.numberOfIntervals();

  if (nintervals == 0) return static_cast<T>(0);  // no overlap

  T result = static_cast<T>(0);

  for (size_t interv = 0; interv < nintervals; interv++) {
    const auto absIndex = integrandSupport.absoluteFromRelative(interv);

    const auto m1Index = m1.getSupport().relativeFromAbsolute(absIndex).value();
    const auto m2Index = m2.getSupport().relativeFromAbsolute(absIndex).value();

    result += integrateIntervalAnalytically<T, F, order1 + 1, order2 + 1>(
        f, m1.getCoefficients()[m1Index], m2.getCoefficients()[m2Index],
        m1.getSupport()[m1Index], m1.getSupport()[m1Index + 1]);
  }
  return result;
}

}  // end namespace internal

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m(x). Calculated
 * analytically.
 *
 * @param m Spline m(x) to be integrated.
 * @tparam T Datatype of the spline m.
 * @tparam order Order of the spline m.
 * @deprecated Use bspline::integration::LinearForm instead.
 */
template <typename T, size_t order>
[[deprecated]] T integrate(const Spline<T, order> &m) {
  T retval = static_cast<T>(0);
  const auto &ints = m.getSupport();
  for (size_t i = 0; i + 1 < ints.size(); i++) {
    const T &start = ints[i];
    const T &end = ints[i + 1];
    T pot = (end - start) /
            static_cast<T>(2);  // power of dxhalf, initialised to dxhalf^1
    const T dxhalf_squared = pot * pot;
    const auto &coeffs = m.getCoefficients()[i];
    for (size_t index = 0; index < order + 1; index += 2) {
      retval +=
          static_cast<T>(2) * coeffs[index] * pot / static_cast<T>(index + 1);
      pot *= dxhalf_squared;
    }
  }
  return retval;
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x) m2(x).
 * Calculated analytically.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::ScalarProduct instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T overlap(const Spline<T, order1> &m1,
                         const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf,
                               [[maybe_unused]] const T &xm) {
    if ((i + j + 1) % 2 == 0) return static_cast<T>(0);
    return static_cast<T>(2) * coeffa * coeffb *
           internal::pow<T>(dxhalf, i + j + 1) / static_cast<T>(i + j + 1);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x) x m2(x).
 * Calculated analytically.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_x(const Spline<T, order1> &m1,
                             const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf, const T &xm) {
    if ((i + j + 1) % 2 == 1)
      return static_cast<T>(2) * coeffa * coeffb * xm *
             internal::pow<T>(dxhalf, i + j + 1) / static_cast<T>(i + j + 1);
    else
      return static_cast<T>(2) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j + 2) / static_cast<T>(i + j + 2);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x) x^2 m2(x).
 * Calculated analytically.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_x2(const Spline<T, order1> &m1,
                              const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf, const T &xm) {
    if ((i + j + 2) % 2 == 1)
      return static_cast<T>(4) * coeffa * coeffb * xm *
             internal::pow<T>(dxhalf, i + j + 2) / static_cast<T>(i + j + 2);
    else
      return static_cast<T>(2) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j + 1) *
             (dxhalf * dxhalf / static_cast<T>(i + j + 3) +
              xm * xm / static_cast<T>(i + j + 1));
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x)
 * \frac{\partial}{\partial x} m2(x). Calculated analytically. Assumes m2(x) is
 * continous.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_dx(const Spline<T, order1> &m1,
                              const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf,
                               [[maybe_unused]] const T &xm) {
    if (j == 0 || (i + j) % 2 == 0)
      return static_cast<T>(0);
    else
      return static_cast<T>(2 * j) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j) / static_cast<T>(i + j);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx x m1(x)
 * \frac{\partial}{\partial x} m2(x). Calculated analytically. Assumes m2(x) is
 * continous.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_x_dx(const Spline<T, order1> &m1,
                                const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf, const T &xm) {
    if (j == 0)
      return static_cast<T>(0);
    else if ((i + j) % 2 == 0)
      return static_cast<T>(2 * j) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j + 1) / static_cast<T>(i + j + 1);
    else
      return static_cast<T>(2 * j) * xm * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j) / static_cast<T>(i + j);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x)
 \frac{\partial^2}{\partial x^2} m2(x). Calculated analytically. Assumes m2(x)
 is at least once continously differentiable.

 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_dx2(const Spline<T, order1> &m1,
                               const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf,
                               [[maybe_unused]] const T &xm) {
    if (j < 2 || (i + j) % 2 == 1) return static_cast<T>(0);
    return static_cast<T>(2 * j * (j - 1)) * coeffa * coeffb *
           internal::pow<T>(dxhalf, i + j - 1) / static_cast<T>(i + j - 1);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x) x
 * \frac{\partial^2}{\partial x^2} m2(x). Calculated analytically. Assumes m2(x)
 * is at least once continously differentiable.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_x_dx2(const Spline<T, order1> &m1,
                                 const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf, const T &xm) {
    if (j < 2)
      return static_cast<T>(0);
    else if ((i + j) % 2 == 1)
      return static_cast<T>(2 * j * (j - 1)) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j) / static_cast<T>(i + j);
    else
      return static_cast<T>(2 * j * (j - 1)) * coeffa * coeffb * xm *
             internal::pow<T>(dxhalf, i + j - 1) / static_cast<T>(i + j - 1);
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

/*!
 * Returns the integral \\int\\limits_{-\\infty}^{\\infty} dx m1(x) x^2
 * \frac{\partial^2}{\partial x^2} m2(x). Calculated analytically. Assumes m2(x)
 * is at least once continously differentiable.
 *
 * @param m1 First spline.
 * @param m2 Second spline.
 * @tparam T Datatype of both splines.
 * @tparam order1 Order of the first spline m1.
 * @tparam order2 Order of the second spline m2.
 * @deprecated Use bspline::integration::BilinearForm instead.
 */
template <typename T, size_t order1, size_t order2>
[[deprecated]] T integrate_x2_dx2(const Spline<T, order1> &m1,
                                  const Spline<T, order2> &m2) {
  static constexpr auto f = [](size_t i, size_t j, const T &coeffa,
                               const T &coeffb, const T &dxhalf, const T &xm) {
    if (j < 2)
      return static_cast<T>(0);
    else if ((i + j) % 2 == 1)
      return static_cast<T>(4 * j * (j - 1)) * xm * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j) / static_cast<T>(i + j);
    else
      return static_cast<T>(2 * j * (j - 1)) * coeffa * coeffb *
             internal::pow<T>(dxhalf, i + j - 1) *
             (dxhalf * dxhalf / static_cast<T>(i + j + 1) +
              xm * xm / static_cast<T>(i + j - 1));
  };
  return internal::helperAnalyticIntegration(f, m1, m2);
}

}  // end  namespace bspline::integration
#endif  // BSPLINE_INTEGRATION_ANALYTICAL_H
