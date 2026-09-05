/*
 * This file contains an additional numerical integration routine for the
 * Splines based on the Gauss-Legendre routines in boost/math/quadrature.
 *
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_INTEGRATION_NUMERICAL_H
#define BSPLINE_INTEGRATION_NUMERICAL_H

#include <bspline/Spline.h>
#include <bspline/exceptions/BSplineException.h>
#include <bspline/internal/misc.h>

#include <boost/math/quadrature/gauss.hpp>

namespace bspline::integration {
using namespace boost::math::quadrature;
using bspline::support::Support;
using namespace bspline::exceptions;

/*!
 * Calculates the 1D integral \f[I=\int\limits_{-\infty}^{\infty} \mathrm{d}x~
 * m_1(x)\, f(x)\, m_2(x).\f] The integral is evaluated numerically on each
 * interval using boost's Gauss-Legendre scheme of order ordergl.
 *
 * @param f Callable \f$f(x)\f$ to be multiplied to the integrand.
 * @param m1 First spline \f$m_1(x)\f$.
 * @param m2 Second spline \f$m_2(x)\f$.
 * @tparam ordergl Order of the Gauss-Legendre integration scheme provided by
 * the boost library.
 * @tparam T Datatype of the calculation.
 * @tparam F Type of the callable \f$f(x)\f$.
 * @tparam order1 Order of the spline \f$m_1(x)\f$.
 * @tparam order2 Order of the spline \f$m_2(x)\f$.
 * @throws BSplineException If the two splines are defined on different grids.
 * @returns The value of the integral \f$I\f$.
 */
template <size_t ordergl, typename T, typename F, size_t order1, size_t order2>
T integrate(const F &f, const bspline::Spline<T, order1> &m1,
            const bspline::Spline<T, order2> &m2) {
  // Will also check whether the two grids are equivalent.
  const Support newSupport = m1.getSupport().calcIntersection(m2.getSupport());
  const size_t nintervals = newSupport.numberOfIntervals();

  T result = static_cast<T>(0);
  for (size_t interv = 0; interv < nintervals; interv++) {
    const auto ai = newSupport.absoluteFromRelative(interv);
    const auto m1Index = m1.getSupport().intervalIndexFromAbsolute(ai).value();
    const auto m2Index = m2.getSupport().intervalIndexFromAbsolute(ai).value();

    const T &xstart = m1.getSupport().at(m1Index);
    const T &xend = m1.getSupport().at(m1Index + 1);
    const T xm = (xstart + xend) / static_cast<T>(2);
    const auto &c1 = m1.getCoefficients().at(m1Index);
    const auto &c2 = m2.getCoefficients().at(m2Index);
    result += gauss<T, ordergl>::integrate(
        [&c1, &c2, &xm, &f](const T &x) {
          using namespace bspline::internal;
          return f(x) * evaluateInterval(x, c1, xm) *
                 evaluateInterval(x, c2, xm);
        },
        xstart, xend);
  }
  return result;
}
}  // namespace bspline::integration
#endif  // BSPLINE_INTEGRATION_NUMERICAL_H
