/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_MISC_H
#define BSPLINE_MISC_H

#include <array>

#ifndef BSPLINE_DOXYGEN_IGNORE
/*!
 * The methods in the internal namespace are internal helper method for this
 * library.
 */
namespace bspline::internal {

/*!
 * Creates an std::array<T, size> with all values set to val.
 *
 * @param val Value to initialise the members of the array with.
 * @tparam T Datatype of the array.
 * @tparam size Size of the array.
 * @returns An array filled with the value val.
 */
template <typename T, size_t size>
std::array<T, size> make_array(T val) {
  std::array<T, size> ret;
  ret.fill(val);
  return ret;
}

/*!
 * Adds two vectors, taking into account that one vector might be longer than
 * the other one.
 *
 * @param a First array to be added.
 * @param b Second array to be added.
 * @tparam T Datatype of both arrays.
 * @tparam sizea Size of the first array.
 * @tparam sizeb Size of the second array.
 * @returns An array representing the sum of the two input arrays.
 */
template <typename T, size_t sizea, size_t sizeb>
std::array<T, std::max(sizea, sizeb)> add(const std::array<T, sizea> &a,
                                          const std::array<T, sizeb> &b) {
  if constexpr (sizeb > sizea) {
    return add(b, a);
  } else {
    std::array<T, sizea> ret = a;
    for (size_t i = 0; i < sizeb; i++) {
      ret[i] += b[i];
    }
    return ret;
  }
}

/*!
 * Copies one array into a larger one, filling up the additional members with
 * zeros.
 *
 * @param in Array to be copied.
 * @tparam T Datatype of the arrays.
 * @tparam sizein Size of the input array.
 * @tparam sizeout Size of the output array. Must fulfil sizeout >= sizein.
 * @returns The array.
 */
template <typename T, size_t sizein, size_t sizeout>
std::array<T, sizeout> changearraysize(const std::array<T, sizein> &in) {
  static_assert(sizeout >= sizein,
                "sizeout must be bigger or equal to sizein.");
  if constexpr (sizeout == sizein)
    return in;
  else {
    std::array<T, sizeout> ret;
    for (size_t i = 0; i < sizein; i++) ret[i] = in[i];
    for (size_t i = sizein; i < sizeout; i++) ret[i] = static_cast<T>(0);
    return ret;
  }
}

/*!
 * Normally, for every evaluation of a spline, a binary search for the correct
 * interval is necessary. This method is defined in order to integrate every
 * interval separately during the 1D integration, omitting the necessity for the
 * binary search.
 *
 * @param x The point at which to evaluate the polynomial.
 * @param coeffs The coefficients of the polynomial.
 * @param xm The middlepoint of the interval with respect to which the
 * polynomial coefficients are defined.
 * @tparam T The datatype of the polynomial.
 * @tparam size The size of the coefficient array (i.e. the order of the
 * polynomial plus one).
 */
template <typename T, size_t size>
T evaluateInterval(const T &x, const std::array<T, size> &coeffs, const T &xm) {
  // Use Horner's scheme to evaluate.
  const T dx = x - xm;
  T result = coeffs.back();
  for (auto it = coeffs.rbegin() + 1; it != coeffs.rend(); it++) {
    result = dx * result + (*it);
  }
  return result;
}

/*!
 * Returns the faculty \f$n!\f$.
 *
 * @param n The value to calculate the faculty of.
 * @tparam T The datatype of the return type.
 * @returns The faculty \f$n!\f$.
 */
template <typename T>
T faculty(size_t n) {
  T retVal = static_cast<T>(1);
  for (size_t i = 2; i <= n; i++) {
    retVal *= static_cast<T>(i);
  }
  return retVal;
}

/*!
 * Returns the faculty ratio \f[\frac{counter!}{denominator!}\f].
 *
 * @param counter The faculty of this value is the counter.
 * @param denominator The faculty of this value is the denominator.
 * @tparam T The datatype of the return type.
 * @returns The value of the faculty ratio.
 */
template <typename T>
T facultyRatio(size_t counter, size_t denominator) {
  if (denominator > counter) {
    return static_cast<T>(1) / facultyRatio<T>(denominator, counter);
  } else {
    T retVal = static_cast<T>(1);
    for (size_t i = denominator + 1; i <= counter; i++) {
      retVal *= static_cast<T>(i);
    }
    return retVal;
  }
}

/*!
 * Returns the binomial coefficient \f[\frac{n!}{k!\,(n-k)!]}\f].
 * Returns zero if \f$k > n\f$.
 *
 * @param n The first parameter.
 * @param k The second parameter.
 * @tparam T The datatype of the return type.
 * @returns The value of the binomial coefficient
 */
template <typename T>
T binomialCoefficient(size_t n, size_t k) {
  if (k > n) {
    return static_cast<T>(0);
  }

  const size_t smaller = std::min(k, n - k);
  const size_t larger = std::max(k, n - k);

  return facultyRatio<T>(n, larger) / faculty<T>(smaller);
}

}  // end namespace bspline::internal

#endif  // BSPLINE_DOXYGEN_IGNORE
#endif  // BSPLINE_MISC_H
