/*
 * template<typename T, size_t order>
 * class Spline
 *
 * Represents a spline of dataytype T and order order. The datatype has to
 * fulfill the following requirements:
 *   - comparisons <, <=, >, >=, == and != have to be implemented.
 *   - arithmetic operators + - * /  += -= *= /= have to be implemented
 *   - A pathway must exist, such that integer values of type T can be
 * constructed via static_cast<T>(int) (e. g. via a constructor taking an int).
 * BSplines can be generated via the method generateBspline(...).
 *
 * All methods accessing two splines assume that these splines are defined on
 * the same grid (i.e. that both splines have the same interval boundaries
 * within the intersection of their respective supports). This may also cause
 * problems when splines are constructed by adding up multiple splines. To be
 * safe, make sure that the supports of two splines being added overlap at least
 * in one grid point.
 *
 *
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_SPLINE_H
#define BSPLINE_SPLINE_H

#include <bspline/exceptions/BSplineException.h>
#include <bspline/internal/misc.h>
#include <bspline/support/Support.h>

#include <algorithm>
#include <array>
#include <optional>
#include <type_traits>
#include <vector>

/*!
 * Main namespace for this library.
 */
namespace bspline {

using namespace support;
using namespace bspline::exceptions;

/*!
 *
 * Spline class representing spline of datatype T and order order.
 * The coefficients of the spline are defined with respect to the center point
 * xm of each interval.
 *
 * @brief The central Spline class of the library.
 * @tparam T Datatype of the spline.
 * @tparam order Order of the spline.
 */
template <typename T, size_t order>
class Spline final {
 private:
  /*! Number of coefficients per interval. */
  static constexpr size_t ARRAY_SIZE = order + 1;
  /*! The support of this spline. */
  Support<T> _support;
  /*! Coefficients of the polynomials on each interval. */
  std::vector<std::array<T, ARRAY_SIZE>> _coefficients;

  /*!
   * Finds the interval in which x lies by binary search. Used during the
   * evaluation of the spline.
   *
   * @brief Finds the index of x in the spline's support.
   * @param x Point, whose interval will be searched.
   * @return The index corresponding to the beginning of the interval which
   * contains x or -1 if x is not part of the spline's support.
   */
  std::optional<size_t> findInterval(const T &x) const {
    if (_support.size() < 2 || x > _support.back() || x < _support.front())
      return std::nullopt;  // x is not part of the spline's support

    const auto begin = _support.begin();
    const auto it = std::lower_bound(begin, _support.end(), x);

    // Type of intervalEndIndex is signed.
    const auto intervalEndIndex = std::distance(begin, it);
    return std::max<decltype(intervalEndIndex)>(0, intervalEndIndex - 1);
  };

  /**
   * @brief Performs consistency checks on the provided data.
   *
   * @param support The support to check.
   * @param coefficients The coefficients to check.
   * @throws BSplineException if the provided data do not represent a valid
   * internal state of a spline.
   */
  void checkValidity(
      const Support<T> &support,
      const std::vector<std::array<T, ARRAY_SIZE>> &coefficients) const {
    const bool isValid =
        (!support.containsIntervals() && coefficients.size() == 0) ||
        (support.size() >= 2 &&
         coefficients.size() == support.numberOfIntervals());
    if (!isValid) throw BSplineException(ErrorCode::INCONSISTENT_DATA);
  };

  /**
   * @brief Performs consistency checks on this Spline object
   *
   * @throws BSplineException if this object is not in a valid state.
   */
  void checkValidity() const { checkValidity(_support, _coefficients); };

  /*!
   * @brief Resets the data of the spline and performs sanity checks.
   *
   * @param intervals The grid points representing the intervals on which the
   * spline is defined.
   * @param coefficients Polynomial coefficients on each of the intervals.
   */
  void setData(Support<T> support,
               std::vector<std::array<T, ARRAY_SIZE>> coefficients) {
    checkValidity(support, coefficients);
    _support = std::move(support);
    _coefficients = std::move(coefficients);
  };

 public:
  /*!
   * @brief Provides acces to the data type T of the spline.
   */
  using data_type = T;

  /*!
   * @brief Provides access to the order of the spline.
   */
  static constexpr size_t spline_order = order;

  /*!
   * @brief Constructor setting the data. Performs sanity checks.
   *
   * @param support The spline's support.
   * @param coefficients Polynomial coefficients of the spline on each interval.
   */
  Spline(Support<T> support,
         std::vector<std::array<T, ARRAY_SIZE>> coefficients)
      : _support(std::move(support)), _coefficients(std::move(coefficients)) {
    checkValidity(_support, _coefficients);
  };

  /**
   * @brief Constructs an empty spline on the global grid.
   *
   * @param grid The global grid.
   */
  explicit Spline(Grid<T> grid)
      : Spline(Support<T>::createEmpty(std::move(grid)), {}){};

  /*!
   * @brief Returns the spline's support.
   * @returns This spline's support.
   */
  const Support<T> &getSupport() const noexcept {
    DURING_TEST_CHECK_VALIDITY();
    return _support;
  };

  /*!
   * Returns the polynomial coefficients of the spline for each interval.
   *
   * @brief The spline's polynomial coefficients.
   * @returns A refrence to a vector of arrays containing this spline's
   * polynomial coefficients.
   */
  const std::vector<std::array<T, ARRAY_SIZE>> &getCoefficients()
      const noexcept {
    DURING_TEST_CHECK_VALIDITY();
    return _coefficients;
  };

  /*!
   * @brief Evaluates the spline at point x.
   *
   * @param x Point at which to evaluate the spline. If x is outside of the
   * support of the spline, zero is returned.
   * @returns The value of the spline at point x. If x is outside the spline's
   * support, zero is returned.
   */
  T operator()(const T &x) const {
    DURING_TEST_CHECK_VALIDITY();
    const auto intervalIndex = findInterval(x);

    if (!intervalIndex) return static_cast<T>(0);

    const T xm = (_support[*intervalIndex + 1] + _support[*intervalIndex]) /
                 static_cast<T>(2);

    return internal::evaluateInterval(x, _coefficients[*intervalIndex], xm);
  };

  /*!
   * Returns the beginning of the support of this spline. If the spline is
   * empty, an exception is thrown.
   *
   * @brief Returns the begin of the spline's support.
   * @throws BSplineException If the spline's support is empty.
   * @returns The begin of the spline's support.
   */
  const T &front() const {
    DURING_TEST_CHECK_VALIDITY();
    return _support.front();
  };

  /*!
   * Returns the end of the support of this spline. If the spline is empty, an
   * exception is thrown.
   *
   * @brief Returns the end of the spline's support.
   * @throws BSplineException If the spline's support is empty.
   * @returns The end of the spline's support.
   */
  const T &back() const {
    DURING_TEST_CHECK_VALIDITY();
    return _support.back();
  };

  /*!
   * @brief Checks whether the supports of the two splines overlap.
   *
   * @param m2 Other spline against which to check.
   * @tparam order2 Order of spline m2.
   * @returns True if the intersection of the supports of the two splines is not
   * empty or point-like.
   */
  template <size_t order2>
  bool checkOverlap(const Spline<T, order2> &m2) const {
    DURING_TEST_CHECK_VALIDITY();
    if (!_support.containsIntervals() || !m2.getSupport().containsIntervals())
      return false;
    const bool isNotOverlapping = m2.getSupport().back() <= _support.front() ||
                                  m2.getSupport().front() >= _support.back();
    return !isNotOverlapping;
  }

  /*!
   * Checks whether this spline returns zero for all x. Can be the case, either
   * if the support contains no intervals (i.e. the vector intervals is empty)
   * or if all coefficients are zero.
   *
   * @brief Checks whether this spline is a zero over the whole number line.
   * @returns True if the evaluation of the spline returns zero for all input
   * values.
   */
  bool isZero() const {
    DURING_TEST_CHECK_VALIDITY();
    static const T ZERO = static_cast<T>(0);
    if (!_support.containsIntervals()) return true;
    for (const auto &cs : _coefficients) {
      for (const auto &c : cs) {
        if (c != ZERO) return false;
      }
    }
    return true;
  };

  //######################## Operator definitions ########################
  //######################################################################

  /*!
   * Scalar-division operator. Divides this spline by the scalar d.
   *
   * @brief Scalar-division operator.
   * @param d Scalar by which to divide this spline.
   * @returns A new, scaled spline.
   */
  Spline<T, order> operator/(const T &d) const {
    DURING_TEST_CHECK_VALIDITY();
    return (*this) * (static_cast<T>(1) / d);
  };

  /*!
   * Scalar-multiplication operator. Multiplies this spline with the scalar d.
   *
   * @brief Scalar-multiplication operator.
   * @param d Scalar by which to multiply this spline.
   * @returns A new, scaled spline.
   */
  Spline<T, order> operator*(const T &d) const {
    DURING_TEST_CHECK_VALIDITY();
    Spline<T, order> ret(*this);
    for (auto &cs : ret._coefficients) {
      for (auto &c : cs) {
        c *= d;
      }
    }
    return ret;
  };

  /*!
   * In-place scalar-multiplication operator. Multiplies this spline with the
   * scalar d in-place.
   *
   * @brief In-place scalar-multiplication operator.
   * @param d Scalar by which to multiply this spline.
   * @returns A reference to this spline.
   */
  Spline<T, order> &operator*=(const T &d) {
    DURING_TEST_CHECK_VALIDITY();
    for (auto &cs : _coefficients) {
      for (auto &c : cs) {
        c *= d;
      }
    }
    return *this;
  };

  /*!
   * In-place scalar-division operator. Divides this spline by the scalar d
   * in-place.
   *
   * @brief In-place scalar-division operator.
   * @param d Scalar by which to divide this spline.
   * @returns A reference to this spline.
   */
  Spline<T, order> &operator/=(const T &d) {
    DURING_TEST_CHECK_VALIDITY();
    (*this) *= (static_cast<T>(1) / d);
    return *this;
  };

  /*!
   * @brief Unary minus operator.
   * @returns A new, scaled spline.
   */
  Spline<T, order> operator-() const {
    DURING_TEST_CHECK_VALIDITY();
    return (*this) * static_cast<T>(-1);
  };

  /*!
   * Copy assign of spline to this spline object. The operation is only well
   * defined if the order of the spline to be assigned is lower than or equal to
   * the order of this spline object.
   *
   * @brief Copy assign operator.
   * @param a Spline to be assigned.
   * @tparam ordera Order of spline a.
   * @returns A reference to this spline.
   */
  template <size_t ordera>
  Spline<T, order> &operator=(const Spline<T, ordera> &a) {
    DURING_TEST_CHECK_VALIDITY();
    // The case ordera == order should be handled by the default assignment
    // operator which is automatically generated.
    static_assert(
        ordera < order,
        "The assignment operator is only defined if the order of the rhs "
        "spline is lower than or equal to that of the lhs spline.");

    std::vector<std::array<T, ARRAY_SIZE>> ncoefficients(
        a.getCoefficients().size(),
        internal::make_array<T, ARRAY_SIZE>(static_cast<T>(0)));
    for (size_t i = 0; i < a.getCoefficients().size(); i++) {
      const auto &coeffsi = a.getCoefficients()[i];
      auto &ncoeffsi = ncoefficients[i];
      for (size_t j = 0; j < coeffsi.size(); j++) ncoeffsi[j] = coeffsi[j];
    }
    setData(a.getSupport(), std::move(ncoefficients));
    return *this;
  }

  /*!
   * Spline-spline multiplication operator. Returns a spline of order order +
   * ordera.
   *
   * @brief Spline-spline multiplication operator.
   * @param a Spline to be multiplied with this spline.
   * @tparam ordera Order of spline a.
   * @throws BSplineException If the two splines are defined on different grids.
   * @returns A new spline representing the product of this spline and spline a.
   */
  template <size_t ordera>
  Spline<T, order + ordera> operator*(const Spline<T, ordera> &a) const {
    DURING_TEST_CHECK_VALIDITY();
    static constexpr size_t NEW_ORDER = order + ordera;
    static constexpr size_t NEW_ARRAY_SIZE = NEW_ORDER + 1;

    // Will also check whether the two grids are equivalent.
    Support newSupport = _support.calcIntersection(a.getSupport());
    const size_t nintervals = newSupport.numberOfIntervals();

    if (nintervals == 0)
      return Spline<T, NEW_ORDER>(std::move(newSupport), {});  // No overlap

    std::vector<std::array<T, NEW_ARRAY_SIZE>> newCoefficients(
        nintervals, internal::make_array<T, NEW_ARRAY_SIZE>(static_cast<T>(0)));

    for (size_t i = 0; i < nintervals; i++) {
      const auto ai = newSupport.absoluteFromRelative(i);

      auto thisRelIndex = _support.intervalIndexFromAbsolute(ai).value();
      auto aRelIndex = a.getSupport().intervalIndexFromAbsolute(ai).value();

      const auto &thiscoeffs = _coefficients[thisRelIndex];
      const auto &acoeffs = a.getCoefficients()[aRelIndex];
      auto &coeffsi = newCoefficients[i];

      for (size_t j = 0; j < order + 1; j++) {
        for (size_t k = 0; k < ordera + 1; k++) {
          coeffsi[j + k] += thiscoeffs[j] * acoeffs[k];
        }
      }
    }
    return Spline<T, NEW_ORDER>(std::move(newSupport),
                                std::move(newCoefficients));
  }

  /*!
   * Addition operator. Adds spline a to this spline.
   *
   * @brief Spline addition operator.
   * @param a Spline to be added.
   * @tparam ordera Order of spline a.
   * @throws BSplineException If the two splines are defined on different grids.
   * @returns A new spline representing the sum of this spline and spline a.
   */
  template <size_t ordera>
  Spline<T, std::max(order, ordera)> operator+(
      const Spline<T, ordera> &a) const {
    DURING_TEST_CHECK_VALIDITY();
    static constexpr size_t NEW_ORDER = std::max(order, ordera);
    static constexpr size_t NEW_ARRAY_SIZE = NEW_ORDER + 1;

    // Will also check whether the two grids are equivalent.
    Support newSupport = _support.calcUnion(a.getSupport());
    const size_t nintervals = newSupport.numberOfIntervals();

    std::vector<std::array<T, NEW_ARRAY_SIZE>> ncoefficients(nintervals);

    for (size_t i = 0; i < nintervals; i++) {
      const auto absIndex = newSupport.absoluteFromRelative(i);
      const auto thisRelIndex = _support.intervalIndexFromAbsolute(absIndex);
      const auto aRelIndex = a.getSupport().intervalIndexFromAbsolute(absIndex);

      if (thisRelIndex && !aRelIndex) {
        ncoefficients[i] =
            internal::changearraysize<T, order + 1, NEW_ARRAY_SIZE>(
                _coefficients[*thisRelIndex]);
      } else if (aRelIndex && !thisRelIndex) {
        ncoefficients[i] =
            internal::changearraysize<T, ordera + 1, NEW_ARRAY_SIZE>(
                a.getCoefficients()[*aRelIndex]);
      } else if (thisRelIndex && aRelIndex) {
        ncoefficients[i] = internal::add<T, ordera + 1, order + 1>(
            a.getCoefficients()[*aRelIndex], _coefficients[*thisRelIndex]);
      } else {
        ncoefficients[i] =
            internal::make_array<T, NEW_ARRAY_SIZE>(static_cast<T>(0));
      }
    }
    return Spline<T, NEW_ORDER>(std::move(newSupport),
                                std::move(ncoefficients));
  }

  /*!
   * In-place addition operator. Adds spline a to this spline. The operation is
   * only well defined if the order of spline a is lower than or equal to the
   * order of this spline object.
   *
   * @brief In-place addition operator.
   * @param a Spline to be added.
   * @tparam ordera Order of spline a.
   * @throws BSplineException If the two splines are defined on different grids.
   * @returns A reference to this spline.
   */
  template <size_t ordera>
  Spline<T, order> &operator+=(const Spline<T, ordera> &a) {
    DURING_TEST_CHECK_VALIDITY();
    static_assert(
        ordera <= order,
        "The operators += and -= are only defined if the order of the rhs "
        "spline is lower than or equal to that of the lhs spline.");
    (*this) = (*this) + a;
    return *this;
  }

  /*!
   * Binary in-place subtraction operator. Subtracts spline a from this spline.
   * The operation is only well defined if the order of the spline to be
   * subtracted is lower than or equal to the order of this spline object.
   *
   * @brief Binary in-place subtraction operator.
   * @param a Spline to be subtracted.
   * @tparam ordera Order of spline a.
   * @throws BSplineException If the two splines are defined on different grids.
   * @returns A reference to this spline.
   */
  template <size_t ordera>
  Spline<T, order> &operator-=(const Spline<T, ordera> &a) {
    (*this) += (static_cast<T>(-1) * a);
    return *this;
  }

  /*!
   * Binary subtraction operator. Subtracts spline a from this spline.
   *
   * @brief Binary subtraction operator.
   * @param a Spline to be subtracted.
   * @tparam ordera Order of spline a.
   * @throws BSplineException If the two splines are defined on different grids.
   * @returns A new spline representing the difference of this spline and spline
   * a.
   */
  template <size_t ordera>
  Spline<T, std::max(order, ordera)> operator-(
      const Spline<T, ordera> &a) const {
    return (*this) + (static_cast<T>(-1) * a);
  }

  /*!
   * @brief Compares two splines for equality.
   *
   * @param other The spline to compare this spline to.
   * @returns true if the splines are identical, false otherwise.
   */
  bool operator==(const Spline &other) const {
    return _support == other._support && _coefficients == other._coefficients;
  }

  /*!
   * @brief Compares two splines for inequality.
   *
   * @param other The spline to compare this spline to.
   * @returns false if the splines are identical, true otherwise.
   */
  bool operator!=(const Spline &other) const { return !(*this == other); }
};  // class Spline

/*!
 * @brief Deduction guide for spline constructed from array.
 */
template <typename T, size_t ARRAY_SIZE>
Spline(Support<T> support, std::vector<std::array<T, ARRAY_SIZE>> coefficients)
    -> Spline<T, ARRAY_SIZE - 1>;

//################### End of defintion of Spline class ###################
//########################################################################

/*!
 * @brief Commutation of spline scalar multiplication operator.
 *
 * @param d Scalar to be multiplied.
 * @param b Spline to be multiplied.
 * @tparam T Datatype of spline and scalar.
 * @tparam order Order of the spline.
 * @returns A new, scaled spline.
 */
template <typename T, size_t order>
inline Spline<T, order> operator*(const T &d, const Spline<T, order> &b) {
  return b * d;
}

/*!
 * Calculates the linear combination of splines. Is more efficient than
 * successive scalar multiplications and spline additions.
 *
 * @brief Calculates the linear combination.
 * @param coeffsBegin The iterator referencing the first element of the
 * coefficient collection.
 * @param coeffsEnd The iterator referencing the end of the coefficient
 * collection.
 * @param splinesBegin The iterator referencing the first element of the spline
 * collection.
 * @param splinesEnd The iterator referencing the end of the spline collection.
 * @tparam CoeffIter An iterator referencing a coefficient of type T.
 * @tparam SplineIter An iterator referenchig a spline of type Spline<T, order>.
 * @returns The linear combination as a spline of type Spline<T, order>.
 * @throws BSplineException If the number of coefficients differs from the
 * number of splines, if the number of coefficients and splines are zero or the
 * grids of all splines are not logically equivalent.
 */
template <typename CoeffIter, typename SplineIter>
auto linearCombination(CoeffIter coeffsBegin, CoeffIter coeffsEnd,
                       SplineIter splinesBegin, SplineIter splinesEnd) {
  // The spline data type.
  using Spline = typename std::remove_cv_t<
      typename std::iterator_traits<SplineIter>::value_type>;

  // The coefficient data type.
  using T = typename std::remove_cv_t<
      typename std::iterator_traits<CoeffIter>::value_type>;

  // The order of the spline.
  constexpr size_t order = Spline::spline_order;

  // Check, the data type of the spline and the coefficients are consistent.
  static_assert(
      std::is_same<typename Spline::data_type, T>::value,
      "Coefficients must be of the same type as the data type of the spline.");

  {
    // The number of coefficients and splines.
    const int coeffsSize = std::distance(coeffsBegin, coeffsEnd);
    const int splinesSize = std::distance(splinesBegin, splinesEnd);

    // Check, the data is consistent.
    if (coeffsSize != splinesSize) {
      throw BSplineException(
          ErrorCode::INCONSISTENT_DATA,
          "The number of coefficients and splines must coincide.");
    }

    // std::distance() may return negative values.
    if (coeffsSize <= 0) {
      throw BSplineException(
          ErrorCode::MISSING_DATA,
          "The number of coefficients and splines may not be zero.");
    }
  }

  const auto &support0 = splinesBegin->getSupport();
  std::optional<size_t> startIndex;
  std::optional<size_t> endIndex;

  // Determine the union of all the splines' supports.
  for (auto it = splinesBegin; it < splinesEnd; it++) {
    if (!it->getSupport().hasSameGrid(support0)) {
      throw BSplineException(ErrorCode::DIFFERING_GRIDS);
    }

    if (!it->getSupport().empty()) {
      const size_t si = it->getSupport().getStartIndex();
      if (!startIndex || si < *startIndex) {
        startIndex = si;
      }

      const size_t ei = it->getSupport().getEndIndex();
      if (!endIndex || ei > *endIndex) {
        endIndex = ei;
      }
    }
  }

  // Set up support and coefficients vector for the returned spline.
  Support newSupport(support0.getGrid(), startIndex.value_or(0),
                     endIndex.value_or(0));
  std::vector<std::array<T, order + 1>> newCoefficients(
      newSupport.numberOfIntervals(),
      internal::make_array<T, order + 1>(static_cast<T>(0)));

  auto coeffIt = coeffsBegin;
  auto splineIt = splinesBegin;
  while (splineIt < splinesEnd) {
    // Get spline and coefficient.
    const auto &spline = *splineIt;
    const T coeff = *coeffIt;

    for (size_t j = 0; j < spline.getSupport().numberOfIntervals(); j++) {
      // Index of the interval relative to the global grid.
      const size_t absoluteIndex = spline.getSupport().absoluteFromRelative(j);

      // Index of the interval relative to the newSupport.
      const size_t newSupportIndex =
          newSupport.intervalIndexFromAbsolute(absoluteIndex).value();

      const std::array<T, order + 1> &splineCoeffs =
          spline.getCoefficients().at(j);
      std::array<T, order + 1> &newCoeffs = newCoefficients.at(newSupportIndex);

      for (size_t k = 0; k < order + 1; k++) {
        newCoeffs[k] += coeff * splineCoeffs[k];
      }
    }

    splineIt++;
    coeffIt++;
  }

  return Spline(std::move(newSupport), std::move(newCoefficients));
}

/*!
 * Calculates the linear combination of splines. Is more efficient than
 * successive scalar multiplications and spline additions.
 *
 * @brief Calculates the linear combination.
 * @param coeffs The coefficient collection.
 * @param splines The spline collection.
 * @tparam CoeffCollection A collection of coefficients of type T. Must provide
 * begin() and end() iterators.
 * @tparam SplineCollection A collection of splines of type Spline<T, order>.
 * Must provide begin() and end() iterators.
 * @returns The linear combination as a spline of type Spline<T, order>.
 * @throws BSplineException If the number of coefficients differs from the
 * number of splines, if the number of coefficients and splines are zero or the
 * grids of all splines are not logically equivalent.
 */
template <typename CoeffCollection, typename SplineCollection>
auto linearCombination(const CoeffCollection &coeffs,
                       const SplineCollection &splines) {
  return linearCombination(coeffs.begin(), coeffs.end(), splines.begin(),
                           splines.end());
}

#ifndef BSPLINE_DOXYGEN_IGNORE
/*!
 * Struct to check wether type is a spline. Implementation for all types that
 * are not a Spline.
 *
 * @tparam S Type to check.
 */
template <typename S>
struct is_spline : std::false_type {};

/*!
 * Struct to check wether type is a spline. Implementation for all Splines.
 *
 * @tparam T Data type of the spline.
 * @tparam order Order of the spline.
 */

template <typename T, size_t order>
struct is_spline<Spline<T, order>> : std::true_type {};
#endif  // BSPLINE_DOXYGEN_IGNORE

/*!
 * Indicates whether the type is a spline.
 * @tparam S The type to check against the Spline class.
 */
template <typename S>
inline constexpr bool is_spline_v = is_spline<S>::value;

}  // namespace bspline
#endif  // BSPLINE_SPLINE_H
