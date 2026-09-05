/*
 * This file contains additional numerical interpolation routines for the
 * splines. The linear algebra routines can be supplied via the implementation
 * of a Solver class (a subclass of internal::ISolver<T>. Implementations based
 * on armadillo and eigen are provided and can be used by defining
 * BSPLINE_INTERPOLATION_USE_ARMADILLO or
 * BSPLINE_INTERPOLATION_USE_EIGEN , respectively.
 *
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_INTERPOLATION_INTERPOLATION_H
#define BSPLINE_INTERPOLATION_INTERPOLATION_H

#include <bspline/Spline.h>
#include <bspline/exceptions/BSplineException.h>
#include <bspline/internal/misc.h>
#include <bspline/support/Support.h>

#ifdef BSPLINE_INTERPOLATION_USE_EIGEN
#include <eigen3/Eigen/Dense>
#endif

#ifdef BSPLINE_INTERPOLATION_USE_ARMADILLO
#include <armadillo>
#endif

/*!
 * Interpolate data using the bspline::Spline
 */
namespace bspline::interpolation {
using namespace bspline::exceptions;

/*!
 * Represents either the first or last node of the interpolation grid. Used to
 * define boundary conditions.
 */
enum class Node { FIRST, LAST };

/*!
 * Represents a boundary condition, i.e. one fixed derivative on either the
 * first or last node of the interpolation grid.
 */
template <typename T>
struct Boundary {
  /*! Node to apply the boundary condition to. */
  Node node = Node::FIRST;
  /*! Order of the derivative to fix. */
  size_t derivative = 1;
  /*! Value of the derivative. */
  T value = static_cast<T>(0);
};

#ifndef BSPLINE_DOXYGEN_IGNORE
namespace internal {
/*!
 * Interface for a solver of linear equation system (LES) used to decouple the
 * interpolation code from the linear algebra frameworks. The LES M.x = b is
 * defined by the matrix M and the vector b, which have to be supplied. The
 * vector x is calculated during the execution of the method solve(). All
 * elements of M and b must be zero if not explicitly set.
 *
 * @tparam T Datatype of the linear system of equations to be solved.
 */
template <typename T>
class ISolver {
 public:
  /*!
   * Default constructor.
   */
  ISolver() = default;

  /*!
   * A constructor taking the problem size must be supplied by all subclasses.
   *
   * @param problemsize Dimension of the problem (i.e. number of coefficients).
   */
  ISolver([[maybe_unused]] size_t problemsize){};
  virtual ~ISolver() = default;

  /*!
   * Retrieve a reference to an element of the matrix M.
   *
   * @param i Row index.
   * @param j Column index.
   * @returns A reference to the corresponding element of M.
   */
  virtual T &M(size_t i, size_t j) = 0;

  /*!
   * Retrieve a reference to an element of the vector b.
   *
   * @param i Row index.
   * @returns A reference to the corresponding element of b.
   */
  virtual T &b(size_t i) = 0;

  /*!
   * Solve the LES and set the vector x accordingly.
   */
  virtual void solve() = 0;

  /*!
   * Retrieve a reference to an element of the vector x.
   *
   * @param i Row index.
   * @returns A reference to the corresponding element of i.
   */
  virtual T &x(size_t i) = 0;
};

/*!
 * Generates the default boundary conditions by setting as many derivatives to
 * zero as needed, starting from the first derivative.
 *
 * @tparam T Datatype of the spline.
 * @tparam order Polynomial order of the spline.
 * @returns An array with default order - 1 default boundary conditions.
 */
template <typename T, size_t order>
std::array<Boundary<T>, order - 1> defaultBoundaries() {
  static_assert(order >= 1, "Order may not be zero.");
  std::array<Boundary<T>, order - 1> ret;
  for (size_t i = 0; i < order - 1; i++) {
    if (i % 2 == 0) {
      ret[i] = Boundary<T>{/*.node = */ Node::FIRST,
                           /*.derivative = */ i / 2 + 1,
                           /*.value = */ static_cast<T>(0)};
    } else {
      ret[i] = Boundary<T>{/*.node = */ Node::LAST,
                           /*.derivative = */ (i - 1) / 2 + 1,
                           /*.value = */ static_cast<T>(0)};
    }
  }
  return ret;
}
}  // end namespace internal

using bspline::support::Support;
#endif  // BSPLINE_DOXYGEN_IGNORE

/*!
 * Interpolates the data given by x and y with a spline of order order. order-1
 * additional conditions are needed for a well defined problem. These can be
 * supplied by fixing derivatives on the first and last node.
 *
 * @param x Data on the abscissa.
 * @param y Data on the ordinate.
 * @param boundaries Boundary conditions.
 * @tparam T Datatype of the spline and data.
 * @tparam order Order of the spline.
 * @tparam Solver Class Wrapping the linear algebra routines.
 * @throws BSplineException If the number of values on the abscissa and ordinate
 * differ.
 * @throws BSplineException If less than two data points are provided.
 * @returns The spline interpolating the input data.
 */
template <typename T, size_t order, class Solver>
bspline::Spline<T, order> interpolate(
    Support<T> x, const std::vector<T> &y,
    const std::array<Boundary<T>, order - 1> &boundaries =
        internal::defaultBoundaries<T, order>()) {
  static_assert(order >= 1, "Order may not be zero.");
  static_assert(std::is_base_of<internal::ISolver<T>, Solver>::value,
                "Solver must be a subclass of internal::ISolver<T>.");

  if (x.size() != y.size()) {
    throw BSplineException(ErrorCode::INCONSISTENT_DATA);
  }

  if (x.size() < 2) {
    throw BSplineException(
        ErrorCode::UNDETERMINED,
        "At least two grid points needed for interpolation.");
  }

  constexpr size_t NUM_COEFFS = order + 1;

  Solver s(NUM_COEFFS * (x.size() - 1));

  size_t row_counter = 0;
  {
    const T dx1 = (x[0] - x[1]) / static_cast<T>(2);
    {
      T power_of_dx1 = static_cast<T>(1);
      for (size_t i = 0; i <= order; i++) {
        s.M(row_counter, i) = power_of_dx1;
        power_of_dx1 *= dx1;
      }
      s.b(row_counter) = y.front();
      row_counter++;
    }

    for (const auto &bo : boundaries) {
      if (bo.derivative == 0 || bo.derivative > order) {
        throw BSplineException(ErrorCode::UNDETERMINED,
                               "Unsupported order of the derivative.");
      }
      if (bo.node == Node::FIRST) {
        T power_of_dx1 = static_cast<T>(1);
        for (size_t i = bo.derivative; i <= order; i++) {
          s.M(row_counter, i) =
              bspline::internal::facultyRatio<T>(i, i - bo.derivative) *
              power_of_dx1;
          power_of_dx1 *= dx1;
        }
        s.b(row_counter) = bo.value;
        row_counter++;
      }
    }
  }

  for (size_t c = 1; c + 1 < x.size(); c++) {
    const T dx1 = (x[c] - x[c - 1]) / static_cast<T>(2);
    const T dx2 = (x[c] - x[c + 1]) / static_cast<T>(2);

    {
      T power_of_dx1 = static_cast<T>(1);
      for (size_t i = 0; i <= order; i++) {
        s.M(row_counter, NUM_COEFFS * (c - 1) + i) = power_of_dx1;
        power_of_dx1 *= dx1;
      }
      s.b(row_counter) = y[c];
      row_counter++;
    }

    {
      T power_of_dx2 = static_cast<T>(1);
      for (size_t i = 0; i <= order; i++) {
        s.M(row_counter, NUM_COEFFS * c + i) = power_of_dx2;
        power_of_dx2 *= dx2;
      }
      s.b(row_counter) = y[c];
      row_counter++;
    }

    for (size_t deriv = 1; deriv < order; deriv++) {
      T power_of_dx1 = static_cast<T>(1);
      T power_of_dx2 = static_cast<T>(1);
      for (size_t i = deriv; i <= order; i++) {
        s.M(row_counter, NUM_COEFFS * (c - 1) + i) =
            bspline::internal::facultyRatio<T>(i, i - deriv) * power_of_dx1;
        s.M(row_counter, NUM_COEFFS * c + i) =
            -bspline::internal::facultyRatio<T>(i, i - deriv) * power_of_dx2;
        power_of_dx1 *= dx1;
        power_of_dx2 *= dx2;
      }
      row_counter++;
    }
  }

  {
    const T dx2 = (x.back() - x[x.size() - 2]) / static_cast<T>(2);
    {
      T power_of_dx2 = static_cast<T>(1);
      for (size_t i = 0; i <= order; i++) {
        s.M(row_counter, NUM_COEFFS * (x.size() - 2) + i) = power_of_dx2;
        power_of_dx2 *= dx2;
      }
      s.b(row_counter) = y.back();
      row_counter++;
    }

    for (const auto &bo : boundaries) {
      if (bo.node == Node::LAST) {
        T power_of_dx2 = static_cast<T>(1);
        for (size_t i = bo.derivative; i <= order; i++) {
          s.M(row_counter, NUM_COEFFS * (x.size() - 2) + i) =
              bspline::internal::facultyRatio<T>(i, i - bo.derivative) *
              power_of_dx2;
          power_of_dx2 *= dx2;
        }
        s.b(row_counter) = bo.value;
        row_counter++;
      }
    }
  }

  if (row_counter != NUM_COEFFS * (x.size() - 1)) {
    throw BSplineException(ErrorCode::UNDETERMINED);
  }

  s.solve();

  std::vector<std::array<T, NUM_COEFFS>> coeffs((x.size() - 1));
  for (size_t i = 0; i + 1 < x.size(); i++) {
    std::array<T, NUM_COEFFS> &coeffsi = coeffs[i];
    for (size_t j = 0; j < NUM_COEFFS; j++)
      coeffsi[j] = s.x(NUM_COEFFS * i + j);
  }
  return bspline::Spline<T, order>(std::move(x), std::move(coeffs));
}

#ifdef BSPLINE_INTERPOLATION_USE_ARMADILLO

/*!
 * Wrapper method around interpolate() using armadillo for the linear algebra
 * routines. Supports only double precision. This method will only be activated
 * if the macro BSPLINE_INTERPOLATION_USE_ARMADILLO is defined.
 *
 * @param x Data on the abscissa. The grid points must be in (steadily)
 * increasing order.
 * @param y Data on the ordinate.
 * @param boundaries Boundary conditions.
 * @tparam order Order of the spline.
 * @throws BSplineException If the number of values on the abscissa and ordinate
 * differ.
 * @throws BSplineException If less than two data points are provided.
 * @returns The spline interpolating the input data.
 */
template <size_t order>
bspline::Spline<double, order> interpolateUsingArmadillo(
    Support<double> x, const std::vector<double> &y,
    const std::array<Boundary<double>, order - 1> &boundaries =
        internal::defaultBoundaries<double, order>()) {
  class ArmadilloSolver final : public internal::ISolver<double> {
   private:
    arma::mat _M;
    arma::vec _b, _x;

   public:
    ArmadilloSolver(size_t problemsize)
        : _M(arma::mat(problemsize, problemsize, arma::fill::zeros)),
          _b(arma::vec(problemsize, arma::fill::zeros)){};
    ~ArmadilloSolver() override = default;
    double &M(size_t i, size_t j) override { return _M(i, j); };
    double &b(size_t i) override { return _b(i); };
    void solve() override { _x = arma::solve(_M, _b); };
    double &x(size_t i) override { return _x(i); };
  };

  return interpolate<double, order, ArmadilloSolver>(std::move(x), y,
                                                     boundaries);
}
#endif

#ifdef BSPLINE_INTERPOLATION_USE_EIGEN
/*!
 * Wrapper method around interpolate() using eigen for the linear algebra
 * routines. This method will only be activated if the macro
 * BSPLINE_INTERPOLATION_USE_EIGEN is defined.
 *
 * @param x Data on the abscissa. The grid points must be in (steadily)
 * increasing order.
 * @param y Data on the ordinate.
 * @param boundaries Boundary conditions.
 * @tparam T Datatype of the spline and data.
 * @tparam order Order of the spline.
 * @throws BSplineException If the number of values on the abscissa and ordinate
 * differ.
 * @throws BSplineException If less than two data points are provided.
 * @returns The spline interpolating the input data.
 */
template <typename T, size_t order>
bspline::Spline<T, order> interpolateUsingEigen(
    Support<T> x, const std::vector<T> &y,
    const std::array<Boundary<T>, order - 1> &boundaries =
        internal::defaultBoundaries<T, order>()) {
  using DeMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  using DeVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

  class EigenSolver final : public internal::ISolver<T> {
   private:
    DeMat _M;
    DeVec _b, _x;

   public:
    EigenSolver(size_t problemsize)
        : _M(DeMat::Zero(problemsize, problemsize)),
          _b(DeVec::Zero(problemsize)){};
    ~EigenSolver() override = default;
    T &M(size_t i, size_t j) override { return _M(i, j); };
    T &b(size_t i) override { return _b(i); };
    void solve() override { _x = _M.colPivHouseholderQr().solve(_b); };
    T &x(size_t i) override { return _x(i); };
  };

  return interpolate<T, order, EigenSolver>(std::move(x), y, boundaries);
}

#endif

}  // namespace bspline::interpolation
#endif  // SPLINE_INTERPOLATION_H
