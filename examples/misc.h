/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_EXAMPLES_MISC_H
#define BSPLINE_EXAMPLES_MISC_H

#ifdef EXAMPLES_USE_BOOST_MULTIPRECISION
// Just as a proof of concept.
// Beware boost multiprecision tries to include the eigen headers from
// <Eigen/Core>. Without additional include definitions, they may be placed
// under <eigen3/Eigen/Core>, however, depending on your installation.
#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/multiprecision/eigen.hpp>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <bspline/Core.h>

#include <vector>

namespace bspline::examples {

#ifdef EXAMPLES_USE_BOOST_MULTIPRECISION
using data_t = boost::multiprecision::cpp_bin_float_oct;
#else
using data_t = double;
#endif

using DeMat = Eigen::Matrix<data_t, Eigen::Dynamic, Eigen::Dynamic>;
using DeVec = Eigen::Matrix<data_t, Eigen::Dynamic, 1>;

constexpr size_t SPLINE_ORDER = 10;

using Spline = bspline::Spline<data_t, SPLINE_ORDER>;

/**
 * The (cubic) potential spline.
 */
using PSpline = bspline::Spline<data_t, 3>;

struct Eigenspace {
  data_t energy;
  Spline wavefunction;
};

/**
 * @brief setUpSymmetricMatrix Sets up a symmetric matrix, where the matrix
 * elements are defined by the bilinear form.
 * @param b The BilinearForm.
 * @param basis The basis functions (i.e. BSplines).
 * @tparam B The type of the bilinear form.
 * @returns The matrix.
 */
template <typename B>
DeMat setUpSymmetricMatrix(const B &b, const std::vector<Spline> &basis) {
  DeMat ret = DeMat::Zero(basis.size(), basis.size());
  for (size_t i = 0; i < basis.size(); i++) {
    for (size_t j = i; j < basis.size(); j++) {
      const data_t val = b(basis.at(i), basis.at(j));
      ret(i, j) = val;
      ret(j, i) = val;
    }
  }
  return ret;
}

/**
 * Turns the real Eigen vector into a std::vector. This method is purely a
 * convenience function to transfer the vector data into a collection which
 * provides iterators (needed for bspline::linearCombination()). As of
 * Eigen 4.3, the Eigen collections are supposed to provide iterators themselves
 * and this method should be obsolete.
 *
 * @param d The real Eigen vector.
 * @tparam Ev Type of d.
 */
template <typename Ev>
std::vector<data_t> toStdVector(const Ev &d) {
  std::vector<data_t> ret;
  ret.reserve(d.size());
  for (int i = 0; i < d.size(); i++) {
    ret.push_back(d(i));
  }
  return ret;
}

}  // namespace bspline::examples
#endif  // BSPLINE_EXAMPLES_MISC_H
