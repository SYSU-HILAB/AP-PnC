/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */
#include <bspline/Core.h>

#include <algorithm>

// For clang, the serialization header must be included before the
// multiprecision headers to avoid compilation errors.
// clang-format off
#include <boost/serialization/nvp.hpp>
#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/multiprecision/eigen.hpp>
// clang-format on

#include <eigen3/Eigen/Eigenvalues>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <string>

using namespace bspline;

template <typename data_t>
using DeMat = Eigen::Matrix<data_t, Eigen::Dynamic, Eigen::Dynamic>;

template <typename B, typename data_t, size_t order>
DeMat<data_t> setUpSymmetricMatrix(
    const B &b, const std::vector<Spline<data_t, order>> &basis) {
  DeMat<data_t> ret = DeMat<data_t>::Zero(basis.size(), basis.size());
  for (size_t i = 0; i < basis.size(); i++) {
    for (size_t j = i; j < basis.size(); j++) {
      const data_t val = b.evaluate(basis.at(i), basis.at(j));
      ret(i, j) = val;
      ret(j, i) = val;
    }
  }
  return ret;
}

/**
 * @brief setUpKnotsVector Sets up the knots vector on which the basis splines
 * are defined.
 * @param points The number of grid points.
 * @return A vector of data_t representing the knots.
 */
template <typename data_t>
static std::vector<data_t> setUpKnotsVector(int points) {
  std::vector<data_t> ret{0};

  const int pointsPerSide = points / 2;

  for (int i = 1; i <= pointsPerSide; i++) {
    const data_t fraction = static_cast<data_t>(i) / pointsPerSide;
    const data_t val = fraction * fraction * 15;

    ret.push_back(val);
    ret.push_back(-val);
  }
  std::sort(ret.begin(), ret.end());
  return ret;
}

/**
 * @brief setUpBasis Sets up the BSpline basis.
 * @return A vector of BSplines representing the basis.
 */
template <typename data_t, size_t order>
static std::vector<bspline::Spline<data_t, order>> setUpBasis(int points) {
  return generateBSplines<order>(setUpKnotsVector<data_t>(points));
}

template <typename data_t>
data_t calcRelativeDeviation(const data_t &result, size_t n) {
  const data_t expected = static_cast<data_t>(2 * n + 1) / 2;
  return abs((result - expected) / expected);
}

template <typename data_t>
struct RetVal {
  size_t basisSize;
  std::vector<data_t> deviations;
};

template <typename data_t, size_t order>
RetVal<data_t> solveHarmonicOscillator(int points) {
  // Hamiltonian operator -1/2 d^2/dx^2 + 1/2 x^2
  static const auto hamiltonOperator =
      (static_cast<data_t>(1) / 2) * (-operators::Dx<2>{} + operators::X<2>{});

  // Get the basis.
  const std::vector<Spline<data_t, order>> basis =
      setUpBasis<data_t, order>(points);

  const DeMat<data_t> hamiltonian =
      setUpSymmetricMatrix(integration::BilinearForm{hamiltonOperator}, basis);

  // overlap matrix
  const DeMat<data_t> overlapMatrix =
      setUpSymmetricMatrix(integration::ScalarProduct{}, basis);

  // Solve the generalized eigenvalue problem A.x = lambda B.x
  const Eigen::GeneralizedSelfAdjointEigenSolver<DeMat<data_t>> ges{
      hamiltonian, overlapMatrix};

  const auto &eigenvalues = ges.eigenvalues();

  std::vector<data_t> ret;
  ret.reserve(10);
  for (size_t i = 0; i < 10; i++) {
    ret.push_back(calcRelativeDeviation(eigenvalues(i), i));
  }
  return {basis.size(), std::move(ret)};
}

template <typename data_t, size_t order>
void printDeviations(const std::string &fileName) {
  std::ofstream o(fileName.c_str());
  o.precision(std::numeric_limits<data_t>::max_digits10);
  const std::vector<int> POINTS{35, 50, 100, 200, 500, 1000, 2000};

  for (int points : POINTS) {
    const auto results = solveHarmonicOscillator<data_t, order>(points);
    o << results.basisSize;
    for (const auto &a : results.deviations) {
      o << "\t" << a;
    }
    o << std::endl;
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Please specify output folder:\n"
              << argv[0] << " <output-folder>." << std::endl;
    return 1;
  }

  using quad = boost::multiprecision::cpp_bin_float_quad;

  std::cout.precision(std::numeric_limits<double>::max_digits10);
  std::cout << "eps(double): " << std::numeric_limits<double>::epsilon()
            << std::endl;
  std::cout.precision(std::numeric_limits<quad>::max_digits10);
  std::cout << "eps(quad): " << std::numeric_limits<quad>::epsilon()
            << std::endl;

  const std::string folder{argv[1]};

  std::vector<std::future<void>> futures;

  futures.push_back(std::async(std::launch::async, [folder]() {
    printDeviations<double, 2>(folder + "/double_2.txt");
    printDeviations<double, 5>(folder + "/double_5.txt");
    printDeviations<double, 10>(folder + "/double_10.txt");
    printDeviations<double, 15>(folder + "/double_15.txt");
  }));

  // Quad
  futures.push_back(std::async(std::launch::async, [folder]() {
    printDeviations<quad, 2>(folder + "/quad_2.txt");
  }));
  futures.push_back(std::async(std::launch::async, [folder]() {
    printDeviations<quad, 5>(folder + "/quad_5.txt");
  }));
  futures.push_back(std::async(std::launch::async, [folder]() {
    printDeviations<quad, 10>(folder + "/quad_10.txt");
  }));
  futures.push_back(std::async(std::launch::async, [folder]() {
    printDeviations<quad, 15>(folder + "/quad_15.txt");
  }));

  return 0;
}
