/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */
#include <bspline/Core.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <string>

using namespace bspline;

static constexpr size_t order = 3;

static void printSplines(const std::string &fileName,
                         const std::vector<double> &knots) {
  const auto splines =
      BSplineGenerator(knots).template generateBSplines<order + 1>();

  std::ofstream o(fileName.c_str());
  o.precision(std::numeric_limits<double>::max_digits10);

  for (double x = 0.0; x <= 5.0; x += 0.01) {
    o << x;
    for (const auto &spline : splines) {
      o << "\t" << spline(x);
    }
    o << "\n";
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Please specify output folder:\n"
              << argv[0] << " <output-folder>." << std::endl;
    return 1;
  }

  const std::string folder{argv[1]};
  const std::vector<double> knotsNormal{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  const std::vector<double> knotsNonContinuous{0.0, 0.0, 0.0, 0.0, 1.0,
                                               2.0, 3.0, 4.0, 5.0};

  printSplines(folder + "/splines_normal.txt", knotsNormal);
  printSplines(folder + "/splines_non_continuous.txt", knotsNonContinuous);

  return 0;
}
