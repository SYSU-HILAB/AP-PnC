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
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>

int main(int argc, char **argv) {
  static constexpr size_t SPLINE_DEGREE = 1;
  using Spline = bspline::Spline<double, SPLINE_DEGREE>;

  // Define knots vector.
  const std::vector<double> knots{0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0, 5.0};

  // Generate Splines.
  const std::vector<Spline> spl =
      bspline::generateBSplines<SPLINE_DEGREE>(knots);

  std::vector<double> coeffs(spl.size(), 1.0);
  std::ofstream out("diffusion.csv");
  out.precision(std::numeric_limits<double>::max_digits10);
  std::cout << spl.size();
  for (double x = -1.0; x <= 6.0; x += 0.01) {
    out << x << ",";
    double val = 0;
    for (size_t i = 0; i < spl.size(); i++) {
      val += spl[i](x) * coeffs[i];
    }
    out << val << "\n";
  }
}