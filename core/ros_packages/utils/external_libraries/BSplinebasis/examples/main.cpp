/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "bspline/interpolation/interpolation.h"
#include "diffusion.h"
#include "harmonic-oscillator.h"
#include "hydrogen.h"
#include "spline-potential.h"

using namespace bspline::examples::hydrogen;
using namespace bspline::examples::harmonic_oscillator;
using namespace bspline::examples::spline_potential;
using namespace bspline::examples::diffusion;
using namespace bspline::examples;

static void harmonicOscillator() {
  std::cout.precision(std::numeric_limits<data_t>::max_digits10);
  const std::vector<Eigenspace> harmonicOscillator = solveHarmonicOscillator();

  std::cout << "Harmonic Oscillator eigenvalues:\n\n";
  std::cout << std::setw(2) << "n"
            << "\t" << std::setw(20) << "energy"
            << "\t" << std::setw(20) << "relative deviation" << '\n';
  for (size_t i = 0; i < harmonicOscillator.size(); i++) {
    const data_t expected = static_cast<data_t>(2 * i + 1) / 2;
    const data_t relativeDev =
        std::abs((harmonicOscillator[i].energy - expected) / expected);
    std::cout << std::setw(2) << i << "\t" << std::setw(20)
              << harmonicOscillator[i].energy << "\t" << std::setw(20)
              << relativeDev << '\n';
  }
  std::cout << '\n' << std::endl;

  {
    std::ofstream out("harmonic-oscillator-wavefunctions.txt");
    out.precision(std::numeric_limits<data_t>::max_digits10);
    for (data_t x = static_cast<data_t>(-101) / 10;
         x <= static_cast<data_t>(101) / 10;
         x += static_cast<data_t>(1) / 100) {
      out << x << "\t" << 0.5 * x * x;
      for (const auto &r : harmonicOscillator) {
        out << "\t" << r.wavefunction(x) + r.energy;
      }
      out << "\n";
    }
  }
}

static void splinePotential() {
  std::cout.precision(std::numeric_limits<data_t>::max_digits10);
  std::vector<data_t> gridPoints;
  for (int i = -100; i <= 100; i++) {
    gridPoints.push_back(static_cast<data_t>(i) / 10);
  }

  const auto v = interpolateFunction(std::move(gridPoints), [](data_t x) {
    return (exp(x) + exp(-x)) / static_cast<data_t>(2) - static_cast<data_t>(1);
  });

  const std::vector<Eigenspace> eigenSpaces = solveSEWithSplinePotential(v);

  std::cout << "Spline potential eigenvalues:\n\n";
  std::cout << std::setw(2) << "n"
            << "\t" << std::setw(20) << "energy" << '\n';
  for (size_t i = 0; i < eigenSpaces.size(); i++) {
    std::cout << std::setw(2) << i << "\t" << std::setw(20)
              << eigenSpaces[i].energy << '\n';
  }
  std::cout << '\n' << std::endl;

  {
    std::ofstream out("spline-potential-wavefunctions.txt");
    out.precision(std::numeric_limits<data_t>::max_digits10);
    for (data_t x = static_cast<data_t>(-101) / 10;
         x <= static_cast<data_t>(101) / 10;
         x += static_cast<data_t>(1) / 100) {
      out << x << "\t" << v(x);
      for (const auto &r : eigenSpaces) {
        out << "\t" << r.wavefunction(x) + r.energy;
      }
      out << "\n";
    }
  }
}

static void radialHydrogen() {
  std::cout.precision(std::numeric_limits<data_t>::max_digits10);
  const std::vector<Eigenspace> hydrogen = solveRadialHydrogen();

  std::cout << "Hydrogen (L=" << L << ") eigenvalues:\n\n";
  std::cout << std::setw(2) << "n"
            << "\t" << std::setw(25) << "energy"
            << "\t" << std::setw(25) << "relative deviation" << '\n';
  for (size_t i = 0; i < hydrogen.size(); i++) {
    const data_t expected =
        static_cast<data_t>(-1) / pow(static_cast<data_t>(L + i + 1), 2);
    const data_t relativeDev =
        std::abs((hydrogen[i].energy - expected) / expected);
    std::cout << std::setw(2) << i + L + 1 << "\t" << std::setw(25)
              << hydrogen[i].energy << "\t" << std::setw(25) << relativeDev
              << '\n';
  }
  std::cout << '\n' << std::endl;

  {
    std::ofstream out("radial-hydrogen-wavefunctions.txt");
    out.precision(std::numeric_limits<data_t>::max_digits10);
    for (data_t x = static_cast<data_t>(0); x <= static_cast<data_t>(1000);
         x = x * static_cast<data_t>(102) / 100 +
             static_cast<data_t>(1) / 1000) {
      out << x << "\t" << ((x < 0.75) ? -2.0 / 0.75 : -2.0 / x);
      for (const auto &r : hydrogen) {
        out << "\t" << r.wavefunction(x) + r.energy;
      }
      out << "\n";
    }
  }
}

static void diffusionEquation() {
  std::cout.precision(std::numeric_limits<data_t>::max_digits10);
  std::vector<data_t> gridPoints;
  std::vector<data_t> diffCoeffVals;
  for (int i = -100; i <= 100; i++) {
    gridPoints.push_back(static_cast<data_t>(i) / 10);
    if (abs(i) <= 33) {
      diffCoeffVals.push_back(static_cast<data_t>(1) / 3);
    } else {
      diffCoeffVals.push_back(static_cast<data_t>(1));
    }
  }
  bspline::support::Grid<data_t> grid(std::move(gridPoints));
  auto support = bspline::support::Support<data_t>::createWholeGrid(grid);

  const auto diffCoeff =
      bspline::interpolation::interpolateUsingEigen<data_t,
                                                    PSpline::spline_order>(
          support, diffCoeffVals);
  const auto solution = solveDiffusionSteadyState(
      diffCoeff, static_cast<data_t>(0), static_cast<data_t>(10));

  {
    std::ofstream out("diffusion.txt");
    out.precision(std::numeric_limits<data_t>::max_digits10);
    for (data_t x = static_cast<data_t>(-10); x <= static_cast<data_t>(10);
         x += static_cast<data_t>(1) / 100) {
      out << x << "\t" << diffCoeff(x) << "\t" << solution(x) << "\n";
    }
  }
}

static void printUsage(const std::string &command) {
  std::cout << "Usage:\n\t" << command
            << " <example>\n\tWhere <example> is either "
               "\"harmonic_oscillator\", \"hydrogen\", \"spline_potential\" or "
               "\"diffusion\"."
            << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string example(argv[1]);

  if (example == "harmonic_oscillator") {
    harmonicOscillator();
  } else if (example == "hydrogen") {
    radialHydrogen();
  } else if (example == "spline_potential") {
    splinePotential();
  } else if (example == "diffusion") {
    diffusionEquation();
  } else {
    printUsage(argv[0]);
    return 1;
  }

  return 0;
}
