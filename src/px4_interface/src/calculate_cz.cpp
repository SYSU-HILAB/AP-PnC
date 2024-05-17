#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
// #include <Eigen::Dense>

namespace fit_data {
// double a0,a1,
// trancated fourier series 4 order
double a0, w;
double a1, a2, a3, a4;
double b1, b2, b3, b4;
double kPho = 1.225;
double kS = 0.25;

void init() {
  w = 1.0;
  a0 = -11.8;
  a1 = -1.099;
  b1 = 22.35;
  a2 = 13.05;
  b2 = 1.314;
  a3 = 0.788;
  b3 = -5.099;
  a4 = -1.015;
  b4 = -0.2115;
}

};  // namespace fit_data

double get_cz_from_vb(const Eigen::Vector3d &vb) {
  double v = vb.norm();
  if (v < 1) {
    return 0.0;
  }
  // MPC state is in quadrotor frame
  double alpha = std::atan2(vb(0), vb(2));
  std::cerr << "Alpha" << alpha << std::endl;
  if (alpha < 0.0) {
    std::cerr << "Sing" << std::endl;

    return .5 * fit_data::kPho * fit_data::kS * 0.0;
  } else {
    std::cerr << "Case3" << std::endl;

    using namespace fit_data;
    init();
    // fourier series about alpha
    double cz = a0 + a1 * std::cos(w * alpha) + b1 * std::sin(w * alpha) +
                a2 * std::cos(2 * w * alpha) + b2 * std::sin(2 * w * alpha) +
                a3 * std::cos(3 * w * alpha) + b3 * std::sin(3 * w * alpha) +
                a4 * std::cos(4 * w * alpha) + b4 * std::sin(4 * w * alpha);

    // factor using rho and area;
    std::cerr << "Cz" << cz << std::endl;
    return 0.5 * fit_data::kPho * fit_data::kS * cz;
  }
}

double get_Cz_from_alpha(const double alpha) {
  // MPC state is in quadrotor frame
  if (alpha < 0.0) {
    std::cerr << "Sing" << std::endl;

    return 0.0;
  }

  using namespace fit_data;
  init();
  // fourier series about alpha
  double cz = a0 + a1 * std::cos(w * alpha) + b1 * std::sin(w * alpha) +
              a2 * std::cos(2 * w * alpha) + b2 * std::sin(2 * w * alpha) +
              a3 * std::cos(3 * w * alpha) + b3 * std::sin(3 * w * alpha) +
              a4 * std::cos(4 * w * alpha) + b4 * std::sin(4 * w * alpha);

  // factor using rho and area;
  std::cerr << "Cz" << cz << std::endl;
  return cz;
}
// 0---120 degree