/*
 * BSD 3-Clause License
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 * Authors: Erchao Rong <rongerch@outlook.com>, Zihao Liu <liuzh297@gmail.com>,
 * Junning Liang <gordonliang27@foxmail.com>
 * Paper: Aerodynamic Prior-free Trajectory Generation and Tracking Control
 * for a Tail-sitter UAV.
 */
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only. Not for
// government, commercial, or other organizational use.
// Derived from Aerodynamics.cpp, MATLAB Coder 5.4, 17-Apr-2023 05:21:59.
// The fitted polars below are expressed directly in body FLU. This maintained
// implementation is not an unmodified MATLAB-generated source file.

#include "Zhang/Aerodynamics.h"

#include <array>
#include <cmath>

namespace
{
  template <std::size_t N>
  double dot(const std::array<double, N> &coefficients,
             const std::array<double, N> &basis)
  {
    double result = 0.0;
    for (std::size_t i = 0; i < N; ++i)
      result += coefficients[i] * basis[i];
    return result;
  }
}  // namespace

void AerodynamicsFlu(double speed, double alpha, double beta, double force[3],
                     double moment[3])
{
  // The odd-in-sideslip fitted terms and moment coefficients use the same
  // positive-left beta as the public FLU interface. No component-frame adapter.
  static constexpr std::array<double, 24> drag{
      -0.5908, -1.8161, -0.2045, -0.0036, -0.2861, -0.8862, -0.059,  -0.9215,
      0.013,   -0.0035, 0.4958,  -0.0032, 0.0762,  0.1533,  -0.0009, 0.1891,
      0.0159,  0.0172,  0.251,   0.0032,  -0.1227, 0.174,   0.7917,  -4.4927};
  static constexpr std::array<double, 24> lift{
      -2.8465, 23.1516, -0.7335, 0.1407, 0.093,  3.8276,  -0.0006, 3.1951,
      -0.0289, 0.6425,  0.0164,  0.0031, 0.6174, -0.1286, 0.0342,  -0.5109,
      0.0982,  -0.0006, 0.1985,  -0.007, 0.041,  0.0037,  -4.3496, 14.1041};
  static constexpr std::array<double, 13> side{
      0.0678, -0.1151, -0.1024, 0.6002,  0.5367, -0.0922, -0.8523,
      -0.522, -0.1006, 0.2266,  -0.0345, 0.0768, -0.0748};
  static constexpr std::array<double, 18> moment_z{
      -0.024, 0.0172, 0.0219,  0.0085,  0.2163,  0.0143,
      0.0152, 0.1874, -0.0021, -0.0065, 0.011,   -0.0029,
      0.0184, 0.0028, -0.0203, 0.0158,  -0.0245, -0.0068};
  static constexpr std::array<double, 18> moment_y{
      1.9963, 2.5059,  -0.5987, -0.012,  -0.7977, -0.0543,
      -0.631, 0.1122,  0.0798,  0.2177,  0.1059,  -0.0029,
      0.1388, -0.0132, 0.2605,  -0.0015, 0.5789,  -0.7902};
  static constexpr std::array<double, 12> moment_x{
      0.0681,  -0.0338, -0.0054, -0.0028, 0.0052, 0.0034,
      -0.0048, -0.0217, -0.0111, -0.0121, 0.0685, 0.024};

  constexpr double pi = 3.1415926535897931;
  const auto       window =
      [alpha](double low, double high, double left = 1.0, double right = 1.0)
  {
    return (1.0 - 1.0 / (std::exp(left * (alpha - low)) + 1.0)) -
           1.0 / (std::exp(-right * (alpha - high)) + 1.0);
  };
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double cb = std::cos(beta), sb = std::sin(beta);
  const double cb2 = cb * cb, sb2 = sb * sb, ca2 = ca * ca, sa2 = sa * sa;
  const double abs_sb_cb = std::abs(sb) * cb;
  const std::array<double, 3> odd{((beta > 0.0) - (beta < 0.0)) * sb2,
                                  sb * cb * ca, sb * cb * sa};
  const std::array<double, 6> even{
      cb2 * ca2, sb2, cb2 * sa2, abs_sb_cb * ca, sa * ca * cb2, abs_sb_cb * sa};
  const std::array<double, 9> cubic{cb2 * std::pow(ca, 3.0),
                                    ca * sb2,
                                    ca * cb2 * sa2,
                                    sa * ca2 * cb2,
                                    sa * sb2,
                                    std::pow(sa, 3.0) * cb2,
                                    abs_sb_cb * ca2,
                                    sa * ca * cb * std::abs(sb),
                                    abs_sb_cb * sa2};
  const double lower = window(-3.3161255787892263, -2.9670597283903604);
  const double upper = window(2.9670597283903604, 3.3161255787892263);
  const double center =
      window(-0.19198621771937624, 0.31415926535897931, 20, 10);
  const double positive =
      window(0.31415926535897931, 2.9670597283903604, 10, 1);
  const double negative =
      window(-2.9670597283903604, -0.3490658503988659, 1, .5);
  const auto polar = [&](double negative_weight)
  {
    std::array<double, 24> b{};
    b[0]  = lower * cb2;
    b[1]  = lower * ((alpha + pi) * cb2);
    b[11] = center * cb2;
    b[12] = center * (alpha * cb2);
    for (std::size_t i = 0; i < cubic.size(); ++i)
    {
      b[i + 2]  = negative_weight * cubic[i];
      b[i + 13] = positive * cubic[i];
    }
    b[22] = upper * cb2;
    b[23] = upper * ((alpha - pi) * cb2);
    return b;
  };
  std::array<double, 13> side_basis{};
  side_basis[0] = lower * cb2;
  side_basis[1] = lower * ((alpha + pi) * cb2);
  const std::array<double, 3> side_weights{
      window(-2.9670597283903604, -1.5707963267948966),
      window(-1.5707963267948966, -0.19198621771937624),
      window(0.3490658503988659, 2.9670597283903604)};
  for (std::size_t block = 0; block < 3; ++block)
    for (std::size_t i = 0; i < 3; ++i)
      side_basis[2 + 3 * block + i] = side_weights[block] * odd[i];
  side_basis[11]        = upper * cb2;
  side_basis[12]        = upper * ((alpha - pi) * cb2);
  const double pressure = 0.592 * (speed * speed);
  const double d =
      -dot(drag,
           polar(window(-2.9670597283903604, -0.17453292519943295, 1, .5))) *
      pressure;
  const double l = -dot(lift, polar(negative)) * pressure;
  const double s = dot(side, side_basis) * pressure;
  // Direct FLU projections, including the fitted model's side-force coupling.
  force[0] = (sa * cb * d - sa * sb * s) + ca * l;
  force[1] = sb * d + cb * s;
  force[2] = (ca * cb * d + ca * cb * s) - sa * l;

  std::array<double, 18> z_basis{}, y_basis{};
  for (auto *b : {&z_basis, &y_basis})
  {
    (*b)[0]  = lower * cb2;
    (*b)[1]  = lower * ((alpha + pi) * cb2);
    (*b)[8]  = center * cb2;
    (*b)[9]  = center * (alpha * cb2);
    (*b)[16] = upper * cb2;
    (*b)[17] = upper * ((alpha - pi) * cb2);
  }
  const std::array<double, 4> z_weights{
      window(-2.9670597283903604, -1.5707963267948966, 1, 5),
      window(-1.5707963267948966, -0.3490658503988659, 5, .5),
      window(1.5707963267948966, 2.9670597283903604, 5, 1),
      window(0.31415926535897931, 1.5707963267948966, 10, 5)};
  for (std::size_t block = 0; block < 4; ++block)
    for (std::size_t i = 0; i < 3; ++i)
      z_basis[(block < 2 ? 2 + block * 3 : 10 + (block - 2) * 3) + i] =
          z_weights[block] * odd[i];
  for (std::size_t i = 0; i < 6; ++i)
  {
    y_basis[i + 2]  = negative * even[i];
    y_basis[i + 10] = positive * even[i];
  }
  std::array<double, 12> x_basis{};
  const double           x_center =
      window(-0.17453292519943295, 0.31415926535897931, 10, 10);
  x_basis[0]  = lower * cb2;
  x_basis[1]  = lower * ((alpha + pi) * cb2);
  x_basis[5]  = x_center * cb2;
  x_basis[6]  = x_center * (alpha * cb2);
  x_basis[10] = upper * cb2;
  x_basis[11] = upper * ((alpha - pi) * cb2);
  for (std::size_t i = 0; i < 3; ++i)
  {
    x_basis[i + 2] =
        window(-2.9670597283903604, -0.17453292519943295, 1, 10) * odd[i];
    x_basis[i + 7] = window(0, 2.9670597283903604, .5, 1) * odd[i];
  }
  moment[0] = dot(moment_x, x_basis) * pressure;
  moment[1] = dot(moment_y, y_basis) * pressure;
  moment[2] = dot(moment_z, z_basis) * pressure;
}
