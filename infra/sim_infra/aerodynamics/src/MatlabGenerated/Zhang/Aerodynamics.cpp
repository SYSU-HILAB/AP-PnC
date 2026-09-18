/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 16:02:09
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2023-04-24 20:24:25
 */
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Aerodynamics.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 17-Apr-2023 05:21:59
//

// Include Files
#include "Zhang/Aerodynamics.h"

#include <cmath>

#include "rt_defines.h"
#include "rt_nonfinite.h"

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1))
  {
    y = rtNaN;
  }
  else if (std::isinf(u0) && std::isinf(u1))
  {
    int b_u0;
    int b_u1;
    if (u0 > 0.0)
    {
      b_u0 = 1;
    }
    else
    {
      b_u0 = -1;
    }
    if (u1 > 0.0)
    {
      b_u1 = 1;
    }
    else
    {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  }
  else if (u1 == 0.0)
  {
    if (u0 > 0.0)
    {
      y = RT_PI / 2.0;
    }
    else if (u0 < 0.0)
    {
      y = -(RT_PI / 2.0);
    }
    else
    {
      y = 0.0;
    }
  }
  else
  {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1))
  {
    y = rtNaN;
  }
  else
  {
    double d;
    double d1;
    d  = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1))
    {
      if (d == 1.0)
      {
        y = 1.0;
      }
      else if (d > 1.0)
      {
        if (u1 > 0.0)
        {
          y = rtInf;
        }
        else
        {
          y = 0.0;
        }
      }
      else if (u1 > 0.0)
      {
        y = 0.0;
      }
      else
      {
        y = rtInf;
      }
    }
    else if (d1 == 0.0)
    {
      y = 1.0;
    }
    else if (d1 == 1.0)
    {
      if (u1 > 0.0)
      {
        y = u0;
      }
      else
      {
        y = 1.0 / u0;
      }
    }
    else if (u1 == 2.0)
    {
      y = u0 * u0;
    }
    else if ((u1 == 0.5) && (u0 >= 0.0))
    {
      y = std::sqrt(u0);
    }
    else if ((u0 < 0.0) && (u1 > std::floor(u1)))
    {
      y = rtNaN;
    }
    else
    {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// compute the aerodynamic force 'f' and momentums 'm' given the air_speed
//  in R3.
//  input:
//    - air_speed (V_b): the air speed vector w.r.t. body and projected onto the
//    body frame
//    - f(F_w): the aerodynamic force vector represented in the wind frame
//        (i.e. drag - body Y - lift)
//    (I think it is in the stability axis,need to conver it to body frame)
//    - m(M_b); the aerdoynamic momentum vector represented in the body frame
//
// Arguments    : const double V_b[3]
//                double F_b[3]
//                double M_b[3]
//                double *alpha
//                double *beta
// Return Type  : void
//
void Aerodynamics(const double V_b[3], double F_b[3], double M_b[3],
                  double *alpha, double *beta)
{
  static const double dv[24]{
      -0.5908, -1.8161, -0.2045, -0.0036, -0.2861, -0.8862, -0.059,  -0.9215,
      0.013,   -0.0035, 0.4958,  -0.0032, 0.0762,  0.1533,  -0.0009, 0.1891,
      0.0159,  0.0172,  0.251,   0.0032,  -0.1227, 0.174,   0.7917,  -4.4927};
  static const double dv2[24]{
      -2.8465, 23.1516, -0.7335, 0.1407, 0.093,  3.8276,  -0.0006, 3.1951,
      -0.0289, 0.6425,  0.0164,  0.0031, 0.6174, -0.1286, 0.0342,  -0.5109,
      0.0982,  -0.0006, 0.1985,  -0.007, 0.041,  0.0037,  -4.3496, 14.1041};
  static const double dv3[18]{-0.024,  0.0172,  -0.0219, -0.0085, -0.2163,
                              -0.0143, -0.0152, -0.1874, -0.0021, -0.0065,
                              -0.011,  0.0029,  -0.0184, -0.0028, 0.0203,
                              -0.0158, -0.0245, -0.0068};
  static const double dv4[18]{-1.9963, -2.5059, 0.5987,  0.012,   0.7977,
                              0.0543,  0.631,   -0.1122, -0.0798, -0.2177,
                              -0.1059, 0.0029,  -0.1388, 0.0132,  -0.2605,
                              0.0015,  -0.5789, 0.7902};
  static const double dv1[13]{-0.0678, 0.1151,  -0.1024, 0.6002,  0.5367,
                              -0.0922, -0.8523, -0.522,  -0.1006, 0.2266,
                              -0.0345, -0.0768, 0.0748};
  static const double dv5[12]{0.0681,  -0.0338, 0.0054, 0.0028, -0.0052, 0.0034,
                              -0.0048, 0.0217,  0.0111, 0.0121, 0.0685,  0.024};
  double              b_y[24];
  double              d_y[18];
  double              c_y[13];
  double              e_y[12];
  double              c_comp_N[9];
  double              d_comp_N[9];
  double              b_comp_N[6];
  double              comp_N[3];
  double              P;
  double              a;
  double              a_tmp;
  double              absxk;
  double              b_a;
  double              b_a_tmp;
  double              b_comp_N_idx_0;
  double              b_comp_N_idx_1;
  double              b_comp_N_idx_2;
  double              b_comp_N_tmp;
  double              b_comp_N_tmp_tmp;
  double              b_y_tmp;
  double              c_a;
  double              c_a_tmp;
  double              c_comp_N_tmp;
  double              c_y_tmp;
  double              cb2_tmp;
  double              cb_tmp;
  double              comp_N_idx_0;
  double              comp_N_idx_0_tmp;
  double              comp_N_idx_1;
  double              comp_N_idx_1_tmp;
  double              comp_N_idx_2;
  double              comp_N_idx_2_tmp;
  double              comp_N_tmp;
  double              comp_N_tmp_tmp;
  double              d_a;
  double              d_a_tmp;
  double              e_a;
  double              f_a;
  double              g_a;
  double              h_a;
  double              i_a;
  double              sb_tmp;
  double              scale;
  double              t;
  double              y;
  double              y_tmp;
  boolean_T           b;
  //  compute the angle of attack and sideslip angle
  *alpha = rt_atan2d_snf(V_b[2], V_b[0]);
  scale  = 3.3121686421112381E-170;
  absxk  = std::abs(V_b[0]);
  if (absxk > 3.3121686421112381E-170)
  {
    a     = 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / 3.3121686421112381E-170;
    a = t * t;
  }
  absxk = std::abs(V_b[1]);
  if (absxk > scale)
  {
    t     = scale / absxk;
    a     = a * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    a += t * t;
  }
  absxk = std::abs(V_b[2]);
  if (absxk > scale)
  {
    t     = scale / absxk;
    a     = a * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    a += t * t;
  }
  a = scale * std::sqrt(a);
  if (a >= 0.0001)
  {
    *beta = std::asin(V_b[1] / a);
  }
  else
  {
    *beta = 0.0;
  }
  //  rho = 1.225;
  //  air density in 25 degree
  P = 0.592 * (a * a);
  //  the aerodynamic lift
  //  the aerodynamic drag
  //  the aerodynamic sideslip
  sb_tmp  = std::sin(*beta);
  cb_tmp  = std::cos(*beta);
  cb2_tmp = cb_tmp * cb_tmp;
  scale   = *beta;
  b       = std::isnan(*beta);
  if (!b)
  {
    if (*beta < 0.0)
    {
      scale = -1.0;
    }
    else
    {
      scale = (*beta > 0.0);
    }
  }
  comp_N_idx_0_tmp = sb_tmp * sb_tmp;
  comp_N_idx_0     = scale * comp_N_idx_0_tmp;
  absxk            = sb_tmp * cb_tmp;
  comp_N_idx_1_tmp = std::cos(*alpha);
  comp_N_idx_1     = absxk * comp_N_idx_1_tmp;
  comp_N_idx_2_tmp = std::sin(*alpha);
  comp_N_idx_2     = absxk * comp_N_idx_2_tmp;
  y_tmp = (1.0 - 1.0 / (std::exp(*alpha - -3.3161255787892263) + 1.0)) -
          1.0 / (std::exp(-(*alpha - -2.9670597283903604)) + 1.0);
  a_tmp = 1.0 - 1.0 / (std::exp(*alpha - -2.9670597283903604) + 1.0);
  a     = a_tmp - 1.0 / (std::exp(-(*alpha - -1.5707963267948966)) + 1.0);
  b_a   = (1.0 - 1.0 / (std::exp(*alpha - -1.5707963267948966) + 1.0)) -
        1.0 / (std::exp(-(*alpha - -0.19198621771937624)) + 1.0);
  b_a_tmp = 1.0 / (std::exp(-(*alpha - 2.9670597283903604)) + 1.0);
  c_a = (1.0 - 1.0 / (std::exp(*alpha - 0.3490658503988659) + 1.0)) - b_a_tmp;
  b_y_tmp = (1.0 - 1.0 / (std::exp(*alpha - 2.9670597283903604) + 1.0)) -
            1.0 / (std::exp(-(*alpha - 3.3161255787892263)) + 1.0);
  //  the rolling momentum
  scale = *beta;
  if (!b)
  {
    if (*beta < 0.0)
    {
      scale = -1.0;
    }
    else
    {
      scale = (*beta > 0.0);
    }
  }
  comp_N[0] = scale * comp_N_idx_0_tmp;
  comp_N[1] = absxk * comp_N_idx_1_tmp;
  comp_N[2] = absxk * comp_N_idx_2_tmp;
  d_a = a_tmp - 1.0 / (std::exp(-5.0 * (*alpha - -1.5707963267948966)) + 1.0);
  c_a_tmp = 1.0 / (std::exp(-0.5 * (*alpha - -0.3490658503988659)) + 1.0);
  e_a = (1.0 - 1.0 / (std::exp(5.0 * (*alpha - -1.5707963267948966)) + 1.0)) -
        c_a_tmp;
  t = 1.0 / (std::exp(-10.0 * (*alpha - 0.31415926535897931)) + 1.0);
  c_y_tmp =
      (1.0 - 1.0 / (std::exp(20.0 * (*alpha - -0.19198621771937624)) + 1.0)) -
      t;
  d_a_tmp = 1.0 - 1.0 / (std::exp(10.0 * (*alpha - 0.31415926535897931)) + 1.0);
  f_a = d_a_tmp - 1.0 / (std::exp(-5.0 * (*alpha - 1.5707963267948966)) + 1.0);
  g_a = (1.0 - 1.0 / (std::exp(5.0 * (*alpha - 1.5707963267948966)) + 1.0)) -
        b_a_tmp;
  //  the pitching momentum
  comp_N_tmp       = comp_N_idx_1_tmp * comp_N_idx_1_tmp;
  b_comp_N[0]      = cb2_tmp * comp_N_tmp;
  b_comp_N[1]      = comp_N_idx_0_tmp;
  b_comp_N_tmp     = comp_N_idx_2_tmp * comp_N_idx_2_tmp;
  b_comp_N[2]      = cb2_tmp * b_comp_N_tmp;
  comp_N_tmp_tmp   = std::abs(sb_tmp);
  b_comp_N_tmp_tmp = comp_N_tmp_tmp * cb_tmp;
  b_comp_N[3]      = b_comp_N_tmp_tmp * comp_N_idx_1_tmp;
  c_comp_N_tmp     = comp_N_idx_2_tmp * comp_N_idx_1_tmp;
  b_comp_N[4]      = c_comp_N_tmp * cb2_tmp;
  b_comp_N[5]      = b_comp_N_tmp_tmp * comp_N_idx_2_tmp;
  c_a_tmp          = a_tmp - c_a_tmp;
  d_a_tmp -= b_a_tmp;
  //  the yawing momentum
  scale = *beta;
  if (!b)
  {
    if (*beta < 0.0)
    {
      scale = -1.0;
    }
    else
    {
      scale = (*beta > 0.0);
    }
  }
  b_comp_N_idx_0 = scale * comp_N_idx_0_tmp;
  b_comp_N_idx_1 = absxk * comp_N_idx_1_tmp;
  b_comp_N_idx_2 = absxk * comp_N_idx_2_tmp;
  h_a = a_tmp - 1.0 / (std::exp(-10.0 * (*alpha - -0.17453292519943295)) + 1.0);
  y   = (1.0 - 1.0 / (std::exp(10.0 * (*alpha - -0.17453292519943295)) + 1.0)) -
      t;
  i_a = (1.0 - 1.0 / (std::exp(0.5 * *alpha) + 1.0)) - b_a_tmp;
  //  F is in the velocity frame(wind frame)
  //  convert to body axes
  //  http://www.mathworks.com/help/aeroblks/aerodynamicforcesandmoments.html
  scale       = rt_powd_snf(comp_N_idx_1_tmp, 3.0);
  c_comp_N[0] = cb2_tmp * scale;
  c_comp_N[1] = comp_N_idx_1_tmp * comp_N_idx_0_tmp;
  c_comp_N[2] = comp_N_idx_1_tmp * cb2_tmp * b_comp_N_tmp;
  c_comp_N[3] = comp_N_idx_2_tmp * comp_N_tmp * cb2_tmp;
  c_comp_N[4] = comp_N_idx_2_tmp * comp_N_idx_0_tmp;
  absxk       = rt_powd_snf(comp_N_idx_2_tmp, 3.0);
  c_comp_N[5] = absxk * cb2_tmp;
  c_comp_N[6] = comp_N_tmp_tmp * cb_tmp * comp_N_tmp;
  c_comp_N_tmp *= cb_tmp;
  c_comp_N[7] = c_comp_N_tmp * comp_N_tmp_tmp;
  c_comp_N[8] = b_comp_N_tmp_tmp * b_comp_N_tmp;
  b_a_tmp =
      a_tmp - 1.0 / (std::exp(-0.5 * (*alpha - -0.17453292519943295)) + 1.0);
  d_comp_N[0] = cb2_tmp * scale;
  d_comp_N[1] = comp_N_idx_1_tmp * comp_N_idx_0_tmp;
  d_comp_N[2] = comp_N_idx_1_tmp * cb2_tmp * b_comp_N_tmp;
  d_comp_N[3] = comp_N_idx_2_tmp * comp_N_tmp * cb2_tmp;
  d_comp_N[4] = comp_N_idx_2_tmp * comp_N_idx_0_tmp;
  d_comp_N[5] = absxk * cb2_tmp;
  d_comp_N[6] = comp_N_tmp_tmp * cb_tmp * comp_N_tmp;
  d_comp_N[7] = c_comp_N_tmp * comp_N_tmp_tmp;
  d_comp_N[8] = b_comp_N_tmp_tmp * b_comp_N_tmp;
  b_y[0]      = y_tmp * cb2_tmp;
  b_y[1]      = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  b_y[11]     = c_y_tmp * cb2_tmp;
  b_y[12]     = c_y_tmp * (*alpha * cb2_tmp);
  for (int i{0}; i < 9; i++)
  {
    t           = c_comp_N[i];
    b_y[i + 2]  = b_a_tmp * t;
    b_y[i + 13] = d_a_tmp * t;
  }
  b_y[22] = b_y_tmp * cb2_tmp;
  b_y[23] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  t       = 0.0;
  for (int i{0}; i < 24; i++)
  {
    t += dv[i] * b_y[i];
  }
  c_y[0]  = y_tmp * cb2_tmp;
  c_y[1]  = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  c_y[2]  = a * comp_N_idx_0;
  c_y[5]  = b_a * comp_N_idx_0;
  c_y[8]  = c_a * comp_N_idx_0;
  c_y[3]  = a * comp_N_idx_1;
  c_y[6]  = b_a * comp_N_idx_1;
  c_y[9]  = c_a * comp_N_idx_1;
  c_y[4]  = a * comp_N_idx_2;
  c_y[7]  = b_a * comp_N_idx_2;
  c_y[10] = c_a * comp_N_idx_2;
  c_y[11] = b_y_tmp * cb2_tmp;
  c_y[12] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  scale   = 0.0;
  for (int i{0}; i < 13; i++)
  {
    scale += dv1[i] * c_y[i];
  }
  b_y[0]  = y_tmp * cb2_tmp;
  b_y[1]  = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  b_y[11] = c_y_tmp * cb2_tmp;
  b_y[12] = c_y_tmp * (*alpha * cb2_tmp);
  for (int i{0}; i < 9; i++)
  {
    absxk       = d_comp_N[i];
    b_y[i + 2]  = c_a_tmp * absxk;
    b_y[i + 13] = d_a_tmp * absxk;
  }
  b_y[22] = b_y_tmp * cb2_tmp;
  b_y[23] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  absxk   = 0.0;
  for (int i{0}; i < 24; i++)
  {
    absxk += dv2[i] * b_y[i];
  }
  c_comp_N[0]  = comp_N_idx_1_tmp * cb_tmp;
  c_comp_N[3]  = -comp_N_idx_1_tmp * cb_tmp;
  c_comp_N[6]  = -comp_N_idx_2_tmp;
  c_comp_N[1]  = sb_tmp;
  c_comp_N[4]  = cb_tmp;
  c_comp_N[7]  = 0.0;
  c_comp_N[2]  = comp_N_idx_2_tmp * cb_tmp;
  c_comp_N[5]  = -comp_N_idx_2_tmp * sb_tmp;
  c_comp_N[8]  = comp_N_idx_1_tmp;
  comp_N_idx_0 = -(t * P);
  comp_N_idx_1 = scale * P;
  comp_N_idx_2 = -(absxk * P);
  d_y[0]       = y_tmp * cb2_tmp;
  d_y[1]       = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  d_y[8]       = c_y_tmp * cb2_tmp;
  d_y[9]       = c_y_tmp * (*alpha * cb2_tmp);
  for (int i{0}; i < 3; i++)
  {
    F_b[i] = (c_comp_N[i] * comp_N_idx_0 + c_comp_N[i + 3] * comp_N_idx_1) +
             c_comp_N[i + 6] * comp_N_idx_2;
    t           = comp_N[i];
    d_y[i + 2]  = d_a * t;
    d_y[i + 5]  = e_a * t;
    d_y[i + 10] = g_a * t;
    d_y[i + 13] = f_a * t;
  }
  d_y[16] = b_y_tmp * cb2_tmp;
  d_y[17] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  t       = 0.0;
  for (int i{0}; i < 18; i++)
  {
    t += dv3[i] * d_y[i];
  }
  d_y[0] = y_tmp * cb2_tmp;
  d_y[1] = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  d_y[8] = c_y_tmp * cb2_tmp;
  d_y[9] = c_y_tmp * (*alpha * cb2_tmp);
  for (int i{0}; i < 6; i++)
  {
    scale       = b_comp_N[i];
    d_y[i + 2]  = c_a_tmp * scale;
    d_y[i + 10] = d_a_tmp * scale;
  }
  d_y[16] = b_y_tmp * cb2_tmp;
  d_y[17] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  scale   = 0.0;
  for (int i{0}; i < 18; i++)
  {
    scale += dv4[i] * d_y[i];
  }
  e_y[0]  = y_tmp * cb2_tmp;
  e_y[1]  = y_tmp * ((*alpha + 3.1415926535897931) * cb2_tmp);
  e_y[5]  = y * cb2_tmp;
  e_y[6]  = y * (*alpha * cb2_tmp);
  e_y[2]  = h_a * b_comp_N_idx_0;
  e_y[7]  = i_a * b_comp_N_idx_0;
  e_y[3]  = h_a * b_comp_N_idx_1;
  e_y[8]  = i_a * b_comp_N_idx_1;
  e_y[4]  = h_a * b_comp_N_idx_2;
  e_y[9]  = i_a * b_comp_N_idx_2;
  e_y[10] = b_y_tmp * cb2_tmp;
  e_y[11] = b_y_tmp * ((*alpha - 3.1415926535897931) * cb2_tmp);
  absxk   = 0.0;
  for (int i{0}; i < 12; i++)
  {
    absxk += dv5[i] * e_y[i];
  }
  M_b[0] = t * P;
  M_b[1] = scale * P;
  M_b[2] = absxk * P;
}

//
// File trailer for Aerodynamics.cpp
//
// [EOF]
//
