/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rtGetInf.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 17-Apr-2023 05:21:59
//

// Abstract:
//       MATLAB for code generation function to initialize non-finite, Inf and
//       MinusInf
// Include Files
#include "rtGetInf.h"

#include "rt_nonfinite.h"

// Function: rtGetInf
// ==================================================================
//  Abstract:
//  Initialize rtInf needed by the generated code.
real_T rtGetInf(void)
{
  return rtInf;
}

// Function: rtGetInfF
// =================================================================
//  Abstract:
//  Initialize rtInfF needed by the generated code.
real32_T rtGetInfF(void)
{
  return rtInfF;
}

// Function: rtGetMinusInf
// =============================================================
//  Abstract:
//  Initialize rtMinusInf needed by the generated code.
real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

// Function: rtGetMinusInfF
// ============================================================
//  Abstract:
//  Initialize rtMinusInfF needed by the generated code.
real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

//
// File trailer for rtGetInf.cpp
//
// [EOF]
//
