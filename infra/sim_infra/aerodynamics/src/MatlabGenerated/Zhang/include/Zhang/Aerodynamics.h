
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
// File: Aerodynamics.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 17-Apr-2023 05:21:59
//

#ifndef AERODYNAMICS_H
#define AERODYNAMICS_H

// Include Files
// #include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
// Body-FLU force [N] and moment [N m]; speed [m/s], FLU alpha/beta [rad].
// The caller applies the model's low-speed fit guard to beta, not to telemetry.
extern void AerodynamicsFlu(double speed, double alpha, double beta,
                            double force[3], double moment[3]);

#endif
//
// File trailer for Aerodynamics.h
//
// [EOF]
//
