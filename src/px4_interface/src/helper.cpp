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

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

// ANSI escape codes for colors
#define RESET "\033[0m"     // Reset to default
#define BLACK "\033[30m"    // Black text
#define RED "\033[31m"      // Red text
#define GREEN "\033[32m"    // Green text
#define YELLOW "\033[33m"   // Yellow text
#define BLUE "\033[34m"     // Blue text
#define MAGENTA "\033[35m"  // Magenta text
#define CYAN "\033[36m"     // Cyan text
#define WHITE "\033[37m"    // White text

// Define a struct for the cost function with automatic differentiation
struct YawCostFunctor
{
  static ceres::Solver::Options options;

  static ceres::Solver::Summary summary;
  static Eigen::Quaterniond     base_q_;  // The target quaternion

  YawCostFunctor(const Eigen::Quaterniond& q_i) : q_i_(q_i) {}

  static double constructProblemAndSolve(
      std::vector<Eigen::Quaterniond>& quaternions)
  {
    double         theta = 0;
    ceres::Problem problem;

    // Add the yaw cost functor for each quaternion in the series
    for (size_t i = 0; i < quaternions.size(); ++i)
    {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<YawCostFunctor, 1, 1>(
              new YawCostFunctor(quaternions[i]));
      problem.AddResidualBlock(cost_function, nullptr, &theta);
    }

    ceres::Solve(options, &problem, &summary);
    return theta;
  }

  template <typename T>
  bool operator()(const T* const theta, T* residual) const
  {
    // Create the yaw quaternion for the given theta
    T cos_half_theta = ceres::cos(theta[0] / T(2.0));
    T sin_half_theta = ceres::sin(theta[0] / T(2.0));

    Eigen::Quaternion<T> q_yaw(cos_half_theta, T(0.0), T(0.0), sin_half_theta);

    // Apply yaw quaternion to the target quaternion
    Eigen::Quaternion<T> transformed_q = base_q_.cast<T>() * q_yaw;

    // Compute the difference between the transformed quaternion and q_i_
    Eigen::Quaternion<T> relative_q = transformed_q.inverse() * q_i_.cast<T>();
    // Compute the residual as the norm of the difference
    residual[0] =
        T(2.0) * ceres::atan2(relative_q.vec().norm(), relative_q.w());

    return true;
  }

  const Eigen::Quaterniond q_i_;  // The i-th quaternion in the series
};

ceres::Solver::Options YawCostFunctor::options = []()
{
  ceres::Solver::Options opt;
  // Customize 'options' if needed
  opt.max_num_iterations = 100;
  opt.linear_solver_type = ceres::DENSE_QR;
  return opt;
}();

Eigen::Quaterniond YawCostFunctor::base_q_ = Eigen::Quaterniond(0, 1, 0, 0);

ceres::Solver::Summary YawCostFunctor::summary;

struct PositionCostFunctor
{
  // Constructor takes a single collected position
  PositionCostFunctor(const Eigen::Vector3d& pos_collecter_i)
      : pos_collecter_i_(pos_collecter_i)
  {
  }

  // Define the cost function using Ceres' AutoDiff mechanism
  template <typename T>
  bool operator()(const T* const pos_dec, T* residual) const
  {
    // The residual is the squared distance between the current estimated
    // position (pos_dec) and the collected position (pos_collecter_i_)
    residual[0] = T(0);
    for (int i = 0; i < 3; ++i)
    {
      residual[0] += ceres::pow(pos_dec[i] - T(pos_collecter_i_(i)), 2);
    }
    return true;
  }

  const Eigen::Vector3d pos_collecter_i_;

  // Solver options and summary (static)
  static ceres::Solver::Options options;
  static ceres::Solver::Summary summary;

  // Function to construct the problem and solve it
  static Eigen::Vector3d constructProblemAndSolve(
      const std::vector<Eigen::Vector3d>& pos_collecter_)
  {
    Eigen::Vector3d pos_dec(0.0, 0.0, 0.0);  // Initial guess for pos_dec
    ceres::Problem  problem;

    // Add the position cost functor for each collected position
    for (const auto& pos_collecter_i : pos_collecter_)
    {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<PositionCostFunctor, 1, 3>(
              new PositionCostFunctor(pos_collecter_i));
      problem.AddResidualBlock(cost_function, nullptr, pos_dec.data());
    }

    // Solve the optimization problem
    ceres::Solve(options, &problem, &summary);
    return pos_dec;  // Return the optimized position
  }
};

// Initialize static members of the struct
ceres::Solver::Options PositionCostFunctor::options = []()
{
  ceres::Solver::Options opt;
  // Customize 'options' if needed
  opt.max_num_iterations = 100;
  opt.linear_solver_type = ceres::DENSE_QR;
  return opt;
}();
;
ceres::Solver::Summary PositionCostFunctor::summary;
