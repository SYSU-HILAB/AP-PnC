/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Description:
 * Pybind11 Python bindings for aerodynamics classes.
 * Provides direct Python access to C++ aerodynamic coefficient computation.
 */

#include <pybind11/eigen.h>  // For Eigen matrix conversions
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>    // For STL conversions

#include <aerodynamics/advanced_lift_drag.hpp>
#include <aerodynamics/bspline_aerodynamics.hpp>
#include <aerodynamics/non_aerodynamics.hpp>
#include <aerodynamics/phi_aerodynamics.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>
#include <memory>
#include <string>

namespace py = pybind11;

using namespace aerodynamics;

// Wrapper class for aerodynamics models that provides a simple Python API
class PyAerodynamicsModel
{
 public:
  PyAerodynamicsModel(std::unique_ptr<AerodynamicsCreator> creator)
      : _creator(std::move(creator))
  {
    if (!_creator->createAerodynamics())
    {
      throw std::runtime_error("Failed to create aerodynamics model");
    }
  }

  /**
   * Compute aerodynamic force coefficients.
   *
   * @param alpha Angle of attack in radians
   * @param speed Airspeed magnitude in m/s (default: 12.0)
   * @return Tuple of (cx, cz) force coefficients
   */
  std::tuple<double, double> compute_coefficients(double alpha, double speed = 12.0)
  {
    // Compute body velocity from alpha
    Eigen::Vector3d body_speed;
    body_speed.x() = speed * std::cos(alpha);
    body_speed.y() = 0.0;  // No sideslip
    body_speed.z() = speed * std::sin(alpha);

    Eigen::Vector3d aero_force_b;
    Eigen::Vector3d aero_moment_b;

    // Output parameters (computed by aerodynamics model)
    double alpha_out = 0.0;
    double beta_out  = 0.0;

    _creator->getAeroWrench(body_speed, aero_force_b, aero_moment_b, alpha_out, beta_out);

    // Normalize coefficients: force / (speed^2)
    const double cx = aero_force_b.x() / (speed * speed);
    const double cz = aero_force_b.z() / (speed * speed);

    return {cx, cz};
  }

  /**
   * Compute aerodynamic force coefficients for multiple angles.
   *
   * @param alphas Array of angles of attack in radians
   * @param speed Airspeed magnitude in m/s (default: 12.0)
   * @return Tuple of (cx_array, cz_array) force coefficients
   */
  std::tuple<Eigen::VectorXd, Eigen::VectorXd> compute_coefficients_array(
      const Eigen::VectorXd& alphas,
      double speed = 12.0)
  {
    const int n = alphas.size();
    Eigen::VectorXd cx(n);
    Eigen::VectorXd cz(n);

    for (int i = 0; i < n; ++i)
    {
      std::tie(cx(i), cz(i)) = compute_coefficients(alphas(i), speed);
    }

    return {cx, cz};
  }

 private:
  std::unique_ptr<AerodynamicsCreator> _creator;
};

// Factory functions for creating aerodynamics models
std::shared_ptr<PyAerodynamicsModel> create_lyu_model()
{
  return std::make_shared<PyAerodynamicsModel>(std::make_unique<LyuAerodynamicsCreator>());
}

std::shared_ptr<PyAerodynamicsModel> create_bspline_model()
{
  return std::make_shared<PyAerodynamicsModel>(
      std::make_unique<BsplineAerodynamicsCreator>());
}

std::shared_ptr<PyAerodynamicsModel> create_phi_model()
{
  return std::make_shared<PyAerodynamicsModel>(std::make_unique<PhiAerodynamicsCreator>());
}

std::shared_ptr<PyAerodynamicsModel> create_advanced_lift_drag_model()
{
  return std::make_shared<PyAerodynamicsModel>(
      std::make_unique<AdvancedLiftDragCreator>());
}

std::shared_ptr<PyAerodynamicsModel> create_non_aerodynamics_model()
{
  return std::make_shared<PyAerodynamicsModel>(std::make_unique<NonAerodynamicsCreator>());
}

PYBIND11_MODULE(aerodynamics_bindings, m)
{
  m.doc() = "Python bindings for AP-PnC aerodynamics models";

  // Expose the PyAerodynamicsModel class
  py::class_<PyAerodynamicsModel>(m, "AerodynamicsModel")
      .def(
          "compute_coefficients",
          &PyAerodynamicsModel::compute_coefficients,
          py::arg("alpha"),
          py::arg("speed") = 12.0,
          "Compute force coefficients (cx, cz) for a single angle of attack")
      .def(
          "compute_coefficients_array",
          &PyAerodynamicsModel::compute_coefficients_array,
          py::arg("alphas"),
          py::arg("speed") = 12.0,
          "Compute force coefficients (cx, cz) for multiple angles of attack");

  // Factory functions
  m.def(
      "create_lyu_model",
      &create_lyu_model,
      "Create Lyu (Zhang-Lyu) aerodynamics model");

  m.def(
      "create_bspline_model",
      &create_bspline_model,
      "Create B-spline aerodynamics model");

  m.def(
      "create_phi_model",
      &create_phi_model,
      "Create Phi-theory aerodynamics model");

  m.def(
      "create_advanced_lift_drag_model",
      &create_advanced_lift_drag_model,
      "Create Advanced Lift-Drag aerodynamics model");

  m.def(
      "create_non_aerodynamics_model",
      &create_non_aerodynamics_model,
      "Create Non-aerodynamics (null) model");
}
