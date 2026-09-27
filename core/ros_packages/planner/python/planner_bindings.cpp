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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 *
 * pybind11 bindings for the pure planning core (no ROS).
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <planner_core/planner.hpp>

namespace py = pybind11;

namespace
{

/**
 * @brief Plan a reference trajectory from a planning yaml.
 *
 * @param yaml_path Path to a sectioned planning yaml
 * @return dict with arrays: t(N), p(N,3), v(N,3), a(N,3), yb(N,3),
 *         omega(N,3), thrust(N) and scalar duration
 */
py::dict plan(const std::string &yaml_path)
{
  planner::core::ProblemConfig config;
  config.load(yaml_path);

  planner::core::Planner planner(config);
  auto                    traj = planner.plan();
  if (!traj)
  {
    throw std::runtime_error("planner_core: planning failed");
  }

  const py::ssize_t n = static_cast<py::ssize_t>(traj->samples.size());

  const std::vector<py::ssize_t> shape1{n};
  const std::vector<py::ssize_t> shape2{n, 3};

  py::array_t<double> t(shape1);
  py::array_t<double> p(shape2);
  py::array_t<double> v(shape2);
  py::array_t<double> a(shape2);
  py::array_t<double> yb(shape2);
  py::array_t<double> omega(shape2);
  py::array_t<double> thrust(shape1);

  auto t_m      = t.mutable_unchecked<1>();
  auto p_m      = p.mutable_unchecked<2>();
  auto v_m      = v.mutable_unchecked<2>();
  auto a_m      = a.mutable_unchecked<2>();
  auto yb_m     = yb.mutable_unchecked<2>();
  auto omega_m  = omega.mutable_unchecked<2>();
  auto thrust_m = thrust.mutable_unchecked<1>();

  for (py::ssize_t i = 0; i < n; i++)
  {
    const auto &s = traj->samples[static_cast<std::size_t>(i)];
    t_m(i)        = s.t;
    thrust_m(i)   = s.thrust;
    for (int j = 0; j < 3; j++)
    {
      p_m(i, j)     = s.p[j];
      v_m(i, j)     = s.v[j];
      a_m(i, j)     = s.a[j];
      yb_m(i, j)    = s.yb[j];
      omega_m(i, j) = s.omega[j];
    }
  }

  py::dict out;
  out["t"]        = std::move(t);
  out["p"]        = std::move(p);
  out["v"]        = std::move(v);
  out["a"]        = std::move(a);
  out["yb"]       = std::move(yb);
  out["omega"]    = std::move(omega);
  out["thrust"]   = std::move(thrust);
  out["duration"] = traj->duration;
  return out;
}

}  // namespace

PYBIND11_MODULE(planner_bindings, m)
{
  m.doc() = "AP-PnC pure planning core (planner_core) Python bindings";
  m.def("plan", &plan, py::arg("yaml_path"),
        "Plan a trajectory from a planning yaml; returns numpy arrays");
}
