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
 *
 * pybind11 bindings for the pure planning core (no ROS).
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <cmath>
#include <limits>
#include <planner_core/planner.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace
{

  /**
   * @brief Plan a reference trajectory from a planning yaml.
   *
   * @param yaml_path Absolute path to a sectioned planning yaml
   * @param sample_dt Export interval [s]; the exact terminal point is included
   * @return dict with arrays: t(N), p(N,3), v(N,3), a(N,3), yb(N,3),
   *         omega(N,3), thrust(N) and scalar duration
   */
  py::dict plan(const std::string &yaml_path, double sample_dt)
  {
    if (!std::isfinite(sample_dt) || sample_dt <= 0.0)
      throw std::invalid_argument("sample_dt must be finite and positive");
    planner::core::ProblemConfig config;
    config.load(yaml_path);

    planner::core::Planner planner(config);
    auto                   traj = planner.plan();
    if (!traj || traj->empty())
    {
      throw std::runtime_error("planner_core: planning failed");
    }

    const double duration  = traj->duration();
    const double intervals = std::ceil(duration / sample_dt);
    // Bound array byte counts before converting the floating-point grid size.
    const auto max_points =
        std::numeric_limits<py::ssize_t>::max() / (3 * sizeof(double));
    if (!std::isfinite(duration) || !std::isfinite(intervals) ||
        intervals >= static_cast<double>(max_points))
      throw std::invalid_argument("sample_dt produces too many samples");
    std::vector<double> times;
    times.reserve(static_cast<std::size_t>(intervals) + 1);
    for (py::ssize_t i = 0; i < static_cast<py::ssize_t>(intervals); ++i)
    {
      const double time = i * sample_dt;
      if (time >= duration)
        break;
      times.push_back(time);
    }
    times.push_back(duration);
    const py::ssize_t n = static_cast<py::ssize_t>(times.size());

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
      const auto s = traj->sample(times[static_cast<std::size_t>(i)]);
      t_m(i)       = s.t;
      thrust_m(i)  = s.thrust;
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
    out["duration"] = duration;
    return out;
  }

}  // namespace

PYBIND11_MODULE(planner_bindings, m)
{
  m.doc() = "AP-PnC pure planning core (planner_core) Python bindings";
  m.def("plan", &plan, py::arg("yaml_path"), py::arg("sample_dt") = 0.02,
        "Plan from an absolute YAML path and sample the continuous trajectory. "
        "sample_dt defaults to 0.02 s; the exact terminal point is included.");
}
