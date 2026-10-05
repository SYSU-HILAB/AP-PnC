#include "simple_sim/io/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <set>
#include <stdexcept>

namespace simple_sim
{
  namespace fs = std::filesystem;
  namespace
  {
    Eigen::Vector3d vector3(const YAML::Node &node)
    {
      if (!node.IsSequence() || node.size() != 3)
        throw std::invalid_argument("expected three-element vector");
      Eigen::Vector3d v(node[0].as<double>(), node[1].as<double>(),
                        node[2].as<double>());
      if (!v.allFinite())
        throw std::invalid_argument("non-finite configuration vector");
      return v;
    }
  }  // namespace
  fs::path project_root()
  {
    const char    *env  = std::getenv("AP_PNC_DIR");
    const fs::path root = env ? fs::path(env) : fs::path(AP_PNC_SOURCE_ROOT);
    if (!root.is_absolute() || !fs::is_directory(root))
      throw std::invalid_argument(
          "AP_PNC_DIR must be an existing absolute project root");
    return fs::canonical(root);
  }
  fs::path bringup_config(const fs::path &root, const std::string &name)
  {
    for (const auto &dir :
         {root / "bringup" / "config", root / "core" / "bringup" / "config"})
    {
      const auto path = dir / name;
      if (fs::is_regular_file(path))
        return fs::canonical(path);
    }
    throw std::runtime_error("bringup config not found: " + name);
  }
  RuntimeConfig load_config(const fs::path    &path,
                            const std::string &override_controller)
  {
    RuntimeConfig cfg;
    cfg.root = project_root();
    if (!path.empty() && !path.is_absolute())
      throw std::invalid_argument("config path must be absolute");
    cfg.sim_yaml = path.empty() ? bringup_config(cfg.root, "simple_sim.yaml")
                                : fs::canonical(path);
    cfg.planning_yaml = bringup_config(cfg.root, "planning.yaml");
    cfg.nmpc_yaml     = bringup_config(cfg.root, "nmpc.yaml");
    const auto n      = YAML::LoadFile(cfg.sim_yaml.string())["simple_sim"];
    cfg.controller    = override_controller.empty()
                            ? n["controller"].as<std::string>()
                            : override_controller;
    if (cfg.controller != "nmpc" && cfg.controller != "se3")
      throw std::invalid_argument("controller must be nmpc or se3");
    cfg.physics_dt_ns   = n["physics_dt_ns"].as<std::int64_t>();
    cfg.duration_s      = n["duration_s"].as<double>();
    cfg.publish_stride  = n["ros"]["publish_stride"].as<std::int64_t>();
    cfg.realtime_factor = n["ros"]["realtime_factor"].as<double>();
    if (cfg.publish_stride <= 0 || !std::isfinite(cfg.realtime_factor) ||
        cfg.realtime_factor < 0.0)
      throw std::invalid_argument(
          "invalid ROS observation/pacing configuration");
    cfg.plant.mass    = n["plant"]["mass"].as<double>();
    cfg.plant.gravity = n["plant"]["gravity"].as<double>();
    cfg.plant.inertia = vector3(n["plant"]["inertia"]);
    cfg.plant.validate();
    cfg.rate_loop.gain       = vector3(n["rate_loop"]["gain"]);
    cfg.rate_loop.max_rate   = vector3(n["rate_loop"]["max_rate"]);
    cfg.rate_loop.max_moment = vector3(n["rate_loop"]["max_moment"]);
    cfg.rate_loop.max_specific_force =
        n["rate_loop"]["max_specific_force"].as<double>();
    cfg.rate_loop.validate();
    if (n["actuator"])
      cfg.actuator_model = n["actuator"]["model"].as<std::string>();
    if (cfg.actuator_model != "ideal_rate_loop" &&
        cfg.actuator_model != "matlab_vtol")
      throw std::invalid_argument("unknown actuator model");
    if (cfg.actuator_model == "matlab_vtol")
    {
      const auto m    = n["actuator"];
      auto       four = [](const YAML::Node &node)
      {
        if (!node.IsSequence() || node.size() != 4)
          throw std::invalid_argument("expected four-vector");
        Eigen::Vector4d v;
        for (int i = 0; i < 4; ++i)
          v[i] = node[i].as<double>();
        return v;
      };
      auto &c        = cfg.motor_model;
      c.arms         = four(m["arms_m"]);
      c.arm_angles   = four(m["arm_angles_rad"]);
      c.tilts        = four(m["tilts_rad"]);
      c.thrust_poly  = vector3(m["thrust_poly"]);
      c.torque_poly  = vector3(m["torque_poly"]);
      c.esc_poly     = vector3(m["esc_poly"]);
      c.damping_wing = vector3(m["damping_wing"]);
      c.kp           = vector3(m["kp"]);
      c.ki           = vector3(m["ki"]);
      c.kd           = vector3(m["kd"]);
      c.tau          = m["motor_tau_s"].as<double>();
      c.diameter     = m["diameter_m"].as<double>();
      c.ct           = m["ct"].as<double>();
      c.cq           = m["cq"].as<double>();
      c.jm           = m["jm"].as<double>();
      c.pwm_min      = m["pwm_min"].as<double>();
      c.pwm_max      = m["pwm_max"].as<double>();
      c.inner_dt_ns  = m["inner_dt_ns"].as<std::int64_t>();
      if (m["mixer"].size() != 4)
        throw std::invalid_argument("expected 4x4 mixer");
      for (int i = 0; i < 4; ++i)
        c.mixer.row(i) = four(m["mixer"][i]).transpose();
      c.validate();
    }
    cfg.se3.position    = vector3(n["se3"]["position_gain"]);
    cfg.se3.velocity    = vector3(n["se3"]["velocity_gain"]);
    cfg.se3.attitude    = vector3(n["se3"]["attitude_gain"]);
    cfg.aero_model      = n["aero"]["model"].as<std::string>();
    cfg.aero.wind_world = vector3(n["aero"]["wind_world"]);
    cfg.aero.scale      = n["aero"]["scale"].as<double>();
    // Canonical model names, shared with the aerodynamics factory
    // (sim::gazebo::<model> and sim::simple::<model> use the same set).
    static const std::set<std::string> aero_models{"lyu", "phi", "ma",
                                                   "advanced", "none"};
    if (!aero_models.count(cfg.aero_model) || !std::isfinite(cfg.aero.scale) ||
        cfg.aero.scale < 0.0)
      throw std::invalid_argument(
          "invalid aero model (lyu | phi | ma | advanced | none) or scale");
    if (cfg.physics_dt_ns <= 0 || !std::isfinite(cfg.duration_s) ||
        cfg.duration_s < 0.0)
      throw std::invalid_argument("invalid simulation period/duration");
    return cfg;
  }
}  // namespace simple_sim
