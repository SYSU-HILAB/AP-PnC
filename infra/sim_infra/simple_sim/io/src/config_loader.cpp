#include "simple_sim/io/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <aerodynamics/aero_models.hpp>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace simple_sim
{
  namespace fs = std::filesystem;
  namespace
  {
    Eigen::Vector3d read_vector3(const YAML::Node &node)
    {
      if (!node.IsSequence() || node.size() != 3)
        throw std::invalid_argument("expected three-element vector");
      Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(),
                            node[2].as<double>());
      if (!value.allFinite())
        throw std::invalid_argument("non-finite configuration vector");
      return value;
    }
    Eigen::Vector4d read_vector4(const YAML::Node &node)
    {
      if (!node.IsSequence() || node.size() != 4)
        throw std::invalid_argument("expected four-element vector");
      Eigen::Vector4d value;
      for (int i = 0; i < 4; ++i)
        value[i] = node[i].as<double>();
      if (!value.allFinite())
        throw std::invalid_argument("non-finite configuration vector");
      return value;
    }
  }  // namespace
  fs::path project_root()
  {
    const char    *env_var = std::getenv("AP_PNC_DIR");
    const fs::path root =
        env_var ? fs::path(env_var) : fs::path(AP_PNC_SOURCE_ROOT);
    if (!root.is_absolute() || !fs::is_directory(root))
      throw std::invalid_argument(
          "AP_PNC_DIR must be an existing absolute project root");
    return fs::canonical(root);
  }
  fs::path bringup_config(const fs::path &root, const std::string &filename)
  {
    for (const auto &directory :
         {root / "bringup" / "config", root / "core" / "bringup" / "config"})
    {
      const auto candidate = directory / filename;
      if (fs::is_regular_file(candidate))
        return fs::canonical(candidate);
    }
    throw std::runtime_error("bringup config not found: " + filename);
  }
  RuntimeConfig load_config(const fs::path    &path,
                            const std::string &override_controller)
  {
    RuntimeConfig config;
    config.root = project_root();
    if (!path.empty() && !path.is_absolute())
      throw std::invalid_argument("config path must be absolute");
    config.sim_yaml      = path.empty()
                               ? bringup_config(config.root, "simple_sim.yaml")
                               : fs::canonical(path);
    config.planning_yaml = bringup_config(config.root, "planning.yaml");
    config.nmpc_yaml     = bringup_config(config.root, "nmpc.yaml");

    const YAML::Node sim =
        YAML::LoadFile(config.sim_yaml.string())["simple_sim"];
    config.controller = override_controller.empty()
                            ? sim["controller"].as<std::string>()
                            : override_controller;
    if (config.controller != "nmpc" && config.controller != "se3")
      throw std::invalid_argument("controller must be nmpc or se3");
    config.physics_dt_ns   = sim["physics_dt_ns"].as<std::int64_t>();
    config.duration_s      = sim["duration_s"].as<double>();
    config.terminal_hold_s = sim["terminal_hold_s"].as<double>(0.0);
    if (sim["arm_length_scale"])
    {
      config.arm_length_scale = sim["arm_length_scale"].as<double>();
      if (!std::isfinite(config.arm_length_scale) ||
          config.arm_length_scale <= 0.0)
        throw std::invalid_argument("arm_length_scale must be positive/finite");
    }
    config.publish_stride  = sim["ros"]["publish_stride"].as<std::int64_t>();
    config.realtime_factor = sim["ros"]["realtime_factor"].as<double>();
    if (config.publish_stride <= 0 || !std::isfinite(config.realtime_factor) ||
        config.realtime_factor < 0.0)
      throw std::invalid_argument(
          "invalid ROS observation/pacing configuration");

    config.plant.mass    = sim["plant"]["mass"].as<double>();
    config.plant.gravity = sim["plant"]["gravity"].as<double>();
    config.plant.inertia = read_vector3(sim["plant"]["inertia"]);
    config.plant.validate();

    config.rate_loop.gain       = read_vector3(sim["rate_loop"]["gain"]);
    config.rate_loop.max_rate   = read_vector3(sim["rate_loop"]["max_rate"]);
    config.rate_loop.max_moment = read_vector3(sim["rate_loop"]["max_moment"]);
    config.rate_loop.max_specific_force =
        sim["rate_loop"]["max_specific_force"].as<double>();
    config.rate_loop.validate();

    if (sim["actuator"])
      config.actuator_model = sim["actuator"]["model"].as<std::string>();
    if (config.actuator_model != "ideal_rate_loop" &&
        config.actuator_model != "practical")
      throw std::invalid_argument("unknown actuator model");
    if (config.actuator_model == "practical")
    {
      const YAML::Node  actuator = sim["actuator"];
      MotorModelConfig &motor    = config.motor_model;
      motor.arms                 = read_vector4(actuator["arms_m"]);
      motor.arm_angles           = read_vector4(actuator["arm_angles_rad"]);
      motor.tilts                = read_vector4(actuator["tilts_rad"]);
      motor.thrust_poly          = read_vector3(actuator["thrust_poly"]);
      motor.torque_poly          = read_vector3(actuator["torque_poly"]);
      motor.esc_poly             = read_vector3(actuator["esc_poly"]);
      motor.damping_wing         = read_vector3(actuator["damping_wing"]);
      motor.kp                   = read_vector3(actuator["kp"]);
      motor.ki                   = read_vector3(actuator["ki"]);
      motor.kd                   = read_vector3(actuator["kd"]);
      motor.tau                  = actuator["motor_tau_s"].as<double>();
      motor.diameter             = actuator["diameter_m"].as<double>();
      motor.ct                   = actuator["ct"].as<double>();
      motor.cq                   = actuator["cq"].as<double>();
      motor.jm                   = actuator["jm"].as<double>();
      motor.pwm_min              = actuator["pwm_min"].as<double>();
      motor.pwm_max              = actuator["pwm_max"].as<double>();
      motor.inner_dt_ns          = actuator["inner_dt_ns"].as<std::int64_t>();
      if (actuator["mixer"].size() != 4)
        throw std::invalid_argument("expected 4x4 mixer");
      for (int row = 0; row < 4; ++row)
        motor.mixer.row(row) = read_vector4(actuator["mixer"][row]).transpose();
      // `arms_m` is the motor-to-CoM lever that scales the position-based
      // moments (pitch/yaw, plus the tilt-driven roll). The extracted private
      // profile is never rewritten, so an explicit positive scale is required
      // to change that geometry.
      motor.arms *= config.arm_length_scale;
      motor.validate();
    }

    config.se3.position    = read_vector3(sim["se3"]["position_gain"]);
    config.se3.velocity    = read_vector3(sim["se3"]["velocity_gain"]);
    config.se3.attitude    = read_vector3(sim["se3"]["attitude_gain"]);
    config.aero_model      = sim["aero"]["model"].as<std::string>();
    config.aero.wind_world = read_vector3(sim["aero"]["wind_world"]);
    config.aero.scale      = sim["aero"]["scale"].as<double>();
    config.aero.lateral_moment_only =
        sim["aero"]["lateral_moment_only"].as<bool>(false);
    // Canonical model names and their project validation status come from the
    // single table in aerodynamics/aero_models.hpp.
    const auto *entry = aerodynamics::find_aero_model(config.aero_model);
    if (entry == nullptr || !std::isfinite(config.aero.scale) ||
        config.aero.scale < 0.0)
      throw std::invalid_argument("invalid aero model name or scale");
    config.aero_validation = aerodynamics::validation_label(entry->status);
    if (config.physics_dt_ns <= 0 || !std::isfinite(config.duration_s) ||
        config.duration_s < 0.0 || !std::isfinite(config.terminal_hold_s) ||
        config.terminal_hold_s < 0.0)
      throw std::invalid_argument("invalid simulation period/duration");
    return config;
  }
}  // namespace simple_sim
