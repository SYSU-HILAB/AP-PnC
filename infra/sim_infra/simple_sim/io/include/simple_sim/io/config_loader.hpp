#pragma once

#include <filesystem>
#include <string>

#include "simple_sim/adapters/aero_model.hpp"
#include "simple_sim/adapters/controllers.hpp"
#include "simple_sim/config.hpp"
#include "simple_sim/motor_model.hpp"
#include "simple_sim/throttle_model.hpp"

namespace simple_sim
{

  struct RuntimeConfig
  {
    std::filesystem::path root;
    std::filesystem::path sim_yaml;
    std::filesystem::path planning_yaml;
    std::filesystem::path nmpc_yaml;
    PlantConfig           plant;
    RateLoopConfig        rate_loop;
    MotorModelConfig      motor_model;
    ThrottleModelConfig   throttle;
    std::string           actuator_model = "ideal_rate_loop";
    Se3Gains              se3;
    AeroConfig            aero;
    std::string           aero_model;
    // Project validation status of `aero_model` (aerodynamics/aero_models.hpp).
    std::string  aero_validation;
    std::string  controller;
    std::int64_t physics_dt_ns = 0;
    double       duration_s = 0.0;  // 0 -> full trajectory plus terminal_hold_s
    double       terminal_hold_s =
        0.0;  // auto-duration extension; final point repeats
    std::int64_t publish_stride  = 5;
    double       realtime_factor = 1.0;  // ROS scheduler only; 0 means unpaced
    // Explicit study override of the extracted private geometry. Recorded in
    // the manifest so a plant result is never confused with the bare profile.
    double arm_length_scale = 1.0;
  };

  std::filesystem::path project_root();
  std::filesystem::path bringup_config(const std::filesystem::path &root,
                                       const std::string           &name);
  RuntimeConfig         load_config(const std::filesystem::path &sim_yaml = {},
                                    const std::string &controller_override = {});

}  // namespace simple_sim
