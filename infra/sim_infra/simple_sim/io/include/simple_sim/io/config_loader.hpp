#pragma once

#include <filesystem>
#include <string>

#include "simple_sim/adapters/aero_model.hpp"
#include "simple_sim/adapters/controllers.hpp"
#include "simple_sim/config.hpp"

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
    Se3Gains              se3;
    AeroConfig            aero;
    std::string           aero_model;
    std::string           controller;
    std::int64_t          physics_dt_ns = 0;
    double                duration_s =
        0.0;  // 0 -> maximum duration with full horizon coverage
    std::int64_t publish_stride  = 5;
    double       realtime_factor = 1.0;  // ROS scheduler only; 0 means unpaced
  };

  std::filesystem::path project_root();
  std::filesystem::path bringup_config(const std::filesystem::path &root,
                                       const std::string           &name);
  RuntimeConfig         load_config(const std::filesystem::path &sim_yaml = {},
                                    const std::string &controller_override = {});

}  // namespace simple_sim
