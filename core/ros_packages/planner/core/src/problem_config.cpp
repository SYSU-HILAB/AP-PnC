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
 */

#include <yaml-cpp/yaml.h>

#include <planner_core/problem_config.hpp>

namespace planner::core
{

  namespace
  {
    /**
     * Read a yaml key with a fallback default when absent.
     *
     * @param[in] node Parent yaml node [-]
     * @param[in] key  Entry name [-]
     * @param[in] def  Fallback value [same as entry]
     * @return Parsed value [same as entry]
     */
    template <typename T>
    T get_or(const YAML::Node &node, const char *key, T def)
    {
      if (node && node[key])
      {
        return node[key].as<T>();
      }
      return def;
    }
  }  // namespace

  void ProblemConfig::load(const std::string &yaml_path)
  {
    YAML::Node root = YAML::LoadFile(yaml_path);

    const YAML::Node problem_node = root["problem_formulation"];
    problem.piece_n = get_or<int>(problem_node, "piece_n", problem.piece_n);

    const YAML::Node box_node = problem_node["box"];
    problem.box.side_inside =
        get_or(box_node, "side_inside", problem.box.side_inside);
    problem.box.side_outside =
        get_or(box_node, "side_outside", problem.box.side_outside);
    problem.box.up    = get_or(box_node, "up", problem.box.up);
    problem.box.down  = get_or(box_node, "down", problem.box.down);
    problem.box.front = get_or(box_node, "front", problem.box.front);
    problem.box.back  = get_or(box_node, "back", problem.box.back);

    const YAML::Node basic_node = problem_node["basic_traj"];
    problem.basic_traj.type =
        get_or(basic_node, "type", problem.basic_traj.type);
    problem.basic_traj.radius =
        get_or(basic_node, "radius", problem.basic_traj.radius);
    problem.basic_traj.height =
        get_or(basic_node, "height", problem.basic_traj.height);
    problem.basic_traj.rate =
        get_or(basic_node, "rate", problem.basic_traj.rate);
    problem.basic_traj.curvature_factor = get_or(
        basic_node, "curvature_factor", problem.basic_traj.curvature_factor);
    problem.basic_traj.wave_len =
        get_or(basic_node, "wave_len", problem.basic_traj.wave_len);

    const YAML::Node cost_node = problem_node["cost"];
    problem.cost.v_max         = get_or(cost_node, "v_max", problem.cost.v_max);
    problem.cost.omg_x_max =
        get_or(cost_node, "omg_x_max", problem.cost.omg_x_max);
    problem.cost.omg_y_max =
        get_or(cost_node, "omg_y_max", problem.cost.omg_y_max);
    problem.cost.omg_z_max =
        get_or(cost_node, "omg_z_max", problem.cost.omg_z_max);
    problem.cost.acc_max = get_or(cost_node, "acc_max", problem.cost.acc_max);
    problem.cost.thrust_min =
        get_or(cost_node, "thrust_min", problem.cost.thrust_min);
    problem.cost.thrust_max =
        get_or(cost_node, "thrust_max", problem.cost.thrust_max);
    problem.cost.vel_weight =
        get_or(cost_node, "vel_weight", problem.cost.vel_weight);
    problem.cost.acc_weight =
        get_or(cost_node, "acc_weight", problem.cost.acc_weight);
    problem.cost.omg_x_weight =
        get_or(cost_node, "omg_x_weight", problem.cost.omg_x_weight);
    problem.cost.omg_y_weight =
        get_or(cost_node, "omg_y_weight", problem.cost.omg_y_weight);
    problem.cost.omg_z_weight =
        get_or(cost_node, "omg_z_weight", problem.cost.omg_z_weight);
    problem.cost.thrust_weight =
        get_or(cost_node, "thrust_weight", problem.cost.thrust_weight);
    problem.cost.time_weight =
        get_or(cost_node, "time_weight", problem.cost.time_weight);
    problem.cost.omg_consistent_weight = get_or(
        cost_node, "omg_consistent_weight", problem.cost.omg_consistent_weight);

    const YAML::Node minco_node = problem_node["minco"];
    problem.minco.smoothing_eps =
        get_or(minco_node, "smoothing_eps", problem.minco.smoothing_eps);
    problem.minco.integral_intervs =
        get_or(minco_node, "integral_intervs", problem.minco.integral_intervs);
    problem.minco.rel_cost_tol =
        get_or(minco_node, "rel_cost_tol", problem.minco.rel_cost_tol);

    const YAML::Node flatness_node = problem_node["flatness"];
    problem.flatness.phi =
        get_or<std::vector<double>>(flatness_node, "phi", problem.flatness.phi);
    problem.flatness.mass =
        get_or(flatness_node, "mass", problem.flatness.mass);

    const YAML::Node io_node = root["io"];
    io.traj_pub_rsl = get_or(io_node, "traj_pub_rsl", io.traj_pub_rsl);

    // Mirror cost settings into the gcopter singleton it actually consumes
    cost_config.v_max                 = problem.cost.v_max;
    cost_config.omg_x_max             = problem.cost.omg_x_max;
    cost_config.omg_y_max             = problem.cost.omg_y_max;
    cost_config.omg_z_max             = problem.cost.omg_z_max;
    cost_config.acc_max               = problem.cost.acc_max;
    cost_config.thrust_min            = problem.cost.thrust_min;
    cost_config.thrust_max            = problem.cost.thrust_max;
    cost_config.vel_weight            = problem.cost.vel_weight;
    cost_config.acc_weight            = problem.cost.acc_weight;
    cost_config.omg_x_weight          = problem.cost.omg_x_weight;
    cost_config.omg_y_weight          = problem.cost.omg_y_weight;
    cost_config.omg_z_weight          = problem.cost.omg_z_weight;
    cost_config.thrust_weight         = problem.cost.thrust_weight;
    cost_config.time_weight           = problem.cost.time_weight;
    cost_config.omg_consistent_weight = problem.cost.omg_consistent_weight;
  }

}  // namespace planner::core
