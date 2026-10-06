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

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <planner_core/planner.hpp>
#include <planner_node/reference_buffer.hpp>

namespace planner::node
{

/**
 * @brief Thin ROS adapter around the pure planning core.
 *
 * Owns a planner_core::Planner and the reference buffer, loads the yaml,
 * schedules planning, and streams the buffered reference as nmpc/reference.
 * All ROS specifics (messages, topics, logger, clock) live here; planner_core
 * stays ROS-free.
 */
class PlannerNode : public rclcpp::Node
{
 public:
  explicit PlannerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("planner_node", options)
  {
    const char *root = std::getenv("AP_PNC_DIR");
    if (!root || !std::filesystem::path(root).is_absolute())
      throw std::runtime_error("AP_PNC_DIR must be an absolute project root");
    auto config_dir = std::filesystem::path(root) / "bringup/config";
    if (!std::filesystem::is_directory(config_dir))
      config_dir = std::filesystem::path(root) / "core/bringup/config";
    const std::string cfg = config_dir.string();

    core::ProblemConfig config;
    config.load(cfg + "/planning.yaml");

    // The NMPC horizon convention is owned by nmpc; the planner reads it and
    // builds the reference on exactly that grid.
    ReferenceBufferConfig buffer_cfg;
    try
    {
      const YAML::Node nmpc = YAML::LoadFile(cfg + "/nmpc.yaml")["nmpc"];
      buffer_cfg.horizon_s =
          nmpc["horizon_s"].as<double>(buffer_cfg.horizon_s);
      buffer_cfg.traj_res_s =
          nmpc["traj_res_s"].as<double>(buffer_cfg.traj_res_s);
      buffer_cfg.ctrl_frq = nmpc["ctrl_frq"].as<double>(buffer_cfg.ctrl_frq);
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Could not read nmpc.yaml (%s); using defaults", e.what());
    }

    auto sink = [this](core::LogLevel level, const std::string &msg)
    {
      switch (level)
      {
        case core::LogLevel::Debug:
          RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
          break;
        case core::LogLevel::Warn:
          RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
          break;
        case core::LogLevel::Error:
          RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
          break;
        case core::LogLevel::Info:
        default:
          RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
          break;
      }
    };

    planner_ = std::make_unique<core::Planner>(config, sink);
    buffer_  = std::make_unique<ReferenceBuffer>(*this, buffer_cfg);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        2s,
        [this]()
        {
          // One-shot policy lives here (the core stays reusable): plan once,
          // then hold the buffered reference.
          if (planned_)
          {
            return;
          }
          auto traj = planner_->plan();
          if (traj)
          {
            buffer_->push(*traj);
            planned_ = true;
          }
        });
  }

 private:
  std::unique_ptr<core::Planner>   planner_;
  std::unique_ptr<ReferenceBuffer> buffer_;
  rclcpp::TimerBase::SharedPtr     timer_;
  bool                             planned_ = false;
};

}  // namespace planner::node

RCLCPP_COMPONENTS_REGISTER_NODE(planner::node::PlannerNode)
