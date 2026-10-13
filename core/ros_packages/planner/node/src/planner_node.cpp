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
 */

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <interface/msg/state.hpp>
#include <memory>
#include <optional>
#include <planner_core/planner.hpp>
#include <planner_node/reference_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <string>

namespace planner::node
{

  /**
   * @brief Thin ROS adapter around the pure planning core.
   *
   * Owns a planner_core::Planner and the reference buffer, loads the yaml,
   * schedules planning, and streams the buffered reference as nmpc/reference.
   * All ROS specifics (messages, topics, logger, clock) live here; planner_core
   * stays ROS-free.
   *
   * `problem_formulation.corridor_source` picks between two independent
   * policies, both reusing the exact same ReferenceBuffer/NMPC interface:
   *   - box_sfc (default): plan once (the original behaviour, unchanged).
   *   - map_sfc (experimental): build a voxel map from a latched ground-truth
   *     point cloud (`/lidar_sim/map`) and continuously replan one leg at a
   *     time via core::Planner::plan_corridor, pushing each new leg onto the
   *     same buffer a real flight stack also reads from (this is the "sim and
   *     real" shared path: nothing here is specific to simulation).
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

      config_.load(cfg + "/planning.yaml");
      map_sfc_ = config_.problem.corridor_source == "map_sfc";

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
      // map_sfc needs a previous leg kept alive while the next one is being
      // replanned; box_sfc keeps capacity 1, its original behaviour.
      if (map_sfc_)
      {
        buffer_cfg.capacity = 3;
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

      planner_ = std::make_unique<core::Planner>(config_, sink);
      buffer_  = std::make_unique<ReferenceBuffer>(*this, buffer_cfg);

      using namespace std::chrono_literals;
      timer_ = this->create_wall_timer(2s,
                                       [this]()
                                       {
                                         // Original one-shot box_sfc policy,
                                         // untouched: plan once, then hold.
                                         if (map_sfc_ || planned_)
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

      if (map_sfc_)
      {
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.problem.map.map_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            { on_map(*msg); });
        state_sub_ = this->create_subscription<interface::msg::State>(
            "/px4ctrl/state", rclcpp::QoS(1).reliable(),
            [this](const interface::msg::State::SharedPtr msg)
            {
              last_position_ =
                  Eigen::Vector3d(msg->state[0], msg->state[1], msg->state[2]);
              state_ready_ = true;
            });
        replan_timer_ =
            this->create_wall_timer(100ms, [this]() { replan_if_due(); });
      }
    }

   private:
    void on_map(const sensor_msgs::msg::PointCloud2 &msg)
    {
      if (map_ || msg.point_step < 12 || msg.height * msg.width == 0)
      {
        return;
      }
      std::vector<Eigen::Vector3d> points;
      points.reserve(msg.height * msg.width);
      for (std::uint32_t i = 0; i < msg.height * msg.width; ++i)
      {
        float xyz[3];
        std::memcpy(xyz, msg.data.data() + i * msg.point_step, sizeof(xyz));
        points.emplace_back(xyz[0], xyz[1], xyz[2]);
      }
      if (points.empty())
      {
        return;
      }

      Eigen::Vector3d lo = points.front(), hi = points.front();
      for (const Eigen::Vector3d &p : points)
      {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
      }
      const double margin = config_.problem.map.bounds_margin_m;
      lo -= Eigen::Vector3d::Constant(margin);
      hi += Eigen::Vector3d::Constant(margin);
      const double          voxel = config_.problem.map.voxel_scale_m;
      const Eigen::Vector3i size =
          ((hi - lo) / voxel).array().ceil().cast<int>().matrix() +
          Eigen::Vector3i::Ones();

      map_.emplace(size, lo, voxel);
      for (const Eigen::Vector3d &p : points)
      {
        map_->setOccupied(p);
      }
      const int dilate_cells = static_cast<int>(
          std::ceil(config_.problem.map.dilate_radius_m / voxel));
      map_->dilate(std::max(1, dilate_cells));
      force_replan_ = true;
      RCLCPP_INFO(this->get_logger(), "map_sfc: ingested map (%zu points)",
                  points.size());
    }

    void replan_if_due()
    {
      if (!map_)
      {
        return;
      }
      const rclcpp::Time now = this->now();
      if (now < next_replan_attempt_)
      {
        return;
      }
      const bool   have_leg = pushed_at_.has_value();
      const double remaining =
          have_leg ? current_leg_duration_s_ - (now - *pushed_at_).seconds()
                   : -1.0;
      const bool due = force_replan_ || !have_leg ||
                       remaining < config_.problem.map.replan_margin_s;
      if (!due)
      {
        return;
      }
      next_replan_attempt_ = now + rclcpp::Duration::from_seconds(
                                       config_.problem.map.replan_margin_s);
      try
      {
        replan_attempt(now, have_leg);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "map_sfc: corridor replan threw (%s); holding the "
                     "current leg and retrying later",
                     e.what());
      }
    }

    void replan_attempt(const rclcpp::Time &now, bool have_leg)
    {
      Eigen::Matrix3d iniState;
      if (have_leg)
      {
        const double local_t = (now - *pushed_at_).seconds();
        const auto   ini     = last_trajectory_->sample(local_t);
        iniState.col(0)      = ini.p;
        iniState.col(1)      = ini.v;
        iniState.col(2)      = ini.a;

        if (state_ready_)
        {
          const double drift = (last_position_ - ini.p).norm();
          if (drift > config_.problem.map.drift_threshold_m &&
              !drift_warning_logged_)
          {
            RCLCPP_WARN(this->get_logger(),
                        "map_sfc: tracked position drifted %.2fm from the "
                        "predicted splice state (threshold %.2fm)",
                        drift, config_.problem.map.drift_threshold_m);
            drift_warning_logged_ = true;
          }
        }
      }
      else
      {
        // Bootstrap: no leg pushed yet, start from rest at the mission route's
        // own first waypoint (mirrors box_sfc's own iniState for plan()).
        const auto route0 = planner_->mission_route().front();
        iniState.col(0)   = route0.col(0);
        iniState.col(1).setZero();
        iniState.col(2).setZero();
      }

      const auto route = planner_->mission_route();
      if (route.size() < 2)
      {
        return;
      }
      const std::size_t piece_n = route.size() - 1;

      if (force_replan_ || !have_leg)
      {
        std::size_t best      = 0;
        double      best_dist = (route.front().col(0) - iniState.col(0)).norm();
        for (std::size_t i = 1; i < route.size(); ++i)
        {
          const double d = (route[i].col(0) - iniState.col(0)).norm();
          if (d < best_dist)
          {
            best_dist = d;
            best      = i;
          }
        }
        mission_waypoint_index_ = best % piece_n;
      }
      const std::size_t target_index = (mission_waypoint_index_ + 1) % piece_n;
      const Eigen::Matrix3d finState = route[target_index].leftCols<3>();

      auto planned = planner_->plan_corridor(iniState, finState, *map_);
      if (!planned || planned->empty())
      {
        if (!map_degenerate_logged_)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "map_sfc: corridor replan failed; holding the current "
                       "leg and retrying later");
          map_degenerate_logged_ = true;
        }
        return;
      }

      last_trajectory_ = std::make_shared<const core::ReferenceTrajectory>(
          std::move(*planned));
      buffer_->push(*last_trajectory_);
      pushed_at_              = now;
      current_leg_duration_s_ = last_trajectory_->duration();
      mission_waypoint_index_ = target_index;
      force_replan_           = false;
    }

    std::unique_ptr<core::Planner>   planner_;
    std::unique_ptr<ReferenceBuffer> buffer_;
    core::ProblemConfig              config_;
    rclcpp::TimerBase::SharedPtr     timer_;
    bool                             planned_ = false;

    // --- map_sfc only ---
    bool map_sfc_ = false;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<interface::msg::State>::SharedPtr         state_sub_;
    rclcpp::TimerBase::SharedPtr       replan_timer_;
    std::optional<voxel_map::VoxelMap> map_;
    bool                               force_replan_          = false;
    bool                               map_degenerate_logged_ = false;
    bool                               drift_warning_logged_  = false;
    bool                               state_ready_           = false;
    Eigen::Vector3d                    last_position_ = Eigen::Vector3d::Zero();
    std::shared_ptr<const core::ReferenceTrajectory> last_trajectory_;
    std::optional<rclcpp::Time>                      pushed_at_;
    double       current_leg_duration_s_ = 0.0;
    std::size_t  mission_waypoint_index_ = 0;
    rclcpp::Time next_replan_attempt_{0, 0, RCL_ROS_TIME};
  };

}  // namespace planner::node

RCLCPP_COMPONENTS_REGISTER_NODE(planner::node::PlannerNode)
