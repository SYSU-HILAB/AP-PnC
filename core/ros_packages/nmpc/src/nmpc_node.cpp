/* BSD-3-Clause. Copyright (c) 2026, Sun Yat-sen University.
 * Author: Erchao Rong <rongerch@outlook.com>, Zihao Liu <liuzh297@gmail.com>,
 * Junning Liang <gordonliang27@foxmail.com>
 */
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <interface/msg/control.hpp>
#include <interface/msg/reference_horizon.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/tracking_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <stdexcept>
#include <vector>

#include "nmpc_controller/tracking_controller.hpp"

class NMPC : public rclcpp::Node
{
 public:
  explicit NMPC(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("NMPC", options)
  {
    const char *root = std::getenv("AP_PNC_DIR");
    if (!root || !std::filesystem::path(root).is_absolute())
      throw std::runtime_error("AP_PNC_DIR must be an absolute project root");
    auto path = std::filesystem::path(root) / "bringup/config/nmpc.yaml";
    if (!std::filesystem::exists(path))
      path = std::filesystem::path(root) / "core/bringup/config/nmpc.yaml";
    const auto           n = YAML::LoadFile(path.string())["nmpc"];
    nmpc::TrackingConfig cfg;
    cfg.horizon_s                 = n["horizon_s"].as<double>();
    cfg.traj_res_s                = n["traj_res_s"].as<double>();
    cfg.ctrl_frq                  = n["ctrl_frq"].as<double>();
    cfg.initial_specific_thrust   = n["initial_specific_thrust"].as<double>();
    cfg.cx_alpha_slope            = n["cx_alpha_slope"].as<double>();
    cfg.cx_slope_estimation       = n["cx_slope_estimation"].as<bool>();
    cfg.cx_slope_forgetting       = n["cx_slope_forgetting"].as<double>();
    cfg.cx_slope_limit            = n["cx_slope_limit"].as<double>();
    cfg.cx_slope_covariance_limit = n["cx_slope_covariance_limit"].as<double>();
    controller_    = std::make_unique<nmpc::TrackingController>(cfg);
    const auto qos = rclcpp::QoS(1).reliable();
    control_pub_ =
        create_publisher<interface::msg::Control>("/nmpc/control", qos);
    ready_pub_ =
        create_publisher<std_msgs::msg::Bool>("/nmpc/reference_request", qos);
    tracking_pub_ = create_publisher<interface::msg::TrackingInfo>(
        "/nmpc/tracking_info", qos);
    state_sub_ = create_subscription<interface::msg::State>(
        "/px4ctrl/state", qos,
        [this](const interface::msg::State &msg)
        {
          state_       = msg.state;
          state_ready_ = true;
          std_msgs::msg::Bool ready;
          ready.data = true;
          ready_pub_->publish(ready);
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu &msg)
        { specific_force_x_ = msg.linear_acceleration.x; });
    reference_sub_ = create_subscription<interface::msg::ReferenceHorizon>(
        "/nmpc/reference", qos,
        [this](const interface::msg::ReferenceHorizon &msg) { solve(msg); });
  }

 private:
  std::unique_ptr<nmpc::TrackingController> controller_;
  std::array<double, 13>                    state_{};
  double                                    specific_force_x_ = 0.0;
  bool                                      state_ready_      = false;
  rclcpp::Subscription<interface::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<interface::msg::ReferenceHorizon>::SharedPtr
                                                             reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     imu_sub_;
  rclcpp::Publisher<interface::msg::Control>::SharedPtr      control_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          ready_pub_;
  rclcpp::Publisher<interface::msg::TrackingInfo>::SharedPtr tracking_pub_;
  void solve(const interface::msg::ReferenceHorizon &reference)
  {
    if (!state_ready_ || reference.points.empty())
      return;
    try
    {
      if (std::abs(reference.traj_resolution -
                   controller_->config().traj_res_s) > 1e-9)
        throw std::runtime_error(
            "reference horizon grid differs from nmpc.yaml");
      std::vector<nmpc::CostReference> horizon;
      horizon.reserve(reference.points.size());
      for (const auto &p : reference.points)
        horizon.push_back(nmpc::cost_reference({p.pos[0], p.pos[1], p.pos[2]},
                                               {p.vel[0], p.vel[1], p.vel[2]},
                                               {p.yb[0], p.yb[1], p.yb[2]}));
      const auto start = std::chrono::steady_clock::now();
      const auto result =
          controller_->compute(state_, specific_force_x_, horizon);
      interface::msg::Control command;
      command.timestamp =
          static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
      command.rates_sp          = result.rates;
      command.specific_force_sp = result.specific_force;
      command.sol_time          = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count();
      control_pub_->publish(command);
      if (!reference.tracking_valid)
        return;
      interface::msg::TrackingInfo info;
      info.timestamp               = command.timestamp;
      const auto              &ref = reference.points.front();
      const Eigen::Quaterniond q(state_[9], state_[10], state_[11], state_[12]);
      const Eigen::Vector3d    v(state_[3], state_[4], state_[5]);
      const Eigen::Vector3d    vb = q.conjugate() * v;
      const Eigen::Vector3d    yb = q.toRotationMatrix().col(1);
      for (int i = 0; i < 3; ++i)
      {
        info.ref_position[i]    = static_cast<float>(ref.pos[i]);
        info.ref_velocity[i]    = static_cast<float>(ref.vel[i]);
        info.ref_yb[i]          = static_cast<float>(ref.yb[i]);
        info.actual_position[i] = static_cast<float>(state_[i]);
        info.actual_velocity[i] = static_cast<float>(state_[3 + i]);
        info.actual_yb[i]       = static_cast<float>(yb[i]);
      }
      info.actual_vxy = static_cast<float>(std::hypot(v.x(), v.y()));
      info.ref_vxy    = static_cast<float>(std::hypot(ref.vel[0], ref.vel[1]));
      info.alpha      = static_cast<float>(std::atan2(vb.x(), vb.z()));
      info.beta =
          static_cast<float>(std::atan2(vb.y(), std::hypot(vb.x(), vb.z())));
      info.cz = static_cast<float>(controller_->cz());
      tracking_pub_->publish(info);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "NMPC cycle skipped: %s", e.what());
    }
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NMPC)
