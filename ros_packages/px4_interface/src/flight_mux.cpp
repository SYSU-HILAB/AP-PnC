/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * flight_mux: the single publisher towards /mavros/setpoint_raw/attitude.
 * Forwards the px4ctrl stream (hover/FSM phases, offboard entry, arming)
 * and switches to the nmpc_bridge stream on the /mode trigger, falling
 * back to px4ctrl whenever the NMPC stream goes stale. px4ctrl keeps
 * ownership of the mavros arming/offboard services.
 */

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>

#include "mavros_msgs/msg/attitude_target.hpp"
#include "mux_selector.h"

using std::placeholders::_1;

namespace
{
  /**
   * Load a nested yaml scalar with a fallback default.
   *
   * @param[in] node Root yaml node [-]
   * @param[in] keys Dotted section path, e.g. "handover.stream_timeout" [-]
   * @param[in] fallback Value returned when the path is missing [-]
   * @return Configured scalar value [-]
   */
  template <typename T>
  T yamlGet(const YAML::Node &node, const std::string &keys, T fallback)
  {
    try
    {
      YAML::Node        cur = node;
      std::stringstream ss(keys);
      std::string       key;
      while (std::getline(ss, key, '.'))
      {
        cur = cur[key];
      }
      return cur.as<T>();
    }
    catch (...)
    {
      return fallback;
    }
  }
}  // namespace

/**
 * Setpoint stream multiplexer between px4ctrl and the NMPC bridge.
 */
class FlightMux : public rclcpp::Node
{
 public:
  FlightMux() : Node("flight_mux")
  {
    loadConfig("bringup/config/mavros_bridge.yaml");

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    px4ctrl_sub_ = create_subscription<mavros_msgs::msg::AttitudeTarget>(
        px4ctrl_topic_, sensor_qos,
        std::bind(&FlightMux::px4ctrlCallback, this, _1));
    nmpc_sub_ = create_subscription<mavros_msgs::msg::AttitudeTarget>(
        bridge_topic_, sensor_qos,
        std::bind(&FlightMux::nmpcCallback, this, _1));
    mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        mode_topic_, rclcpp::QoS(10),
        std::bind(&FlightMux::modeCallback, this, _1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        reset_topic_, rclcpp::QoS(10),
        std::bind(&FlightMux::resetCallback, this, _1));

    output_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
        output_topic_, sensor_qos);

    // Watchdog only re-evaluates the selection (e.g. NMPC staleness
    // fallback); it never republishes, so a dead upstream cannot be masked
    // by a frozen setpoint stream.
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this]() { runSelector(false, false); });

    RCLCPP_INFO(get_logger(),
                "flight_mux up: px4ctrl<-%s, nmpc<-%s -> %s (timeout %.2f s)",
                px4ctrl_topic_.c_str(), bridge_topic_.c_str(),
                output_topic_.c_str(), stream_timeout_);
  }

 private:
  /**
   * Read sectioned yaml configuration (cwd-relative, repo convention).
   *
   * @param[in] path Config file path relative to the working directory [-]
   */
  void loadConfig(const std::string &path)
  {
    YAML::Node cfg;
    try
    {
      cfg = YAML::LoadFile(path);
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_FATAL(get_logger(), "Cannot load %s: %s", path.c_str(), e.what());
      throw std::runtime_error("config load failed");
    }
    const auto mx = cfg["flight_mux"];

    stream_timeout_ = yamlGet(mx, "handover.stream_timeout", 0.2);
    selector_.setStreamTimeout(stream_timeout_);

    px4ctrl_topic_ = yamlGet<std::string>(mx, "io.px4ctrl_topic",
                                          "/px4ctrl/setpoint_raw/attitude");
    bridge_topic_  = yamlGet<std::string>(mx, "io.bridge_topic",
                                          "/nmpc_bridge/setpoint_raw/attitude");
    output_topic_  = yamlGet<std::string>(mx, "io.output_topic",
                                          "/mavros/setpoint_raw/attitude");
    mode_topic_    = yamlGet<std::string>(mx, "io.mode_topic", "/mode");
    reset_topic_   = yamlGet<std::string>(mx, "io.reset_topic", "/mux_reset");
  }

  /**
   * Track px4ctrl stream (forwarded when selected).
   *
   * @param[in] msg px4ctrl body-rate + thrust setpoint [-]
   */
  void px4ctrlCallback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
  {
    last_px4ctrl_msg_ = *msg;
    px4ctrl_fresh_    = true;
    runSelector(false, true);
  }

  /**
   * Track the NMPC bridge stream (forwarded when selected).
   *
   * @param[in] msg NMPC body-rate + thrust setpoint [-]
   */
  void nmpcCallback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
  {
    last_nmpc_msg_  = *msg;
    last_nmpc_time_ = now();
    runSelector(false, true);
  }

  /**
   * /mode true latches NMPC selection intent (stack trigger go).
   *
   * @param[in] msg Boolean trigger [-]
   */
  void modeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "NMPC handover requested (/mode)");
      runSelector(true, false);
    }
  }

  /**
   * /mux_reset true drops the NMPC latch back to px4ctrl.
   *
   * @param[in] msg Boolean reset [-]
   */
  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      selector_.reset(now().seconds());
      RCLCPP_INFO(get_logger(), "Mux reset to px4ctrl stream");
    }
  }

  /**
   * Advance the selection state and forward the winning stream.
   *
   * @param[in] mode_trigger True when invoked from the /mode trigger [-]
   * @param[in] publish Forward the winning stream after the update [-]
   */
  void runSelector(bool mode_trigger, bool publish)
  {
    const double nmpc_age = last_nmpc_time_.nanoseconds() == 0
                                ? 1.0e9
                                : (now() - last_nmpc_time_).seconds();
    const auto   prev     = selector_.source();
    const auto src = selector_.update(mode_trigger, nmpc_age, now().seconds());

    if (src != prev)
    {
      RCLCPP_INFO(get_logger(), "Mux switch: %s",
                  src == MuxSelector::Source::kNmpc ? "NMPC" : "PX4CTRL");
    }

    if (!publish)
    {
      return;
    }
    if (src == MuxSelector::Source::kNmpc)
    {
      output_pub_->publish(last_nmpc_msg_);
    }
    else if (px4ctrl_fresh_)
    {
      output_pub_->publish(last_px4ctrl_msg_);
    }
  }

  double      stream_timeout_ = 0.2;
  std::string px4ctrl_topic_, bridge_topic_, output_topic_, mode_topic_,
      reset_topic_;

  MuxSelector                      selector_;
  mavros_msgs::msg::AttitudeTarget last_px4ctrl_msg_, last_nmpc_msg_;
  bool                             px4ctrl_fresh_  = false;
  rclcpp::Time                     last_nmpc_time_ = rclcpp::Time(0);

  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr
      px4ctrl_sub_;
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr nmpc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              reset_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr    output_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightMux>());
  rclcpp::shutdown();
  return 0;
}
