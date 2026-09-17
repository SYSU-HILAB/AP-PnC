/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * nmpc_bridge: mavros replacement of the uXRCE-DDS px4_interface.
 *  - State adapter: mavros/local_position/odom (ENU, FLU) -> /cur_state.
 *  - RLS throttle model + INDI incremental thrust law (reference:
 *    PX4-Neupilot src/modules/acc_rates_control/AccRatesControl.cpp).
 *  - Emits AttitudeTarget body rates (FLU) + normalized thrust on an
 *    internal topic consumed by flight_mux (single mavros publisher).
 */

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>

#include "interface/msg/controls.hpp"
#include "interface/msg/state.hpp"
#include "interface/msg/throttle_model_status.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rls_throttle_model.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
  /**
   * Load a nested yaml scalar with a fallback default.
   *
   * @param[in] node Root yaml node [-]
   * @param[in] keys Dotted section path, e.g. "rls.lambda" [-]
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
 * Bridge node converting NMPC controls into mavros rate setpoints with an
 * online RLS throttle model.
 */
class NmpcBridge : public rclcpp::Node
{
 public:
  NmpcBridge() : Node("nmpc_bridge")
  {
    loadConfig("bringup/config/mavros_bridge.yaml");

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto latch_qos  = rclcpp::QoS(1).transient_local();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&NmpcBridge::odomCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos, std::bind(&NmpcBridge::imuCallback, this, _1));
    controls_sub_ = create_subscription<interface::msg::Controls>(
        controls_topic_, sensor_qos,
        std::bind(&NmpcBridge::controlsCallback, this, _1));

    state_pub_ =
        create_publisher<interface::msg::State>(state_topic_, latch_qos);
    state_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(state_odom_topic_, latch_qos);
    odom_world_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        odom_world_topic_, sensor_qos);
    setpoint_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
        bridge_setpoint_topic_, sensor_qos);
    status_pub_ = create_publisher<interface::msg::ThrottleModelStatus>(
        status_topic_, sensor_qos);

    state_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int>(1.0e6 / state_rate_hz_)),
        std::bind(&NmpcBridge::publishState, this));

    rls_.reset(eta_init_);
    RCLCPP_INFO(
        get_logger(),
        "nmpc_bridge up: eta0=%.2f m/s^2, lambda=%.4f, bounds=[%.1f, %.1f]",
        rls_.getEta(), rls_lambda_, eta_min_, eta_max_);
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
    const auto br = cfg["nmpc_bridge"];

    rls_lambda_               = yamlGet(br, "rls.lambda", rls_lambda_);
    eta_min_                  = yamlGet(br, "rls.eta_min", eta_min_);
    eta_max_                  = yamlGet(br, "rls.eta_max", eta_max_);
    const double eta_init_raw = yamlGet(br, "rls.eta_init", 0.0);
    const double hover_thr    = yamlGet(br, "rls.hover_throttle", 0.5);
    eta_init_ =
        eta_init_raw > 0.0 ? eta_init_raw : RlsThrottleModel::kOneG / hover_thr;
    rls_u_min_       = yamlGet(br, "rls.u_min", rls_u_min_);
    rls_p_init_      = yamlGet(br, "rls.p_init", rls_p_init_);
    rls_var_thr_     = yamlGet(br, "rls.var_valid_thr", rls_var_thr_);
    innovation_gate_ = yamlGet(br, "rls.innovation_gate", innovation_gate_);

    kp_    = yamlGet(br, "indi.specific_force_p", kp_);
    u_min_ = yamlGet(br, "indi.throttle_min", u_min_);
    u_max_ = yamlGet(br, "indi.throttle_max", u_max_);
    throttle_lpf_.setCutoff(yamlGet(br, "indi.throttle_lpf_cutoff_hz", 20.0));
    setpoint_lpf_.setCutoff(yamlGet(br, "indi.setpoint_lpf_cutoff_hz", 20.0));
    meas_lpf_.setCutoff(yamlGet(br, "indi.meas_lpf_cutoff_hz", 20.0));

    state_rate_hz_ = yamlGet(br, "io.state_rate_hz", state_rate_hz_);

    odom_topic_ = yamlGet<std::string>(br, "io.odom_topic",
                                       "/mavros/local_position/odom");
    imu_topic_ =
        yamlGet<std::string>(br, "io.imu_topic", "/mavros/imu/data_raw");
    controls_topic_ =
        yamlGet<std::string>(br, "io.controls_topic", "/mpc_controls");
    state_topic_ = yamlGet<std::string>(br, "io.state_topic", "/cur_state");
    state_odom_topic_ =
        yamlGet<std::string>(br, "io.state_odom_topic", "/cur_state_odom");
    odom_world_topic_      = yamlGet<std::string>(br, "io.odom_world_topic",
                                                  "/nmpc_bridge/odom_world");
    bridge_setpoint_topic_ = yamlGet<std::string>(
        br, "io.bridge_setpoint_topic", "/nmpc_bridge/setpoint_raw/attitude");
    status_topic_ = yamlGet<std::string>(br, "io.status_topic",
                                         "/nmpc_bridge/throttle_model_status");

    rls_.setForgettingFactor(rls_lambda_);
    rls_.setEtaBounds(eta_min_, eta_max_);
    rls_.setMinThrustForUpdate(rls_u_min_);
    rls_.setPInit(rls_p_init_);
    rls_.setVarValidThreshold(rls_var_thr_);
    rls_.setInnovationGate(innovation_gate_);
  }

  /**
   * Record the takeoff origin on the first odometry sample, then keep the
   * latest ENU/FLU state for the fixed-rate /cur_state publication.
   *
   * @param[in] msg mavros local position odometry (ENU world, FLU body) [-]
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!origin_set_)
    {
      init_pos_ =
          Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
      origin_set_ = true;
      RCLCPP_INFO(get_logger(), "State origin locked at (%.2f, %.2f, %.2f)",
                  init_pos_.x(), init_pos_.y(), init_pos_.z());
    }

    const Eigen::Vector3d    p(msg->pose.pose.position.x - init_pos_.x(),
                               msg->pose.pose.position.y - init_pos_.y(),
                               msg->pose.pose.position.z - init_pos_.z());
    const Eigen::Quaterniond q(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // mavros local_position/odom twist is body-frame (FLU); the state
    // contract (px4ctrl / c_nmpc / ekf_quat lineage) is world-frame.
    const Eigen::Vector3d v = q * Eigen::Vector3d(msg->twist.twist.linear.x,
                                                  msg->twist.twist.linear.y,
                                                  msg->twist.twist.linear.z);
    const Eigen::Vector3d w(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z);

    // Corrected odometry for world-frame consumers (px4ctrl).
    nav_msgs::msg::Odometry odom_world = *msg;
    odom_world.header.frame_id         = "world";
    odom_world.child_frame_id.clear();
    odom_world.twist.twist.linear.x = v.x();
    odom_world.twist.twist.linear.y = v.y();
    odom_world.twist.twist.linear.z = v.z();
    odom_world_pub_->publish(odom_world);

    using State = interface::msg::State;
    std::copy(p.data(), p.data() + 3,
              state_.state.begin() + State::POS_START_INDEX);
    std::copy(v.data(), v.data() + 3,
              state_.state.begin() + State::VEL_START_INDEX);
    std::copy(w.data(), w.data() + 3,
              state_.state.begin() + State::RATE_START_INDEX);
    const Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    std::copy(q_vec.data(), q_vec.data() + 4,
              state_.state.begin() + State::QUAT_START_INDEX);
    state_.cur_cz = 0.0;
    odom_valid_   = true;
  }

  /**
   * Step the RLS throttle model with the filtered commanded throttle and
   * the filtered thrust-axis specific force (ROS-FLU, hover ~ +1 g).
   *
   * @param[in] msg mavros HIGHRES_IMU raw specific force (FLU) [-]
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double dt =
        last_imu_stamp_.nanoseconds() > 0
            ? (rclcpp::Time(msg->header.stamp) - last_imu_stamp_).seconds()
            : 0.005;
    last_imu_stamp_ = rclcpp::Time(msg->header.stamp);

    const double a_meas = msg->linear_acceleration.z;
    const double a_lpf  = meas_lpf_.apply(a_meas, dt);
    const double u_lpf  = throttle_lpf_.apply(throttle_memory_, dt);

    rls_.step(u_lpf, a_lpf);
  }

  /**
   * Convert one NMPC control packet into an AttitudeTarget (FLU rates +
   * INDI-computed normalized thrust) and publish it plus the estimator
   * status.
   *
   * @param[in] msg NMPC output: rates_sp (FLU) + thrust_acc_sp [m/s^2]
   */
  void controlsCallback(const interface::msg::Controls::SharedPtr msg)
  {
    if (!origin_set_ || !odom_valid_)
    {
      return;
    }

    const auto   now_steady = std::chrono::steady_clock::now();
    const double dt =
        last_ctrl_time_.time_since_epoch().count() > 0
            ? std::chrono::duration<double>(now_steady - last_ctrl_time_)
                  .count()
            : ctrl_dt_fallback_;
    last_ctrl_time_ = now_steady;

    const double a_sp = setpoint_lpf_.apply(msg->thrust_acc_sp, dt);

    const double u_cmd =
        computeIndiThrottle(throttle_memory_, rls_.getEta(), a_sp,
                            meas_lpf_.value(), kp_, dt, u_min_, u_max_);
    throttle_memory_ = u_cmd;

    mavros_msgs::msg::AttitudeTarget sp;
    sp.header.stamp = this->now();
    sp.type_mask    = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
    sp.body_rate.x  = msg->rates_sp[0];
    sp.body_rate.y  = msg->rates_sp[1];
    sp.body_rate.z  = msg->rates_sp[2];
    sp.thrust       = u_cmd;
    setpoint_pub_->publish(sp);

    interface::msg::ThrottleModelStatus status;
    status.timestamp               = now().nanoseconds() / 1000;
    status.timestamp_sample        = msg->timestamp;
    status.eta                     = rls_.getEta();
    status.eta_variance            = rls_.getEtaVar();
    status.innovation              = rls_.getInnovation();
    status.innovation_test_ratio   = rls_.getInnovationTestRatio();
    status.estimate_valid          = rls_.isEstimateValid();
    status.throttle_input          = throttle_lpf_.value();
    status.measured_specific_force = meas_lpf_.value();
    status_pub_->publish(status);
  }

  /**
   * Fixed-rate /cur_state publication for c_nmpc.
   */
  void publishState()
  {
    if (!origin_set_ || !odom_valid_)
    {
      return;
    }
    state_.timestamp = now().nanoseconds() / 1000;
    state_pub_->publish(state_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp            = now();
    odom.header.frame_id         = "world";
    using State                  = interface::msg::State;
    odom.pose.pose.position.x    = state_.state[State::POS_START_INDEX];
    odom.pose.pose.position.y    = state_.state[State::POS_START_INDEX + 1];
    odom.pose.pose.position.z    = state_.state[State::POS_START_INDEX + 2];
    odom.pose.pose.orientation.w = state_.state[State::QUAT_START_INDEX];
    odom.pose.pose.orientation.x = state_.state[State::QUAT_START_INDEX + 1];
    odom.pose.pose.orientation.y = state_.state[State::QUAT_START_INDEX + 2];
    odom.pose.pose.orientation.z = state_.state[State::QUAT_START_INDEX + 3];
    state_odom_pub_->publish(odom);
  }

  // RLS model parameters
  double rls_lambda_      = 0.999;
  double eta_min_         = 9.81;
  double eta_max_         = 40.0;
  double eta_init_        = 19.6;
  double rls_u_min_       = 0.10;
  double rls_p_init_      = 10.0;
  double rls_var_thr_     = 0.01;
  double innovation_gate_ = 5.0;

  // INDI law parameters
  double kp_               = 2.0;
  double u_min_            = 0.05;
  double u_max_            = 1.0;
  double ctrl_dt_fallback_ = 0.02;

  double state_rate_hz_   = 50.0;
  double throttle_memory_ = 0.5;

  std::string odom_topic_, imu_topic_, controls_topic_, state_topic_;
  std::string state_odom_topic_, odom_world_topic_, bridge_setpoint_topic_;
  std::string status_topic_;

  RlsThrottleModel rls_;
  FirstOrderLpf    throttle_lpf_, setpoint_lpf_, meas_lpf_;

  Eigen::Vector3d       init_pos_   = Eigen::Vector3d::Zero();
  bool                  origin_set_ = false;
  bool                  odom_valid_ = false;
  interface::msg::State state_;

  rclcpp::Time                          last_imu_stamp_ = rclcpp::Time(0);
  std::chrono::steady_clock::time_point last_ctrl_time_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
  rclcpp::Subscription<interface::msg::Controls>::SharedPtr controls_sub_;

  rclcpp::Publisher<interface::msg::State>::SharedPtr   state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_world_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<interface::msg::ThrottleModelStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NmpcBridge>());
  rclcpp::shutdown();
  return 0;
}
