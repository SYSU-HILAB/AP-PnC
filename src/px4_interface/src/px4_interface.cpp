// #include
#include <stdint.h>

#include <Eigen/Geometry>
#include <chrono>
#include <interface/msg/controls.hpp>
#include <interface/msg/state.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_acc_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Transcribe {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d r;
  Eigen::Quaterniond q;

  Eigen::Quaterniond q_flu_frd = Eigen::Quaterniond(0, 1, 0, 0);
  Eigen::Quaterniond q_enu_ned = Eigen::Quaterniond(0, 1, 1, 0);
  Eigen::Quaterniond q_transform;

public:
  enum class E_World_Frame { NED, FRD };
  explicit Transcribe(const E_World_Frame world_frame) {
    q_flu_frd.normalize();
    if (world_frame == E_World_Frame::NED) {
      q_transform = Eigen::Quaterniond(0, 1, 1, 0);
    } else {
      q_transform = q_flu_frd;
    }
    q_transform.normalize();
  };
  template <typename T>
  void toFLU(const std::array<T, 3>& pos, const std::array<T, 3>& vel,
             const std::array<T, 3>& rate, const std::array<T, 4>& quat,
             interface::msg::State& state) {
    p = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    v = Eigen::Vector3d(vel[0], vel[1], vel[2]);
    r = Eigen::Vector3d(rate[0], rate[1], rate[2]);
    q = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
    p = q_transform * p;
    v = q_transform * v;
    r = q_flu_frd * r;
    q = q_transform * q * q_flu_frd;
    q.normalize();
    if (q.w() < 0) {
      q.coeffs() = -q.coeffs();
    }
    using interface::msg::State;
    std::copy(p.data(), p.data() + 3,
              state.state.begin() + State::POS_START_INDEX);
    std::copy(v.data(), v.data() + 3,
              state.state.begin() + State::VEL_START_INDEX);
    std::copy(r.data(), r.data() + 3,
              state.state.begin() + State::RATE_START_INDEX);
    // q.coeffs
    // returns
    // x,y,z,w
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    std::copy(q_vec.data(), q_vec.data() + 4,
              state.state.begin() + State::QUAT_START_INDEX);
  }

  template <typename T>
  std::array<float, 3>& toFRD(const std::array<T, 3>& rate) {
    static std::array<float, 3> rate_frd;

    r = Eigen::Vector3d(rate[0], rate[1], rate[2]);

    r = q_flu_frd * r;

    std::copy(r.data(), r.data() + 3, rate_frd.data());
    // q.coeffs
    // returns
    // x,y,z,w
    return rate_frd;
  }
};

class PX4Interface : public rclcpp::Node {
public:
  PX4Interface() : Node("px4_interface") {
    this->declare_parameter(
        "world_"
        "frame",
        "NED");
    std::string world_frame = this->get_parameter("world_frame").as_string();
    if (world_frame == "NED") {
      trans_ = std::make_unique<Transcribe>(Transcribe::E_World_Frame::NED);
    } else {
      trans_ = std::make_unique<Transcribe>(Transcribe::E_World_Frame::FRD);
    }

    auto reliable_qos = rclcpp::ParametersQoS().keep_last(1).transient_local();
    auto keep_last_qos = rclcpp::SensorDataQoS().keep_last(1);

    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

    state_publisher_ = this->create_publisher<interface::msg::State>(
        "/cur_state", reliable_qos);

    state_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/cur_state_odom", reliable_qos);
    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", keep_last_qos,
        std::bind(&PX4Interface::vehicle_odometry_callback, this,
                  std::placeholders::_1));
    vehicle_thrust_acc_setpoint_publisher_ =
        this->create_publisher<VehicleThrustAccSetpoint>(
            "/fmu/in/vehicle_thrust_acc_setpoint", 10);
    controls_subscriber_ = this->create_subscription<interface::msg::Controls>(
        "/mpc_controls", rclcpp::SensorDataQoS(),
        std::bind(&PX4Interface::controls_callback, this,
                  std::placeholders::_1));
    auto heartbeat_callback = [this]() -> void {
      publish_offboard_control_mode();
    };

    this->declare_parameter("control_interval", 0.02);
    control_interval_ = this->get_parameter("control_interval").as_double();
    heartbeat_timer_ = this->create_wall_timer(100ms, heartbeat_callback);
  }

public:
  std::unique_ptr<Transcribe> trans_;
  double control_interval_;
  bool is_init_ = false;
  interface::msg::Controls controls_;
  interface::msg::State state_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  // sub
  // VehicleOdometry
  // Controls;
  // pub
  // vehicle_thrust_setpoint
  // State
  rclcpp::Publisher<VehicleThrustAccSetpoint>::SharedPtr
      vehicle_thrust_acc_setpoint_publisher_;
  rclcpp::Publisher<interface::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr
      offboard_control_mode_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_publisher_;

  rclcpp::Subscription<interface::msg::Controls>::SharedPtr
      controls_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      vehicle_odometry_subscriber_;

  // sub
  // std
  // bool
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscriber_;
  // subscription
  // callbacks
  // above
  void controls_callback(const interface::msg::Controls::SharedPtr msg);
  void vehicle_odometry_callback(
      const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void trigger_cb();
  void publish_offboard_control_mode();
  void publish_state();
};

void PX4Interface::controls_callback(
    const interface::msg::Controls::SharedPtr msg) {
  static px4_msgs::msg::VehicleThrustAccSetpoint px4_control;
  std::cout << "Controls received: " << msg->timestamp << std::endl;
  px4_control.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  px4_control.rates_sp = trans_->toFRD(msg->rates_sp);
  px4_control.thrust_acc_sp = msg->thrust_acc_sp;
  vehicle_thrust_acc_setpoint_publisher_->publish(px4_control);
  // publish_px4_control();
}

/**
 * @brief
 * Callback
 * on
 * vehicle_odometry
 */
void PX4Interface::vehicle_odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  std::cout << "Odometry "
               "received:"
               " "
            << msg->timestamp << std::endl;
  if (!is_init_) {
    is_init_ = true;
    state_pub_timer_ = this->create_wall_timer(
        20ms, std::bind(&PX4Interface::publish_state, this));
  }
  this->trans_->toFLU(msg->position, msg->velocity, msg->angular_velocity,
                      msg->q, state_);
  std::copy(state_.state.begin(), state_.state.end(), state_.mpc_state.begin());
}

void PX4Interface::publish_state() {
  std::cout << "State "
               "publish"
            << state_.timestamp << std::endl;

  state_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  state_publisher_->publish(state_);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "world";
  using interface::msg::State;
  odom.pose.pose.position.x = state_.state[State::POS_START_INDEX];
  odom.pose.pose.position.y = state_.state[State::POS_START_INDEX + 1];
  odom.pose.pose.position.z = state_.state[State::POS_START_INDEX + 2];
  odom.twist.twist.linear.x = state_.state[State::VEL_START_INDEX];
  odom.twist.twist.linear.y = state_.state[State::VEL_START_INDEX + 1];
  odom.twist.twist.linear.z = state_.state[State::VEL_START_INDEX + 2];
  odom.twist.twist.angular.x = state_.state[State::RATE_START_INDEX];
  odom.twist.twist.angular.y = state_.state[State::RATE_START_INDEX + 1];
  odom.twist.twist.angular.z = state_.state[State::RATE_START_INDEX + 2];
  odom.pose.pose.orientation.w = state_.state[State::QUAT_START_INDEX];
  odom.pose.pose.orientation.x = state_.state[State::QUAT_START_INDEX + 1];
  odom.pose.pose.orientation.y = state_.state[State::QUAT_START_INDEX + 2];
  odom.pose.pose.orientation.z = state_.state[State::QUAT_START_INDEX + 3];
  state_odom_publisher_->publish(odom);
}

void PX4Interface::publish_offboard_control_mode() {
  OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
  std::cout << "Starting PX4 Interface Node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Interface>());

  rclcpp::shutdown();
  return 0;
}