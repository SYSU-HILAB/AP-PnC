/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


 // #include 
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_acc_setpoint.hpp>
#include <interface/msg/controls.hpp>
#include <interface/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>
#include <std_msgs/msg/bool.hpp>
// #include <interface/msg/
// #include 
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class Transcribe {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d r;
  Eigen::Quaterniond q;
  Eigen::Quaterniond q_flu_frd = Eigen::Quaterniond(0, 1, 0, 1);

  public:

  Transcribe() {
    q_flu_frd.normalize();
  }
  template <typename T>
  void to_flu(const std::array<T, 3>& pos, const std::array<T, 3>& vel, const std::array<T, 3>& rate, const std::array<T,4>& quat, interface::msg::State& state) {
    p = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    v = Eigen::Vector3d(vel[0], vel[1], vel[2]);
    r = Eigen::Vector3d(rate[0], rate[1], rate[2]);
    q = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
    p = q_flu_frd * p;
    v = q_flu_frd * v;
    r = q_flu_frd * r;
    q = q_flu_frd * q * q_flu_frd;
    q.normalize();
    if (q.coeffs()[0] < 0) {
      q.coeffs() = -q.coeffs();
    }
    using interface::msg::State;
    std::copy(p.data(), p.data() + 3, state.state.begin() + State::POS_START_INDEX);
    std::copy(v.data(), v.data() + 3, state.state.begin() + State::VEL_START_INDEX);
    std::copy(r.data(), r.data() + 3, state.state.begin() + State::RATE_START_INDEX);
    std::copy(q.coeffs().data(), q.coeffs().data() + 4, state.state.begin() + State::QUAT_START_INDEX);
  }
};


class PX4Interface : public rclcpp::Node, public Transcribe
{
public:
  PX4Interface() : Node("px4_interface")
  {

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    // Init sub VehicleOdometry Controls; pub vehicle_thrust_setpoint State
    auto keep_last_qos = rclcpp::SensorDataQoS().keep_last(1);
    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", keep_last_qos, std::bind(&PX4Interface::vehicle_odometry_callback, this, std::placeholders::_1));

    auto heartbeat_callback = [this]() -> void {
      publish_offboard_control_mode();
      };
    // set paramter to decide the control frequency, i.e 1/control_interval.
    // the unit of control_interval is second 
    this->declare_parameter("control_interval", 0.01);
    control_interval_ = this->get_parameter("control_interval").as_double();
    // std::chrono::duration<double>(control_interval_)
    heartbeat_timer_ = this->create_wall_timer(100ms, heartbeat_callback);
    // trigger_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("trigger", keep_last_qos, trigger_cb);
  }



private:
  double control_interval_;
  bool is_init_ = false;
  interface::msg::Controls controls_;
  interface::msg::State state_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  // sub VehicleOdometry Controls; pub vehicle_thrust_setpoint State
  rclcpp::Publisher<VehicleThrustAccSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;
  rclcpp::Publisher<interface::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

  rclcpp::Subscription<interface::msg::Controls>::SharedPtr controls_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

  // sub std bool 
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscriber_;
  // subscription callbacks above
  void controls_callback(const interface::msg::Controls::SharedPtr msg);
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void trigger_cb();
  void publish_offboard_control_mode();
  void publish_state();

  // px4_control is set to vehicle_thrust_acc_setpoint
  void publish_px4_control();
};



/**
 * @brief Callback on vehicle_odometry
 */
void PX4Interface::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  std::cout << "Odometry received: " << msg->timestamp << std::endl;
  if (!is_init_) {
    is_init_ = true;
    // state_pub_timer_ = this->create_wall_timer(100ms, std::bind(&PX4Interface::publish_state, this));
    // use control_interval_ to set pub_timer_
    state_pub_timer_ = this->create_wall_timer(std::chrono::duration<double>(control_interval_), std::bind(&PX4Interface::publish_state, this));
  }
  this->to_flu(msg->position, msg->velocity, msg->angular_velocity , msg->q, state_);
}

void PX4Interface::publish_state()
{
  state_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  state_publisher_->publish(state_);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void PX4Interface::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}




int main(int argc, char* argv[])
{
  std::cout << "Starting PX4 Interface Node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Interface>());

  rclcpp::shutdown();
  return 0;
}