#include <Eigen/Geometry>
#include <chrono>
#include <tailsitter_simulator/Tailsitter.hpp>

#include "interface/msg/controls.hpp"
#include "interface/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace simulator_interface {
using namespace std::chrono_literals;

using MpcStateMsg = interface::msg::State;
using OdometryMsg = nav_msgs::msg::Odometry;
using ControlMsg = interface::msg::Controls;

using SimState = TailsitterSimulator::Tailsitter::State;

TailsitterSimulator::Tailsitter tailsitter;

Eigen::Vector4d &rotToQuat(const Eigen::Matrix3d &R) {
  Eigen::Quaterniond q(R);  // 从旋转矩阵构造四元数
  q.normalize();            // 确保四元数是单位四元数

  // 如果 w 小于 0，取反四元数
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }

  static Eigen::Vector4d quat;
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();

  return quat;
}

class SimulatorInterface : public rclcpp::Node {
 private:
  std::mutex control_mutex_;

  double sim_time_step_;
  TailsitterSimulator::Tailsitter tailsitter_simulator_;
  TailsitterSimulator::Control control_;

  rclcpp::Subscription<ControlMsg>::SharedPtr control_subscriber_;
  rclcpp::Publisher<MpcStateMsg>::SharedPtr mpc_state_publisher_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

 private:
  void declareParams() {
    this->declare_parameter("sim_time_step", 0.001);
    this->declare_parameter("init_p", std::vector<double>{0, 0, 10});
    this->declare_parameter("init_v", std::vector<double>{12, 0, 0});
    this->declare_parameter("init_q", std::vector<double>{1, 0, 0, 0});
  }
  void updateParams() {
    sim_time_step_ = this->get_parameter("sim_time_step").as_double();
  }

 public:
  SimulatorInterface() : Node("simulator_interface") {
    // parameters declaration
    declareParams();
    // subscriber and publisher
    control_subscriber_ = this->create_subscription<ControlMsg>(
        "/mpc_controls", 10,
        std::bind(&SimulatorInterface::controlCallback, this,
                  std::placeholders::_1));
    mpc_state_publisher_ = this->create_publisher<MpcStateMsg>("cur_state", 10);
    odometry_publisher_ =
        this->create_publisher<OdometryMsg>("cur_state_odom", 10);

    sim_time_step_ = this->get_parameter("sim_time_step").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(sim_time_step_),
        std::bind(&SimulatorInterface::simulatorCallback, this));

    initState();
  }

 private:
  void initState() {
    SimState state;
    auto p = this->get_parameter("init_p").as_double_array();
    auto v = this->get_parameter("init_v").as_double_array();
    auto q = this->get_parameter("init_q").as_double_array();
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
    std::cout << "init_p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    std::cout << "init_v: " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    std::cout << "init_q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
              << std::endl;
    quat.normalize();
    state.p = Eigen::Vector3d(p[0], p[1], p[2]);
    state.v = Eigen::Vector3d(v[0], v[1], v[2]);
    state.R = quat.toRotationMatrix();
    tailsitter_simulator_.setState(state);
  }

  void controlCallback(const ControlMsg::SharedPtr msg) {
    // 从 msg 中获取控制量
    // 设置到 tailsitter_simulator_ 中
    // lock_guard for the variable Control
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      control_.thrust_vec[0] = msg->motors_u[0];
      control_.thrust_vec[1] = msg->motors_u[1];
      control_.thrust_vec[2] = msg->motors_u[2];
      control_.thrust_vec[3] = msg->motors_u[3];
    }
    RCLCPP_INFO(this->get_logger(), "MPC controls: [%f %f %f %f]",
                control_.thrust_vec[0], control_.thrust_vec[1],
                control_.thrust_vec[2], control_.thrust_vec[3]);
  }

  void simulatorCallback() {
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      tailsitter_simulator_.setControl(control_);
    }

    tailsitter_simulator_.step(sim_time_step_);
    publishMpcState();
    publishOdometry();
  }

  void publishMpcState() {
    static SimState sim_state;
    static MpcStateMsg mpc_state_msg;
    // now, only update mpc_state_msg 中的 mpc_state
    sim_state = tailsitter_simulator_.getState();
    auto &msg_array = mpc_state_msg.mpc_state;  // p v r q
    auto mpc_state_eigen_array =
        Eigen::Map<Eigen::Array<double, 13, 1>>(msg_array.data());

    mpc_state_eigen_array << sim_state.p.array(), sim_state.v.array(),
        sim_state.omega.array(), rotToQuat(sim_state.R).array();

    mpc_state_publisher_->publish(mpc_state_msg);
  }

  void publishOdometry() {
    static SimState sim_state;
    static OdometryMsg odometry_msg;
    sim_state = tailsitter_simulator_.getState();
    odometry_msg.header.stamp = this->now();
    odometry_msg.header.frame_id = "world";
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.pose.pose.position.x = sim_state.p(0);
    odometry_msg.pose.pose.position.y = sim_state.p(1);
    odometry_msg.pose.pose.position.z = sim_state.p(2);
    // sim_state's member is position p and velocity v, rotation matrix R,
    // angular velocity omega
    Eigen::Quaterniond q(sim_state.R);
    odometry_msg.pose.pose.orientation.w = q.w();
    odometry_msg.pose.pose.orientation.x = q.x();
    odometry_msg.pose.pose.orientation.y = q.y();
    odometry_msg.pose.pose.orientation.z = q.z();

    odometry_publisher_->publish(odometry_msg);
  }
};
}  // namespace simulator_interface

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // set thread to 2
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);

  auto sim_node = std::make_shared<simulator_interface::SimulatorInterface>();
  // executor.add_node(sim_node);
  // executor.spin();
  rclcpp::spin(sim_node);
  rclcpp::shutdown();
  return 0;
}
