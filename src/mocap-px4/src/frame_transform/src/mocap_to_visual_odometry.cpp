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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#include <frame_transform/frame_transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

class AlphaBetaFilter
{
 public:
  double alpha = 0.2;
  double beta  = 0.05;
  double x_est;
  double v_est;
  double last_timestamp = -1.0;

  // return velocity estimate
  double update(double x_measured, double timestamp)
  {
    if (last_timestamp < 0)
    {
      last_timestamp = timestamp;
      x_est          = x_measured;
      v_est          = 0;
      return v_est;
    }

    double dt = timestamp - last_timestamp;

    if (dt > 0)
    {
      // Predict
      double x_pred = x_est + v_est * dt;

      // Update
      double residual = x_measured - x_pred;
      x_est           = x_pred + alpha * residual;
      v_est           = v_est + (beta / dt) * residual;

      last_timestamp = timestamp;
    }
    return v_est;
    return v_est;
  }

  std::pair<double, double> predict(double timestamp) const
  {
    double dt     = timestamp - last_timestamp;
    double x_pred = x_est + v_est * dt;
    return std::make_pair(x_pred, v_est);
  }

  double getPositionEstimate() const { return x_est; }
  double getVelocityEstimate() const { return v_est; }
};

// #include <rclcpp
// #include
using namespace px4_ros_interface::frame_transforms;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;

void extractTransform(const Eigen::Affine3d& transform,
                      Eigen::Vector3d&       translation,
                      Eigen::Quaterniond&    rotation)
{
  translation = transform.translation();                   // 获取平移部分
  rotation    = Eigen::Quaterniond(transform.rotation());  // 获取旋转部分
}

class MocapToVisualOdometry : public rclcpp::Node
{
  using VehicleOdometry         = px4_msgs::msg::VehicleOdometry;
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

 public:
  MocapToVisualOdometry() : Node("mocap_to_visual_odometry")
  {
    // 创建订阅者，订阅/mocap/odometry话题，指定回调函数
    RCLCPP_INFO(this->get_logger(), "MocapToVisualOdometry is Running!");
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mocap/odometry", qos,
        std::bind(&MocapToVisualOdometry::mocapCallback, this,
                  std::placeholders::_1));

    // 创建发布者，发布到/fmu/in/visual_odometry话题
    publisher_ = this->create_publisher<VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", qos);
    // declare paramter
    this->declare_parameter("time_unit", .033);
    this->declare_parameter("alpha", .2);
    this->declare_parameter("beta", .05);
    this->get_parameter("time_unit", time_unit_);
    double alpha, beta;
    this->get_parameter("alpha", alpha);
    this->get_parameter("beta", beta);
    for (auto& filter : filters)
    {
      filter.alpha = alpha;
      filter.beta  = beta;
    }
  }

 private:
  void calAverageWindowedOdom()
  {
    auto quatTemp = arrayQuaterniond[0];
    for (const auto& quat : arrayQuaterniond)
    {
      avgQuat.coeffs() += quat.coeffs();
    }
    avgQuat.coeffs() /= static_cast<double>(arrayQuaterniond.size());
    avgQuat.normalize();
    // return sumQuat;
    for (const auto& vec : arrayPosd)
    {
      avgPos += vec;
    }

    avgPos /= static_cast<double>(arrayPosd.size());
  }

  void mocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // pos_enu is actually flu local frame!
    // pos_ned is actually frd local frame!
    // That is the same for quat_enu, quat_ned

    Eigen::Vector3d pos_enu;
    tf2::fromMsg(msg->pose.position, pos_enu);

    Eigen::Quaterniond q_enu;
    tf2::fromMsg(msg->pose.orientation, q_enu);
    q_enu.normalize();
    if (q_enu.w() < 0)
    {
      q_enu.coeffs() *= -1.0;
    }
    if (!intialized)
    {
      arrayPosd.push_back(pos_enu);
      arrayQuaterniond.push_back(q_enu);
      count += 1;
      if (count == window_size)
      {
        calAverageWindowedOdom();
        intialized = true;
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "=========================================================="
            "============================= ");
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "Position Estimation is OK and Offboard Control is ready "
            "to go!");
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Bias pos: " << avgPos.transpose() << " Bias quat: "
                                        << avgQuat.coeffs().transpose());
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "=========================================================="
            "============================= ");
      }
      return;
    }
    pos_enu = avgQuat.inverse() * (pos_enu - avgPos);
    q_enu   = avgQuat.inverse() * q_enu;

    // RCLCPP_WARN_STREAM(this->get_logger(), "Pos: " << pos_enu.transpose()
    // << " Quat: " << q_enu.coeffs().transpose());

    auto vo_msg           = VehicleOdometry();
    vo_msg.pose_frame     = VehicleOdometry::POSE_FRAME_NED;
    vo_msg.velocity_frame = VehicleOdometry::POSE_FRAME_NED;
    // auto q_enu = msg->orientation;
    // tf2::toMsg(*msg);

    Eigen::Quaterniond q_ned = frd_to_flu_orientation(
        baselink_to_aircraft_orientation(q_enu.normalized()));
    q_ned.normalize();
    if (q_ned.w() < 0)
    {
      q_ned.coeffs() *= -1.0;
    }
    Eigen::Vector3d pos_ned = transform_frame(pos_enu, AIRCRAFT_BASELINK_Q);
    printf("filter %f", filters[0].alpha);

    vo_msg.position[0] = pos_ned(0);
    vo_msg.position[1] = pos_ned(1);
    vo_msg.position[2] = pos_ned(2);

    vo_msg.velocity[0] = (float)filters[0].update(
        pos_ned(0), static_cast<double>(msg->header.stamp.sec) +
                        static_cast<double>(msg->header.stamp.nanosec) / 1e9);
    vo_msg.velocity[1] = filters[1].update(
        pos_ned(1), static_cast<double>(msg->header.stamp.sec) +
                        static_cast<double>(msg->header.stamp.nanosec) / 1e9);
    vo_msg.velocity[2] = filters[2].update(
        pos_ned(2), static_cast<double>(msg->header.stamp.sec) +
                        static_cast<double>(msg->header.stamp.nanosec) / 1e9);

    vo_msg.q[0] = q_ned.w();
    vo_msg.q[1] = q_ned.x();
    vo_msg.q[2] = q_ned.y();
    vo_msg.q[3] = q_ned.z();

    vo_msg.timestamp_sample =
        msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    vo_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // TODO add a Kalman filter to get velocity

    // tf2::convert()
    // vo_msg.position = msg->position;
    // vo_msg.position = frame_transforms::ros_to_px4_orientation
    // 在这里将接收到的消息再发布到/fmu/in/visual_odometry话题上
    if (intialized)
    {
      // 30Hz
      if (vo_msg.timestamp - lastPublishedTime > time_unit_ * 1e6)
      {
        publisher_->publish(vo_msg);
        lastPublishedTime = vo_msg.timestamp;
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
                                                subscription_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr publisher_;

  bool                            intialized = false;
  double                          time_unit_;
  constexpr static int            window_size = 50;
  std::vector<Eigen::Vector3d>    arrayPosd;
  std::vector<Eigen::Quaterniond> arrayQuaterniond;
  Eigen::Quaterniond avgQuat           = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d    avgPos            = Eigen::Vector3d(0.0, 0.0, 0.0);
  uint64_t           lastPublishedTime = 0;
  std::array<AlphaBetaFilter, 3> filters;
  int                            count = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MocapToVisualOdometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
