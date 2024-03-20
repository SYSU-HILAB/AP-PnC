// #include <r
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory = interface::msg::RefTrajectory;
// TrajecotryPoint point_;

constexpr int kRefStateDims = 9;
constexpr int kRefTrajBulletSize = TrajecotryBullet::SIZE;
class TrajectoryBuffer : public rclcpp::Node {
 private:
  rclcpp::Publisher<TrajecotryBullet>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub;

  int index_ = 0;
  bool init_traj_set_ = false;

  Trajectory trajectory_;
  TrajecotryBullet message_;
  TrajecotryPoint point_;
  void ref_pub_cb(std_msgs::msg::Bool::SharedPtr msg) {
    if (index_ == trajectory_.points.size()) {
      return;
    }
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "ref_pub_cb");
      RCLCPP_INFO(this->get_logger(), "index_ %d", index_);

      for (int i = index_; i < index_ + kRefTrajBulletSize; i++) {
        if (i < trajectory_.points.size()) {
          message_.points[i - index_] = trajectory_.points[i];
        } else {
          message_.points[i - index_] = trajectory_.points.back();
        }
      }
      index_ += 1;
      publisher_->publish(message_);
    }
  }

  void declareParam() {
    this->declare_parameter("target_position", std::vector<double>{0, 0, 10});
  }

  void init() {
    std::fill(std::begin(message_.points), std::end(message_.points), point_);
    publisher_->publish(message_);
  }

  void update_traj_cb(const Trajectory::SharedPtr msg) {
    if (init_traj_set_ == false) {
      trajectory_.points.clear();
      // copy msg to trajectory_
      trajectory_.points = msg->points;
      init_traj_set_ = true;
    }
    // print trajectory. point is an array {p[0], p[1], p[2], v[0], v[1], v[2],
    // r[0], r[1], r[2]}
    for (int i = 0; i < trajectory_.points.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "point %d: %f %f %f %f %f %f %f %f %f", i,
                  trajectory_.points[i].point[0], trajectory_.points[i].point[1],
                  trajectory_.points[i].point[2], trajectory_.points[i].point[3],
                  trajectory_.points[i].point[4], trajectory_.points[i].point[5],
                  trajectory_.points[i].point[6], trajectory_.points[i].point[7],
                  trajectory_.points[i].point[8]);
    }
  }

   public:
    TrajectoryBuffer() : Node("trajectory_buffer") {
      auto reliabe_qos_keep_last =
          rclcpp::QoS(1)
              .reliable()
              .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
              .keep_last(1);
      // declareParam();
      publisher_ = this->create_publisher<TrajecotryBullet>(
          "ref_trajecotry_bullet", reliabe_qos_keep_last);
      bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "ref_can_set", reliabe_qos_keep_last,
          std::bind(&TrajectoryBuffer::ref_pub_cb, this,
                    std::placeholders::_1));

      traj_sub = this->create_subscription<Trajectory>(
          "control_traj", reliabe_qos_keep_last,
          std::bind(&TrajectoryBuffer::update_traj_cb, this,
                    std::placeholders::_1));
    }
  };
  int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryBuffer>());
    rclcpp::shutdown();
    return 0;
  }
