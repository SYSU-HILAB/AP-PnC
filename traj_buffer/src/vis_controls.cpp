// #include <r
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
      RCLCPP_INFO(
          this->get_logger(), "point %d: %f %f %f %f %f %f %f %f %f", i,
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
        std::bind(&TrajectoryBuffer::ref_pub_cb, this, std::placeholders::_1));

    traj_sub = this->create_subscription<Trajectory>(
        "control_traj", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::update_traj_cb, this,
                  std::placeholders::_1));
  }
};
using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node {
 public:
  MarkerPublisher() : Node("marker_publisher") {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MarkerPublisher::timerCallback, this));
  }

 private:
  void timerCallback() {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.707;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.707;

    // Set the scale of the marker
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(2);

    publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
