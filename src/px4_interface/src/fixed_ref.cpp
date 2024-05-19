#include <Eigen/Dense>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "calculate_cz.cpp"

using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory = interface::msg::RefTrajectory;
constexpr int kRefPointDims = 16;
constexpr int kYbStartIndex = 9;
bool if_quadrotor = false;

class FixedRefCallbacks {
public:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
  rclcpp::Publisher<TrajecotryBullet>::SharedPtr publisher_;

  const rclcpp::QoS reliabe_qos_keep_last =
      rclcpp::QoS(10)
          .reliable()
          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
          .keep_last(1);

public:
  virtual void initSubsPubs() = 0;
  virtual void
  refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) = 0;
};

class FixedRef : public rclcpp::Node, private FixedRefCallbacks {
private:
  Trajectory trajectory_;
  TrajecotryBullet message_;
  TrajecotryPoint point_;

private:
  void initSubsPubs() override;
  void refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) override;

  void inline declareParameter();
  void inline updateParameter();
  std::vector<double> pos_ = {0.0, 0.0, 2.0};

public:
  FixedRef();
};
FixedRef::FixedRef() : Node("trajectory_buffer") {
  initSubsPubs();
  declareParameter();
  updateParameter();
}

void FixedRef::declareParameter() {
  // array of double
  this->declare_parameter<std::vector<double>>("pos", pos_);
}

void FixedRef::updateParameter() {
  this->get_parameter("pos", pos_);
  RCLCPP_INFO(this->get_logger(), "Vector param: [%f, %f, %f]", pos_[0],
              pos_[1], pos_[2]);
  for (auto &point : message_.points) {
    point.point[0] = pos_[0];
    point.point[1] = pos_[1];
    point.point[2] = pos_[2];
    // the left is zero initialized
    for (int i = 3; i < kRefPointDims; i++) {
      point.point[i] = 0.0;
    }

    point.point[kYbStartIndex + 1] = 1.0; // 1,0,0,  Yaw 90
  }
}

void FixedRef::refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    // interface::msg::TrajectoryPoint point =
    // trajectory_.points[index_];
    publisher_->publish(message_);
  }
};

void FixedRef::initSubsPubs() {
  bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "ref_can_set", reliabe_qos_keep_last,
      std::bind(&FixedRef::refTrajectoryBulletPublish, this,
                std::placeholders::_1));

  publisher_ = this->create_publisher<TrajecotryBullet>("ref_trajectory_bullet",
                                                        reliabe_qos_keep_last);
};

int main(int argc, char **argv) {
#ifdef QUADROTOR
  if_quadrotor = true;
#endif

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedRef>());
  rclcpp::shutdown();
  return 0;
}
