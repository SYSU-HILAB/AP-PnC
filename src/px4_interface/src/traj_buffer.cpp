#include <rclcpp/rclcpp.hpp>
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <interface/msg/state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

#include "calculate_cz.cpp"

using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory = interface::msg::RefTrajectory;
constexpr double kUpdateIncrementTime = 0.02;
constexpr int kRefStateDims = 9;
constexpr int kRefTrajBulletSize = TrajecotryBullet::SIZE;
constexpr double discount = 0.99;
bool if_quadrotor = false;

class TrajectoryBufferCallbacks {
public:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<interface::msg::State>::SharedPtr state_sub_;

  rclcpp::Publisher<TrajecotryBullet>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ref_bullet_vis;
  const rclcpp::QoS reliabe_qos_keep_last =
    rclcpp::QoS(1)
    .reliable()
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .keep_last(1);
public:
  virtual void initSubsPubs() = 0;
  virtual void stateCallback(const interface::msg::State::SharedPtr msg) = 0;
  virtual void refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) = 0;
  virtual void updateTrajCallback(const Trajectory::SharedPtr msg) = 0;
};



class TrajectoryBuffer : public rclcpp::Node, private TrajectoryBufferCallbacks {
private:
  int index_ = 0;
  bool init_traj_set_ = false;
  double cz_predict_ = 0.0;
  Trajectory trajectory_;
  TrajecotryBullet message_;
  TrajecotryPoint point_;
private:
  void initSubsPubs();
  void stateCallback(const interface::msg::State::SharedPtr msg) override;
  void refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) override;
  void updateTrajCallback(const Trajectory::SharedPtr msg) override;
public:
  TrajectoryBuffer();

};
TrajectoryBuffer::TrajectoryBuffer() : Node("trajectory_buffer") {
  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  initSubsPubs();
}

// state callback is to update cz_predict
void TrajectoryBuffer::stateCallback(const interface::msg::State::SharedPtr msg) {
  auto state = msg->mpc_state;  // p,v,r,q
  // get v and q to eigen
  auto v = Eigen::Vector3d(state[3], state[4], state[5]);

  double v_norm = v.norm();
  if (v_norm < 1) {
    cz_predict_ = 0.0;
    return;
  }
  auto q = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
  Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;

  cz_predict_ = get_cz_from_vb(vb);
  if (if_quadrotor)
    cz_predict_ = 0.0;

  std::cerr << "cz_predict_ " << cz_predict_ << std::endl;
  RCLCPP_INFO(this->get_logger(), "Entering ref_buffer state cz %f",
    cz_predict_);
};

void TrajectoryBuffer::refTrajectoryBulletPublish(std_msgs::msg::Bool::SharedPtr msg) {
  if (index_ == trajectory_.points.size()) {
    return;
  }
  if (msg->data) {
    RCLCPP_INFO(this->get_logger(), "refTrajectoryBulletPublish");
    RCLCPP_INFO(this->get_logger(), "index_ %d", index_);
    auto vis_msg = visualization_msgs::msg::MarkerArray();
    double now_discount = 1.0;
    for (int i = index_; i < index_ + kRefTrajBulletSize; i++) {
      if (i < trajectory_.points.size()) {
        message_.points[i - index_] = trajectory_.points[i];
      }
      else {
        message_.points[i - index_] = trajectory_.points.back();
      }
      now_discount *= discount;
      message_.points[i - index_].cz_prediction = now_discount * cz_predict_;

      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.scale.x = 1.0;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 0.2;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.pose.position.x = message_.points[i - index_].point[0];
      marker.pose.position.y = message_.points[i - index_].point[1];
      marker.pose.position.z = message_.points[i - index_].point[2];
      marker.id = i - index_;
      marker.ns = "ref_bullet";
      auto yb = Eigen::Vector3d();
      yb(0) =
        message_.points[i - index_].point[TrajecotryPoint::YB_START_INDEX];
      yb(1) = message_.points[i - index_]
        .point[TrajecotryPoint::YB_START_INDEX + 1];
      yb(2) = message_.points[i - index_]
        .point[TrajecotryPoint::YB_START_INDEX + 2];

      auto rot_axis = Eigen::Vector3d::UnitX().cross(yb);
      auto rot_angle = std::acos(Eigen::Vector3d::UnitX().dot(yb));
      // from rot axis and angle to construct quaternion
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(rot_angle, rot_axis.normalized());
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      vis_msg.markers.push_back(marker);
    }
    index_ += 1;
    publisher_->publish(message_);
    ref_bullet_vis->publish(vis_msg);

  }
};


void TrajectoryBuffer::updateTrajCallback(const Trajectory::SharedPtr msg) {
  if (init_traj_set_ == false) {
    trajectory_.points.clear();
    // copy msg to trajectory_
    trajectory_.points = msg->points;
    init_traj_set_ = true;
    RCLCPP_INFO(this->get_logger(), "trajectory_ size %d",
      trajectory_.points.size());
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

  // Trajectory is received, waiitng for ref_can_set to be true . 
  // Warning Print that message.
  RCLCPP_WARN(this->get_logger(), "Trajectory is received, waiitng for /ref_can_set to be true");
};


void TrajectoryBuffer::initSubsPubs() {
  bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "ref_can_set", reliabe_qos_keep_last,
    std::bind(&TrajectoryBuffer::refTrajectoryBulletPublish, this, std::placeholders::_1));

  traj_sub_ = this->create_subscription<Trajectory>(
    "control_traj", reliabe_qos_keep_last,
    std::bind(&TrajectoryBuffer::updateTrajCallback, this,
      std::placeholders::_1));

  state_sub_ = this->create_subscription<interface::msg::State>(
    "cur_state", reliabe_qos_keep_last,
    std::bind(&TrajectoryBuffer::stateCallback, this, std::placeholders::_1));

  ref_bullet_vis =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ref_bullet_vis", reliabe_qos_keep_last);

  publisher_ = this->create_publisher<TrajecotryBullet>(
    "ref_trajectory_bullet", reliabe_qos_keep_last);
};


int main(int argc, char** argv) {
#ifdef QUADROTOR
  if_quadrotor = true;
#endif
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryBuffer>());
  rclcpp::shutdown();
  return 0;
}
