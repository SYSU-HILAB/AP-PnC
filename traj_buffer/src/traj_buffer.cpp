// #include <r
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory = interface::msg::RefTrajectory;
// TrajecotryPoint point_;
#include <interface/msg/state.hpp>

#include "calculate_cz.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// csv Writer
#include "traj_buffer/CSVWriter.hpp"
constexpr double kUpdateIncrementTime = 0.02;
constexpr int kRefStateDims = 9;
constexpr int kRefTrajBulletSize = TrajecotryBullet::SIZE;
constexpr double discount = 0.99;
class TrajectoryBuffer : public rclcpp::Node {
 private:
  rclcpp::Publisher<TrajecotryBullet>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<interface::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      ref_bullet_vis;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  int index_ = 0;
  bool init_traj_set_ = false;
  double cz_predict_ = 0.0;

  Trajectory trajectory_;
  TrajecotryBullet message_;
  TrajecotryPoint point_;
  struct VisData {
    int count = 0;
    double alpha, beta;
    double alpha_deg, beta_deg;
    double cz;  // up
    double cur_t = 0;
    Eigen::Quaterniond q;
    Eigen::Vector3d p, v, r;
    Eigen::Vector3d ref_p, ref_v, ref_r;
    Eigen::Vector3d aero_force;
    double rmse = 0;
    double rmse_xy = 0;
    CSVWriter csv_writer;

    VisData() : csv_writer(";") {
      // NOW DONT UPDATES
      aero_force = Eigen::Vector3d(0, 0, 0);
      csv_writer << "Time"
                 << "Beta"
                 << "Alpha"
                 << "Cz"
                 << "Px"
                 << "Py"
                 << "Pz"
                 << "RefPx"
                 << "RefPy"
                 << "RefPz"
                 << "V"
                 << "Vx"
                 << "Vy"
                 << "Vz"
                 << "RefV"
                 << "RefVx"
                 << "RefVy"
                 << "RefVz"
                 << "Rate"
                 << "Ratex"
                 << "Ratey"
                 << "Ratez"
                 << "RefRate"
                 << "RefRatex"
                 << "RefRatey"
                 << "RefRatez"
                 << "AeroForce"
                 << "AeroForcex"
                 << "AeroForcey"
                 << "AeroForcez"
                 << "PosErrorNorm"
                 << "RMSE"
                 << "PosErrorXY"
                 << "RMSE_XY"
                 << "\n";
    }

    void update_vis_data(const TrajecotryPoint& point) {
      ref_p = Eigen::Vector3d(point.point[0], point.point[1], point.point[2]);
      ref_v = Eigen::Vector3d(point.point[3], point.point[4], point.point[5]);
      ref_r = Eigen::Vector3d(point.point[6], point.point[7], point.point[8]);
    }
    void update_vis_data(const interface::msg::State& state) {
      p = Eigen::Vector3d(state.mpc_state[0], state.mpc_state[1],
                          state.mpc_state[2]);
      v = Eigen::Vector3d(state.mpc_state[3], state.mpc_state[4],
                          state.mpc_state[5]);
      r = Eigen::Vector3d(state.mpc_state[6], state.mpc_state[7],
                          state.mpc_state[8]);
      q = Eigen::Quaterniond(state.mpc_state[9], state.mpc_state[10],
                             state.mpc_state[11], state.mpc_state[12]);
    }
    void update_aeros() {
      Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;
      // calculate alpha  z/x  but x and z interchange
      // calculate beta_norm
      if (v.norm() < 0.5) {
        alpha = 0;
        beta = 0;
      } else {
        alpha = std::atan2(vb(0), vb(2));
        beta = std::asin(vb(1) / vb.norm());
        alpha_deg = alpha * 180 / M_PI;
        beta_deg = beta * 180 / M_PI;
      }
      // aero_force = Eigen::Vector3d(0, 0, get_cz_from_vb(vb));
      cz = get_cz_from_vb(vb);
    }

    void write_to_csv() {
      Eigen::Vector2d xy;
      auto p_e = ref_p - p;
      xy << p_e(0), p_e(1);
      double surprise = (ref_p - p).norm();
      double suprise_xy = xy.norm();
      if (count == 0)
        surprise = 0.0;
        suprise_xy = 0.0;
      rmse_xy = sqrt(
        (suprise_xy*suprise_xy + rmse_xy * rmse_xy * count) / (count + 1)
      );
      rmse = sqrt(
        (surprise * surprise + rmse * rmse * count) / (count + 1)
      );
      count++;
      csv_writer.newRow() << cur_t << beta_deg << alpha_deg << cz << p(0)
                          << p(1) << p(2) << ref_p(0) << ref_p(1) << ref_p(2)
                          << v.norm() << v(0) << v(1) << v(2)
                          << ref_v.norm() << ref_v(0) << ref_v(1) << ref_v(2) 
                          << r.norm() << r(0) << r(1) << r(2) 
                          << ref_r.norm() << ref_r(0) << ref_r(1) << ref_r(2) 
                          << aero_force.norm() << aero_force(0) << aero_force(1) << aero_force(2)
                          << surprise << rmse << suprise_xy << rmse_xy;
      cur_t += kUpdateIncrementTime;
    }

    ~VisData() { csv_writer.writeToFile("vis_data.csv"); }
  };

  VisData vis_data_;

 public:
  TrajectoryBuffer() : Node("trajectory_buffer") {
    auto reliabe_qos_keep_last =
        rclcpp::QoS(1)
            .reliable()
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .keep_last(1);

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    // declareParam();
    ref_bullet_vis =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "ref_bullet_vis", reliabe_qos_keep_last);

    publisher_ = this->create_publisher<TrajecotryBullet>(
        "ref_trajecotry_bullet", reliabe_qos_keep_last);
    bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ref_can_set", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::ref_pub_cb, this, std::placeholders::_1));

    traj_sub_ = this->create_subscription<Trajectory>(
        "control_traj", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::update_traj_cb, this,
                  std::placeholders::_1));

    state_sub_ = this->create_subscription<interface::msg::State>(
        "cur_state", sensor_qos,
        std::bind(&TrajectoryBuffer::state_cb, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(1000),
    //     std::bind(&TrajectoryBuffer::timer_cb, this));
  }
  // void timer_cb() {
  //   RCLCPP_INFO(this->get_logger(), "Wrting to csv");
  //   vis_data_.write_to_csv();
  // }
  void state_cb(const interface::msg::State::SharedPtr msg) {
    vis_data_.update_vis_data(*msg);
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
    std::cerr << "cz_predict_ " << cz_predict_ << std::endl;
    RCLCPP_INFO(this->get_logger(), "Entering ref_buffer state cz %f",
                cz_predict_);
  }

  void ref_pub_cb(std_msgs::msg::Bool::SharedPtr msg) {
    if (index_ == trajectory_.points.size()) {
      return;
    }
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "ref_pub_cb");
      RCLCPP_INFO(this->get_logger(), "index_ %d", index_);
      auto vis_msg = visualization_msgs::msg::MarkerArray();
      double now_discount = 1.0;
      vis_data_.update_vis_data(trajectory_.points[index_]);
      for (int i = index_; i < index_ + kRefTrajBulletSize; i++) {
        if (i < trajectory_.points.size()) {
          message_.points[i - index_] = trajectory_.points[i];
        } else {
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
      // vis_msg.markers.

      // write to csv
      vis_data_.update_aeros();
      vis_data_.write_to_csv();
    }
  }

  void declareParam() {
    this->declare_parameter("target_position", std::vector<double>{0, 0, 10});
  }

  void update_traj_cb(const Trajectory::SharedPtr msg) {
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
  }
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryBuffer>());
  rclcpp::shutdown();
  return 0;
}
