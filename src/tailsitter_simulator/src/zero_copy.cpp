// #include <r
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;

TrajecotryPoint point_;

class RefTrajLis : public rclcpp::Node {
 private:
  rclcpp::Subscription<TrajecotryBullet>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  //   TrajecotryBullet message_;
  TrajecotryPoint point_;

  void declareParam() {
    this->declare_parameter("target_position", std::vector<double>{0, 0, 10});
  }

  void sub_callback(const TrajecotryBullet::SharedPtr msg) {
    // transfer msg time to rclcpp time
    rclcpp::Time msg_time(msg->timestamp);
    auto now_time = this->now();
    auto diff = (now_time - msg_time);
    RCLCPP_INFO(this->get_logger(), "timedff: '%f'", diff.seconds());
  }

 public:
  RefTrajLis() : Node("RefTrajLis") {
    declareParam();
    sub_ = this->create_subscription<TrajecotryBullet>(
        "ref_trajecotry_bullet", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&RefTrajLis::sub_callback, this, std::placeholders::_1));

    auto pos_vary = this->get_parameter("target_position").as_double_array();
    assert(pos_vary.size() == 3);
    point_.point = {pos_vary[0], pos_vary[1], pos_vary[2], 0, 0, 0, 0, 0, 0};
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RefTrajLis>());
  rclcpp::shutdown();
  return 0;
}

