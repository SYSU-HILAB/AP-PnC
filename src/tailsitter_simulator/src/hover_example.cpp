// #include <r
#include <rclcpp/rclcpp.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>

#include <interface/msg/trajectory_point.hpp>

using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;

TrajecotryPoint point_;

class HoverExample : public rclcpp::Node {
private:
    rclcpp::Publisher<TrajecotryBullet>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    TrajecotryBullet message_;
    TrajecotryPoint point_;
    void timer_callback() {
        std::fill(std::begin(message_.points), std::end(message_.points), point_);
        message_.timestamp = this->now();
        publisher_->publish(message_);

    }

    void declareParam() {
        this->declare_parameter("target_position", std::vector<double>{0, 0, 10});
    }
public:
    HoverExample() : Node("hover_example") {
      declareParam();
        publisher_ = this->create_publisher<TrajecotryBullet>("ref_trajecotry_bullet", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&HoverExample::timer_callback, this));

        auto pos_vary = this->get_parameter("target_position").as_double_array();
        assert(pos_vary.size() == 3);
        point_.point = {pos_vary[0], pos_vary[1], pos_vary[2], 0, 0, 0, 0, 0, 0};
    }
};
int main (int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoverExample>());
    rclcpp::shutdown();
    return 0;
}