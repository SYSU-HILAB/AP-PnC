#include <basic_trajectories/lemniscate.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
   private:
    rclcpp::TimerBase::SharedPtr                                timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    std::unique_ptr<basic_trajectories::Lemniscate>             lem;

   public:
    MinimalPublisher() : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "basic_traj", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
        lem = std::make_unique<basic_trajectories::Lemniscate>(2, sqrt(2), 5);

        auto arr = lem->samplingPointArray(50);
        // print  arr
        for (auto& p : arr)
        {
            std::cout << p << std::endl;
        }
    }

   private:
    void timer_callback()
    {
        auto message            = lem->toMsg(50);
        message.header.frame_id = "world";
        message.header.stamp    = this->get_clock()->now();
        publisher_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();

    return 0;
}