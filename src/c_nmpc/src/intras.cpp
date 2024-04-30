#include <thread>

#include "rclcpp/rclcpp.hpp"

class HeavyComputationNode : public rclcpp::Node {
 public:
  HeavyComputationNode() : Node("heavy_computation_node") {
    // 在构造函数中启动线程
    // computation_thread_ =
    //     std::thread(&HeavyComputationNode::heavy_computation, this);
    // computation_thread2_ =
    //     std::thread(&HeavyComputationNode::heavy_computation2, this);
    // create a timer event
    timer = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&HeavyComputationNode::tm_cb, this));
  }

  ~HeavyComputationNode() {
    // 确保在销毁节点前线程能够安全地完成
    computation_thread_.join();
    computation_thread2_.join();
  }

 private:
  std::thread computation_thread_;
  std::thread computation_thread2_;
  // timer
  // callback
  rclcpp::TimerBase::SharedPtr timer;
  void tm_cb() {
    static int count = 0;
    RCLCPP_INFO(this->get_logger(), "Timer callback %d", count++);
    computation_thread_ =
        std::thread(&HeavyComputationNode::heavy_computation, this);
    computation_thread_.detach();
    computation_thread2_ =  std::thread(&HeavyComputationNode::heavy_computation2, this);
    computation_thread2_.detach();
  }
  void heavy_computation() {
    static int count = 0;
    int var = count;
    RCLCPP_INFO(this->get_logger(), "111111111 started. %d ", count++);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // 模拟耗时操作
    RCLCPP_INFO(this->get_logger(), "111111111 finished. %d", var);
  }
  void heavy_computation2() {
    static int count = 0;
    int var = count;
    RCLCPP_INFO(this->get_logger(), "2222222222222 started. %d", count++);
    std::this_thread::sleep_for(std::chrono::seconds(5));  // 模拟耗时操作
    RCLCPP_INFO(this->get_logger(), "2222222222222 finished. %d", var);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeavyComputationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
