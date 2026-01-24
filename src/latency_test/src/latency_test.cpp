/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Latency test for zero-copy communication between traj_buffer and c_nmpc.
 * Uses rclcpp subscription with loaned message support.
 * The middleware (rmw_fastrtps_cpp) uses zero-copy internally when available.
 */

#include <chrono>
#include <cmath>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <mutex>

using namespace std::chrono_literals;
using RefTrajectoryBullet = interface::msg::RefTrajectoryBullet;

class LatencyTestNode : public rclcpp::Node
{
public:
  LatencyTestNode() : Node("latency_test_node"),
                       message_count_(0),
                       total_latency_us_(0.0),
                       min_latency_us_(std::numeric_limits<double>::max()),
                       max_latency_us_(0.0),
                       ros_clock_(RCL_ROS_TIME),
                       start_time_(ros_clock_.now())
  {
    // QoS - match the publisher's settings
    const rclcpp::QoS reliable_qos =
        rclcpp::QoS(10)
            .reliable()
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .keep_last(1);

    // Create subscription with loaned message support
    // Note: While this uses a callback, the middleware (rmw_fastrtps_cpp) can still
    // use zero-copy internally when:
    // 1. Shared memory transport is enabled (fastdds.xml)
    // 2. Publisher uses borrow_loaned_message() (traj_buffer does this)
    // 3. Message type is POD (Plain Old Data)
    subscription_ = this->create_subscription<RefTrajectoryBullet>(
        "/ref_trajectory_bullet",
        reliable_qos,
        [this](RefTrajectoryBullet::UniquePtr msg)
        {
          this->handle_message(std::move(msg));
        }
    );

    // Check if loaned messages are supported
    const auto* rcl_sub = subscription_->get_subscription_handle().get();
    if (rcl_sub && rcl_subscription_can_loan_messages(rcl_sub))
    {
      RCLCPP_INFO(this->get_logger(), "Zero-copy subscription: SUPPORTED by middleware");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Zero-copy subscription: NOT SUPPORTED - messages will be copied");
    }

    // Create publisher to trigger traj_buffer (simulates c_nmpc behavior)
    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ref_can_set", reliable_qos);

    // Create timer for periodic statistics reporting
    stats_timer_ = this->create_wall_timer(
        5s,
        [this]() { this->report_stats(); }
    );

    // Create timer to periodically request trajectory bullets (simulates c_nmpc control loop)
    trigger_timer_ = this->create_wall_timer(
        20ms,  // 50Hz - matches the expected control frequency
        [this]()
        {
          auto msg = std_msgs::msg::Bool();
          msg.data = true;
          trigger_pub_->publish(msg);
        });

    // Parameter for test duration (0 = run forever)
    this->declare_parameter("max_messages", 0);
    this->get_parameter("max_messages", max_messages_);

    RCLCPP_INFO(this->get_logger(), "Latency Test Node Started");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /ref_can_set (trigger)");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /ref_trajectory_bullet");
    RCLCPP_INFO(this->get_logger(), "Max messages: %d (0 = unlimited)", max_messages_);

    // Check zero-copy environment
    const char* disable_loaned = std::getenv("ROS_DISABLE_LOANED_MESSAGES");
    if (disable_loaned != nullptr && std::string(disable_loaned) == "0")
    {
      RCLCPP_INFO(this->get_logger(), "Zero-copy: ENABLED (ROS_DISABLE_LOANED_MESSAGES=0)");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Zero-copy: DISABLED (set ROS_DISABLE_LOANED_MESSAGES=0 to enable)");
    }
  }

private:
  // Handle message (callback-based, but middleware may use zero-copy internally)
  void handle_message(RefTrajectoryBullet::UniquePtr msg)
  {
    auto now = ros_clock_.now();

    // Skip messages with unset timestamps (both sec and nanosec are 0)
    if (msg->timestamp.sec == 0 && msg->timestamp.nanosec == 0)
    {
      return;
    }

    auto msg_time = rclcpp::Time(msg->timestamp.sec, msg->timestamp.nanosec, RCL_ROS_TIME);

    // Calculate latency in microseconds
    double latency_us = (now - msg_time).nanoseconds() / 1000.0;

    // Only record valid latencies (positive values)
    if (latency_us > 0)
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      latencies_.push_back(latency_us);
      total_latency_us_ += latency_us;
      min_latency_us_ = std::min(min_latency_us_, latency_us);
      max_latency_us_ = std::max(max_latency_us_, latency_us);
      message_count_++;
    }

    // Check if we've reached max messages
    if (max_messages_ > 0)
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      if (message_count_ >= static_cast<size_t>(max_messages_))
      {
        report_stats();
        RCLCPP_INFO(this->get_logger(), "Test completed. Shutting down...");
        rclcpp::shutdown();
      }
    }
  }

  void report_stats()
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    if (message_count_ == 0)
    {
      RCLCPP_WARN(this->get_logger(), "No messages received yet - make sure traj_buffer is running!");
      return;
    }

    // Calculate statistics
    double avg_latency_us = total_latency_us_ / message_count_;

    // Calculate median
    std::vector<double> sorted_latencies = latencies_;
    std::sort(sorted_latencies.begin(), sorted_latencies.end());
    double median_us;
    size_t n = sorted_latencies.size();
    if (n % 2 == 0)
    {
      median_us = (sorted_latencies[n/2 - 1] + sorted_latencies[n/2]) / 2.0;
    }
    else
    {
      median_us = sorted_latencies[n/2];
    }

    // Calculate percentiles
    double p50_us = sorted_latencies[static_cast<size_t>(n * 0.50)];
    double p95_us = sorted_latencies[static_cast<size_t>(n * 0.95)];
    double p99_us = sorted_latencies[static_cast<size_t>(n * 0.99)];

    // Calculate standard deviation
    double variance = 0.0;
    for (double lat : latencies_)
    {
      variance += (lat - avg_latency_us) * (lat - avg_latency_us);
    }
    variance /= latencies_.size();
    double std_dev_us = std::sqrt(variance);

    // Print statistics header
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "      Zero-Copy Latency Statistics");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Message Count:           %zu", message_count_);
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Latency (us):");
    RCLCPP_INFO(this->get_logger(), "  Min:                   %10.2f", min_latency_us_);
    RCLCPP_INFO(this->get_logger(), "  Max:                   %10.2f", max_latency_us_);
    RCLCPP_INFO(this->get_logger(), "  Avg:                   %10.2f", avg_latency_us);
    RCLCPP_INFO(this->get_logger(), "  Median:                %10.2f", median_us);
    RCLCPP_INFO(this->get_logger(), "  Std Dev:               %10.2f", std_dev_us);
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Percentiles (us):");
    RCLCPP_INFO(this->get_logger(), "  50th (p50):            %10.2f", p50_us);
    RCLCPP_INFO(this->get_logger(), "  95th (p95):            %10.2f", p95_us);
    RCLCPP_INFO(this->get_logger(), "  99th (p99):            %10.2f", p99_us);
    RCLCPP_INFO(this->get_logger(), "========================================");

    // Print frequency estimate
    double duration_sec = (ros_clock_.now() - start_time_).seconds();
    if (duration_sec > 0)
    {
      double freq = message_count_ / duration_sec;
      RCLCPP_INFO(this->get_logger(), "Receive Frequency:       %.2f Hz", freq);
    }
  }

  // Members
  rclcpp::Subscription<RefTrajectoryBullet>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr trigger_timer_;

  // Statistics (protected by mutex)
  std::mutex stats_mutex_;
  std::vector<double> latencies_;
  size_t message_count_;
  double total_latency_us_;
  double min_latency_us_;
  double max_latency_us_;
  int max_messages_;

  // ROS clock and start time for consistent time measurements
  rclcpp::Clock ros_clock_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LatencyTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
