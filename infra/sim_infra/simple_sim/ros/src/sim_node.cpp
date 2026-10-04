#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>

#include "simple_sim/io/experiment.hpp"

namespace simple_sim
{
  namespace
  {
    builtin_interfaces::msg::Time stamp(std::int64_t ns)
    {
      if (ns < 0 ||
          ns / 1000000000LL > std::numeric_limits<std::int32_t>::max())
        throw std::runtime_error("simulation time exceeds ROS timestamp range");
      builtin_interfaces::msg::Time t;
      t.sec     = static_cast<std::int32_t>(ns / 1000000000LL);
      t.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
      return t;
    }
    geometry_msgs::msg::Vector3 vector(const Eigen::Vector3d &v)
    {
      geometry_msgs::msg::Vector3 out;
      out.x = v.x();
      out.y = v.y();
      out.z = v.z();
      return out;
    }
  }  // namespace

  class SimNode final : public rclcpp::Node
  {
   public:
    SimNode() : Node("simple_sim")
    {
      config_    = load_config(declare_parameter<std::string>("config", ""),
                               declare_parameter<std::string>("controller", ""));
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::ClockQoS());
      odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
      imu_pub_    = create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
      wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
          "~/aero_wrench", 10);
      actual_pub_    = create_publisher<nav_msgs::msg::Path>("~/path", 1);
      reference_pub_ = create_publisher<nav_msgs::msg::Path>(
          "~/reference", rclcpp::QoS(1).transient_local());
      reset_experiment();
      // This timer only schedules complete Runner transactions. It never
      // supplies dt.
      const double control_dt =
          static_cast<double>(experiment_->runner().control_dt_ns());
      const double delay_ns = config_.realtime_factor == 0.0
                                  ? 1000000.0
                                  : control_dt / config_.realtime_factor;
      if (!std::isfinite(delay_ns) ||
          delay_ns >
              static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("ROS pacing period out of range");
      timer_ = create_wall_timer(
          std::chrono::nanoseconds(
              std::max<std::int64_t>(1, std::llround(delay_ns))),
          [this]
          {
            if (experiment_->runner().status() == RunStatus::Paused ||
                experiment_->runner().status() == RunStatus::Finished ||
                experiment_->runner().status() == RunStatus::Failed)
              return;
            try
            {
              advance();
            }
            catch (const std::exception &e)
            {
              RCLCPP_ERROR(get_logger(), "%s", e.what());
              timer_->cancel();
            }
          });
      services_.push_back(create_service<std_srvs::srv::Trigger>(
          "~/pause",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            experiment_->runner().pause();
            response->success =
                experiment_->runner().status() == RunStatus::Paused;
          }));
      services_.push_back(create_service<std_srvs::srv::Trigger>(
          "~/resume",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            experiment_->runner().resume();
            response->success =
                experiment_->runner().status() == RunStatus::Running;
          }));
      services_.push_back(create_service<std_srvs::srv::Trigger>(
          "~/step",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            if (experiment_->runner().status() != RunStatus::Paused)
            {
              response->success = false;
              response->message = "pause before single-stepping";
              return;
            }
            try
            {
              advance();
              response->success = true;
            }
            catch (const std::exception &e)
            {
              response->success = false;
              response->message = e.what();
            }
          }));
      services_.push_back(create_service<std_srvs::srv::Trigger>(
          "~/reset",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            try
            {
              reset_experiment();
              timer_->reset();
              response->success = true;
            }
            catch (const std::exception &e)
            {
              response->success = false;
              response->message = e.what();
              timer_->cancel();
            }
          }));
    }

   private:
    RuntimeConfig                                                   config_;
    std::unique_ptr<Experiment>                                     experiment_;
    rclcpp::TimerBase::SharedPtr                                    timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr         clock_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               actual_pub_,
        reference_pub_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
    nav_msgs::msg::Path actual_path_;
    void                reset_experiment()
    {
      const auto cached_reference =
          experiment_ ? experiment_->reference_handle() : nullptr;
      if (experiment_ && !experiment_->finalized())
        experiment_->finish("interrupted", "user reset");
      auto next = std::make_unique<Experiment>(config_, cached_reference);
      next->runner().pause();    // explicit resume after startup/reset
      config_ = next->config();  // frozen snapshots; reset is not reload/replan
      experiment_                  = std::move(next);
      actual_path_                 = nav_msgs::msg::Path{};
      actual_path_.header.frame_id = "world";
      publish_clock();
      publish_state();
      nav_msgs::msg::Path reference;
      reference.header.frame_id = "world";
      reference.header.stamp    = stamp(0);
      const double duration     = experiment_->trajectory().duration();
      const auto   count = static_cast<std::int64_t>(std::ceil(duration / 0.1));
      for (std::int64_t i = 0; i <= count; ++i)
      {
        const double t     = std::min(duration, static_cast<double>(i) * 0.1);
        const auto   point = experiment_->trajectory().sample(t);
        geometry_msgs::msg::PoseStamped pose;
        pose.header             = reference.header;
        pose.pose.position.x    = point.p.x();
        pose.pose.position.y    = point.p.y();
        pose.pose.position.z    = point.p.z();
        pose.pose.orientation.w = 1.0;
        reference.poses.push_back(pose);
      }
      reference_pub_->publish(reference);
      RCLCPP_INFO(get_logger(),
                  "Paused; output=%s; call /simple_sim/resume to start",
                  experiment_->output_dir().c_str());
    }
    void advance()
    {
      experiment_->step();
      publish_clock();
      if (experiment_->runner().index() % config_.publish_stride == 0 ||
          experiment_->runner().status() == RunStatus::Finished)
        publish_state();
    }
    void publish_clock()
    {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamp(experiment_->runner().time_ns());
      clock_pub_->publish(clock);
    }
    void publish_state()
    {
      const auto             &s = experiment_->runner().state();
      const auto             &f = experiment_->runner().feedback();
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id         = "world";
      odom.child_frame_id          = "base_link";
      odom.header.stamp            = stamp(experiment_->runner().time_ns());
      odom.pose.pose.position.x    = s.position.x();
      odom.pose.pose.position.y    = s.position.y();
      odom.pose.pose.position.z    = s.position.z();
      odom.pose.pose.orientation.w = s.attitude.w();
      odom.pose.pose.orientation.x = s.attitude.x();
      odom.pose.pose.orientation.y = s.attitude.y();
      odom.pose.pose.orientation.z = s.attitude.z();
      // Odometry twist is expressed in child_frame_id, not in world.
      odom.twist.twist.linear  = vector(s.attitude.conjugate() * s.velocity);
      odom.twist.twist.angular = vector(s.angular_velocity);
      odom_pub_->publish(odom);
      sensor_msgs::msg::Imu imu;
      imu.header              = odom.header;
      imu.header.frame_id     = "base_link";
      imu.orientation         = odom.pose.pose.orientation;
      imu.angular_velocity    = vector(s.angular_velocity);
      imu.linear_acceleration = vector(f.specific_force);
      imu_pub_->publish(imu);
      geometry_msgs::msg::WrenchStamped wrench;
      wrench.header        = imu.header;
      wrench.wrench.force  = vector(f.aerodynamic.force);
      wrench.wrench.torque = vector(f.aerodynamic.moment);
      wrench_pub_->publish(wrench);
      geometry_msgs::msg::PoseStamped pose;
      pose.header         = odom.header;
      pose.pose           = odom.pose.pose;
      actual_path_.header = odom.header;
      actual_path_.poses.push_back(pose);
      if (actual_path_.poses.size() > 4096)
        actual_path_.poses.erase(actual_path_.poses.begin());
      actual_pub_->publish(actual_path_);
    }
  };

}  // namespace simple_sim

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    rclcpp::spin(std::make_shared<simple_sim::SimNode>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("simple_sim"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
