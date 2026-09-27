#include <signal.h>

#include <std_srvs/srv/set_bool.hpp>

#include "PX4CtrlFSM.h"
#include "rclcpp/rclcpp.hpp"

void mySigintHandler(int sig)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[PX4Ctrl] exit...");
  rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("px4ctrl");

  signal(SIGINT, mySigintHandler);
  rclcpp::sleep_for(std::chrono::seconds(1));

  Parameters param;
  param.config_from_ros_handle(node);

  Controller controller(param);
  PX4CtrlFSM fsm(param, controller, node);

  auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [&fsm](const mavros_msgs::msg::State::SharedPtr msg)
      { fsm.state_data.feed(msg); });

  auto extended_state_sub =
      node->create_subscription<mavros_msgs::msg::ExtendedState>(
          "/mavros/extended_state", 10,
          [&fsm](const mavros_msgs::msg::ExtendedState::SharedPtr msg)
          { fsm.extended_state_data.feed(msg); });

  // SensorDataQoS (best-effort, keep-last-1): the odom/cmd/imu/bat streams are
  // high-rate control inputs; queuing them only adds latency.
  auto qos      = rclcpp::SensorDataQoS();
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos, [&fsm, &param](const nav_msgs::msg::Odometry::SharedPtr msg)
      { fsm.odom_data.feed(msg, param.odom_twist_is_body); });

  auto cmd_sub =
      node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
          "cmd", qos,
          [&fsm](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
          { fsm.cmd_data.feed(msg); });

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data",
      qos,  // Note: do NOT change it to /mavros/imu/data_raw !!!
      [&fsm](const sensor_msgs::msg::Imu::SharedPtr msg)
      { fsm.imu_data.feed(msg); });

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
  if (!param.takeoff_land.no_RC)  // mavros will still publish wrong rc messages
                                  // although no RC is connected
  {
    rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
        "/mavros/rc/in", 10, [&fsm](const mavros_msgs::msg::RCIn::SharedPtr msg)
        { fsm.rc_data.feed(msg); });
  }

  auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", qos,
      [&fsm](const sensor_msgs::msg::BatteryState::SharedPtr msg)
      { fsm.bat_data.feed(msg); });

  auto takeoffland_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  auto takeoff_land_sub =
      node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
          "takeoff_land", takeoffland_qos,
          [&fsm](const quadrotor_msgs::msg::TakeoffLand::SharedPtr msg)
          { fsm.takeoff_land_data.feed(msg); });
  // External rates + thrust setpoint forwarded verbatim in PASS_THROUGH
  // (external stream; the internal nmpc controller mode uses /nmpc/control).
  auto pass_through_sub =
      node->create_subscription<mavros_msgs::msg::AttitudeTarget>(
          "/px4ctrl/setpoint_raw/attitude", qos,
          [&fsm](const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
          { fsm.pass_through_data.feed(msg); });

  // NMPC controls (rates_sp FLU + specific_force_sp) for ctrl_mode=1.
  auto controls_sub = node->create_subscription<interface::msg::Control>(
      "/nmpc/control", qos,
      [&fsm](const interface::msg::Control::SharedPtr msg)
      { fsm.controls_data.feed(msg); });

  // State adapter input (mavros ENU/FLU odometry): origin lock + world
  // twist conversion feeding the /px4ctrl/state contract.
  auto state_adapter_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", qos,
      [&fsm](const nav_msgs::msg::Odometry::SharedPtr msg)
      { fsm.state_adapter_odom_callback(msg); });

  // Synchronous PASS_THROUGH toggle for the upper layer (stack trigger).
  auto toggle_pass_through_srv = node->create_service<std_srvs::srv::SetBool>(
      "/px4ctrl/toggle_pass_through",
      [&fsm](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr      res)
      {
        res->success = fsm.toggle_pass_through(req->data);
        res->message = res->success ? "pass-through toggled"
                                    : "rejected, see px4ctrl logs";
      });

  fsm.ctrl_FCU_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 10);
  auto trigger_qos = rclcpp::QoS(1)
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::TransientLocal);
  fsm.traj_start_trigger_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger", trigger_qos);

  fsm.debug_pub = node->create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>(
      "/debugPx4ctrl", 10);  // debug

  // State adapter outputs + throttle estimator telemetry.
  auto latch_qos = rclcpp::QoS(1).transient_local();
  fsm.state_pub =
      node->create_publisher<interface::msg::State>("/px4ctrl/state", latch_qos);
  fsm.state_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
      "/px4ctrl/state_odom", latch_qos);
  fsm.odom_world_pub = node->create_publisher<nav_msgs::msg::Odometry>(
      "/px4ctrl/odom_world", qos);
  fsm.throttle_status_pub =
      node->create_publisher<interface::msg::ThrottleModelStatus>(
          "/px4ctrl/throttle_model_status", qos);

  // /px4ctrl/state publication at 50 Hz for nmpc + planner.
  auto state_timer = node->create_wall_timer(
      std::chrono::milliseconds(20), [&fsm]() { fsm.publish_state_adapter(); });

  fsm.set_FCU_mode_srv =
      node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  fsm.arming_client_srv =
      node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  if (param.planner_reset.enable)
  {
    fsm.planner_reset_srv = node->create_client<std_srvs::srv::Trigger>(
        param.planner_reset.service);
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  if (param.takeoff_land.no_RC)
  {
    RCLCPP_WARN(node->get_logger(),
                "[PX4CTRL] Remote controller disabled, be careful!");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "[PX4CTRL] Waiting for RC");
    while (rclcpp::ok())
    {
      rclcpp::spin_some(node);
      if (fsm.rc_is_received(rclcpp::Clock().now()))
      {
        RCLCPP_INFO(node->get_logger(), "[PX4CTRL] RC received.");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  int trials = 0;
  while (rclcpp::ok() && !fsm.state_data.current_state.connected)
  {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (trials++ > 5)
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to connect to PX4!!!");
    }
  }

  rclcpp::Rate rate(param.ctrl_freq_max * 2);
  while (rclcpp::ok())
  {
    rate.sleep();

    // Spin at 2x the control rate so the high-rate subscribers (odom, imu)
    // keep up; the controller itself runs at ctrl_freq_max.
    rclcpp::spin_some(node);

    static int counter = 0;
    if (counter % 2 == 0)
    {
      fsm.process();  // We DO NOT rely on feedback as trigger, since there is
                      // no significant performance difference through our test.
    }
    ++counter;
  }

  return 0;
}
