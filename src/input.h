#ifndef __INPUT_H
#define __INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

class RC_Data_t
{
public:
    double mode;
    double gear;
    double last_mode;
    double last_gear;
    bool valid;
    bool kill_switch_engaged;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    double ch[4];

    mavros_msgs::msg::RCIn msg;
    rclcpp::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_hover_mode;
    bool enter_hover_mode;

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double KILL_SWITCH_THRESHOLD_VALUE = 0.75;
    static constexpr double DEAD_ZONE = 0.25;
    static constexpr int MODE_CHANNEL = 4;
    static constexpr int GEAR_CHANNEL = 5;
    static constexpr int KILL_SWITCH_CHANNEL = 6;
    static constexpr int MIN_CHANNELS = 7;
    static constexpr int RC_PWM_MIN = 800;
    static constexpr int RC_PWM_MAX = 2200;

    RC_Data_t();
    bool check_validity();
    bool check_centered();
    void feed(mavros_msgs::msg::RCIn::ConstPtr pMsg);
    bool is_received(const rclcpp::Time &now_time);
};

class Odom_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::msg::Odometry msg;
    rclcpp::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t();
    void feed(nav_msgs::msg::Odometry::ConstPtr pMsg);
};

class Imu_Data_t
{
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::msg::Imu msg;
    rclcpp::Time rcv_stamp;

    Imu_Data_t();
    void feed(sensor_msgs::msg::Imu::ConstPtr pMsg);
};

class State_Data_t
{
public:
    mavros_msgs::msg::State current_state;
    mavros_msgs::msg::State state_before_offboard;

    State_Data_t();
    void feed(mavros_msgs::msg::State::ConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
    mavros_msgs::msg::ExtendedState current_extended_state;

    ExtendedState_Data_t();
    void feed(mavros_msgs::msg::ExtendedState::ConstPtr pMsg);
};

class Command_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;
    uint8_t yaw_control_mode;

    quadrotor_msgs::msg::PositionCommand msg;
    rclcpp::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::msg::PositionCommand::ConstPtr pMsg);
};

class Battery_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::msg::BatteryState msg;
    rclcpp::Time rcv_stamp;

    Battery_Data_t();
    void feed(sensor_msgs::msg::BatteryState::ConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its definition

    quadrotor_msgs::msg::TakeoffLand msg;
    rclcpp::Time rcv_stamp;

    Takeoff_Land_Data_t();
    void feed(quadrotor_msgs::msg::TakeoffLand::ConstPtr pMsg);
};

#endif
