#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand_t
{
    bool landed{true};
    rclcpp::Time toggle_takeoff_land_time;
    rclcpp::Time not_armed_since; // start of the continuous unarmed window during AUTO_TAKEOFF
    std::pair<bool, rclcpp::Time> delay_trigger{std::pair<bool, rclcpp::Time>(false, rclcpp::Time(0, 0, RCL_SYSTEM_TIME))};
    Eigen::Vector4d start_pose;

    static constexpr double MOTORS_SPEEDUP_TIME = 3.0;       // motors idle running for 3 seconds before takeoff
    static constexpr double DELAY_TRIGGER_TIME = 2.0;        // Time to be delayed when reach at target height
    static constexpr double NOT_ARMED_DEADZONE_TIME = 3.0;   // unarmed dead-zone before aborting AUTO_TAKEOFF
};

class PX4CtrlFSM
{
public:
    Parameter_t &param;

    RC_Data_t rc_data;
    State_Data_t state_data;
    ExtendedState_Data_t extended_state_data;
    Odom_Data_t odom_data;
    Imu_Data_t imu_data;
    Command_Data_t cmd_data;
    Battery_Data_t bat_data;
    Takeoff_Land_Data_t takeoff_land_data;

    Controller &controller;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_FCU_pub;
    rclcpp::Publisher<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr debug_pub;       // debug
    rclcpp::Publisher<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr smith_debug_pub; // Smith predictor shadow debug
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_srv;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_reset_srv;

    quadrotor_msgs::msg::Px4ctrlDebug debug_msg; // debug

    Eigen::Vector4d hover_pose;
    rclcpp::Time last_set_hover_pose_time;

    /* While px4ctrl is locked (MANUAL_CTRL), the drone is NOT under command
     * control. Keep the l3 planner FSM reset to WAIT_TARGET so it stops
     * streaming /setpoint_cmd (a stale planner trajectory trips the
     * cmd_is_received takeoff/hover guard). Throttled — a fresh reset is all
     * the planner needs, no need to spam the service. Disabled by default;
     * enable with planner_reset.enable for stacks that own that service. */
    rclcpp::Time last_planner_reset_time;
    static constexpr double PLANNER_RESET_INTERVAL_S = 0.2;

    enum State_t
    {
        MANUAL_CTRL = 1, // px4ctrl is deactivated. FCU is controlled by the remote controller only
        AUTO_HOVER,      // px4ctrl is activated, it will keep the drone hovering from odom measurements while waiting for commands from PositionCommand topic.
        CMD_CTRL,        // px4ctrl is activated, and controlling the drone.
        AUTO_TAKEOFF,
        AUTO_LAND
    };

    static const char *state_name(State_t s)
    {
        switch (s)
        {
        case MANUAL_CTRL:
            return "MANUAL_CTRL";
        case AUTO_HOVER:
            return "AUTO_HOVER";
        case CMD_CTRL:
            return "CMD_CTRL";
        case AUTO_TAKEOFF:
            return "AUTO_TAKEOFF";
        case AUTO_LAND:
            return "AUTO_LAND";
        default:
            return "UNKNOWN";
        }
    }

    // mmz change 因为ros2 中srv 好像必须使用node来判断，所有加了一个node成员
    rclcpp::Node::SharedPtr node;
    PX4CtrlFSM(Parameter_t &, Controller &, rclcpp::Node::SharedPtr &node);
    void process();
    bool rc_is_received(const rclcpp::Time &now_time);
    bool cmd_is_received(const rclcpp::Time &now_time);
    bool odom_is_received(const rclcpp::Time &now_time);
    bool imu_is_received(const rclcpp::Time &now_time);
    bool bat_is_received(const rclcpp::Time &now_time);
    bool recv_new_odom();
    State_t get_state() { return state; }
    bool get_landed() { return takeoff_land.landed; }

private:
    State_t state;      // Should only be changed in PX4CtrlFSM::process() function!
    State_t last_state; // Track previous state for debug message
    AutoTakeoffLand_t takeoff_land;

    // ---- control related ----
    Desired_State_t get_hover_des();
    Desired_State_t get_cmd_des();

    // ---- auto takeoff/land ----
    void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
    void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing
    void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
    Desired_State_t get_rotor_speed_up_des(const rclcpp::Time now);
    Desired_State_t get_takeoff_land_des(const double speed);

    // ---- tools ----
    void set_hov_with_odom();
    void set_hov_with_rc();
    void apply_rc_vel_bias(Desired_State_t &des);

    bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
    bool toggle_arm_disarm(bool arm);       // It will only try to toggle once, so not blocked.

    void publish_bodyrate_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
    void publish_attitude_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
    void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);
    void reset_planner(const rclcpp::Time &now_time);
};

#endif
