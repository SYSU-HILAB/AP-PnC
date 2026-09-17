#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <stdexcept>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

class Parameter_t
{
public:
    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kvi0, Kvi1, Kvi2;
        double Kvd0, Kvd1, Kvd2;
        double KAngR, KAngP, KAngY;
    };

    struct RotorDrag
    {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MsgTimeout
    {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping
    {
        bool print_val;
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        double hover_percentage;
        bool noisy_imu;
    };

    struct RCReverse
    {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand
    {
        bool enable;
        bool no_RC;
        double height;
        double speed;
    };

    struct PlannerReset
    {
        bool enable;
        std::string service;
    };

    Gain gain;
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;
    PlannerReset planner_reset;

    int pose_solver;
    bool enable_body_rate_ctrl;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double max_yaw_target_mode_rate_max;
    double yaw_target_kp;
    double yaw_target_deadband;
    double rc_cmd_vel_bias_scale;
    double low_voltage;

    Parameter_t();
    void config_from_ros_handle(const rclcpp::Node::SharedPtr &node);

private:
    template <typename TName, typename TVal>
    void read_essential_param(const rclcpp::Node::SharedPtr &node, const TName &name, TVal &val)
    {
        // Declare statically typed but with no default: a missing parameter
        // (or a wrong YAML type) fails loud instead of silently using a
        // fallback value.
        const std::string param_name(name);
        if constexpr (std::is_same_v<TVal, bool>)
        {
            node->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if constexpr (std::is_same_v<TVal, double>)
        {
            node->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if constexpr (std::is_same_v<TVal, int>)
        {
            node->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else
        {
            static_assert(!std::is_same_v<TVal, TVal>, "read_essential_param: unsupported parameter type");
        }

        if (!node->get_parameter(param_name, val))
        {
            RCLCPP_ERROR(node->get_logger(), "Read param: %s failed.", param_name.c_str());
            throw std::runtime_error("Parameter read failed: " + param_name);
        }
    }

    template <typename TName, typename TVal>
    void read_optional_param(const rclcpp::Node::SharedPtr &node, const TName &name, TVal &val)
    {
        const std::string param_name(name);
        node->declare_parameter(param_name, val);
        node->get_parameter(param_name, val);
    }
};

#endif
