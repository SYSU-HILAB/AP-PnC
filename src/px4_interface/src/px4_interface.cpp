// #include
#include <stdint.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <interface/msg/controls.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/tracking_info.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/tracking_info.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_acc_setpoint.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

inline double inverse_thrust_model(double a, double b, double c)
{
    return (-b + std::sqrt(b * b + 4 * a * c)) / (2 * a);
}

// inverse to calcuate linear matrix equation
Eigen::Vector2d fit_a_b(double hover_estimate)
{
    Eigen::Matrix2d A;
    Eigen::Vector2d b_tilde;
    A.row(0) << hover_estimate * hover_estimate, hover_estimate;
    A.row(1) << 2 * hover_estimate, 1;
    b_tilde << 9.81, hover_estimate;
    return Eigen::Vector2d(
        9.81 / (hover_estimate * hover_estimate * hover_estimate), 0);
}

class Transcribe
{
    Eigen::Vector3d    p;
    Eigen::Vector3d    v;
    Eigen::Vector3d    r;
    Eigen::Quaterniond q;

    Eigen::Quaterniond q_flu_frd = Eigen::Quaterniond(0, 1, 0, 0);
    Eigen::Quaterniond q_enu_ned = Eigen::Quaterniond(0, 1, 1, 0);
    Eigen::Quaterniond q_transform;

   public:
    enum class E_World_Frame
    {
        NED,
        FRD
    };
    explicit Transcribe(const E_World_Frame world_frame)
    {
        q_flu_frd.normalize();
        if (world_frame == E_World_Frame::NED)
        {
            q_transform = Eigen::Quaterniond(0, 1, 1, 0);
        }
        else
        {
            q_transform = q_flu_frd;
        }
        q_transform.normalize();
    };
    template <typename T>
    void toFLU(const std::array<T, 3>& pos, const std::array<T, 3>& vel,
               const std::array<T, 3>& rate, const std::array<T, 4>& quat,
               interface::msg::State& state)
    {
        p = Eigen::Vector3d(pos[0], pos[1], pos[2]);
        v = Eigen::Vector3d(vel[0], vel[1], vel[2]);
        r = Eigen::Vector3d(rate[0], rate[1], rate[2]);
        q = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        p = q_transform * p;
        v = q_transform * v;
        r = q_flu_frd * r;
        q = q_transform * q * q_flu_frd;
        q.normalize();
        using interface::msg::State;
        std::copy(p.data(), p.data() + 3,
                  state.state.begin() + State::POS_START_INDEX);
        std::copy(v.data(), v.data() + 3,
                  state.state.begin() + State::VEL_START_INDEX);
        std::copy(r.data(), r.data() + 3,
                  state.state.begin() + State::RATE_START_INDEX);
        // q.coeffs
        // returns
        // x,y,z,w
        Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
        std::copy(q_vec.data(), q_vec.data() + 4,
                  state.state.begin() + State::QUAT_START_INDEX);
    }

    template <typename T>
    std::array<float, 3>& toFRD(const std::array<T, 3>& rate)
    {
        static std::array<float, 3> rate_frd;

        r = Eigen::Vector3d(rate[0], rate[1], rate[2]);

        r = q_flu_frd * r;

        std::copy(r.data(), r.data() + 3, rate_frd.data());
        // q.coeffs
        // returns
        // x,y,z,w
        return rate_frd;
    }
};

class PX4Interface : public rclcpp::Node
{
   public:
    PX4Interface() : Node("px4_interface")
    {
        this->declare_parameter("world_frame", "NED");
        this->declare_parameter("hover_estimate", 0.73);
        hover_estimate_ = this->get_parameter("hover_estimate").as_double();
        a_b_            = fit_a_b(hover_estimate_);
        // warn print ab
        RCLCPP_WARN(this->get_logger(), "a: %f, b: %f", a_b_(0), a_b_(1));
        std::string world_frame =
            this->get_parameter("world_frame").as_string();
        if (world_frame == "NED")
        {
            RCLCPP_WARN(this->get_logger(), "world frame: NED");
            trans_ =
                std::make_unique<Transcribe>(Transcribe::E_World_Frame::NED);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "world frame: FRD");
            trans_ =
                std::make_unique<Transcribe>(Transcribe::E_World_Frame::FRD);
        }

        auto reliable_qos =
            rclcpp::ParametersQoS().keep_last(1).transient_local();
        auto keep_last_qos = rclcpp::SensorDataQoS().keep_last(1);

        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);
        px4_tracking_info_publisher_ =
            this->create_publisher<px4_msgs::msg::TrackingInfo>(
                "/fmu/in/tracking_info", 10);
        state_publisher_ = this->create_publisher<interface::msg::State>(
            "/cur_state", reliable_qos);

        state_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/cur_state_odom", reliable_qos);
        vehicle_odometry_subscriber_ =
            this->create_subscription<VehicleOdometry>(
                "/fmu/out/vehicle_odometry", keep_last_qos,
                std::bind(&PX4Interface::vehicle_odometry_callback, this,
                          std::placeholders::_1));
        vehicle_thrust_acc_setpoint_publisher_ =
            this->create_publisher<VehicleThrustAccSetpoint>(
                "/fmu/in/vehicle_thrust_acc_setpoint", 10);
        controls_subscriber_ =
            this->create_subscription<interface::msg::Controls>(
                "/mpc_controls", rclcpp::SensorDataQoS(),
                std::bind(&PX4Interface::controls_callback, this,
                          std::placeholders::_1));
        tracking_info_subscriber_ =
            this->create_subscription<interface::msg::TrackingInfo>(
                "/tracking_info", keep_last_qos,
                std::bind(&PX4Interface::tracking_info_cb, this,
                          std::placeholders::_1));
        auto heartbeat_callback = [this]() -> void
        { publish_offboard_control_mode(); };

        this->declare_parameter("control_interval", 0.02);
        control_interval_ = this->get_parameter("control_interval").as_double();
        heartbeat_timer_  = this->create_wall_timer(100ms, heartbeat_callback);
    }

   public:
    std::unique_ptr<Transcribe> trans_;
    double                      control_interval_;
    bool                        is_init_ = false;
    interface::msg::Controls    controls_;
    interface::msg::State       state_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr state_pub_timer_;

    double          hover_estimate_;
    Eigen::Vector2d a_b_;
    // sub
    // VehicleOdometry
    // Controls;
    // pub
    // vehicle_thrust_setpoint
    // State
    rclcpp::Publisher<VehicleThrustAccSetpoint>::SharedPtr
        vehicle_thrust_acc_setpoint_publisher_;
    rclcpp::Publisher<interface::msg::State>::SharedPtr state_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrackingInfo>::SharedPtr
        px4_tracking_info_publisher_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr
        offboard_control_mode_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_publisher_;

    rclcpp::Subscription<interface::msg::Controls>::SharedPtr
        controls_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        vehicle_odometry_subscriber_;

    // rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
    //     vehicle_odometry_subscriber_;
    // // sub
    // std
    // bool
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscriber_;
    rclcpp::Subscription<interface::msg::TrackingInfo>::SharedPtr
        tracking_info_subscriber_;
    // subscription
    // callbacks
    // above
    void controls_callback(const interface::msg::Controls::SharedPtr msg);
    void vehicle_odometry_callback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    void trigger_cb();
    void publish_offboard_control_mode();
    void publish_state();

    void tracking_info_cb(const interface::msg::TrackingInfo::SharedPtr msg);
};

void PX4Interface::tracking_info_cb(
    const interface::msg::TrackingInfo::SharedPtr msg)
{
    static px4_msgs::msg::TrackingInfo tracking_info;
    tracking_info.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    tracking_info.actual_position = msg->actual_position;
    tracking_info.actual_velocity = msg->actual_velocity;
    tracking_info.ref_position    = msg->ref_position;
    tracking_info.ref_velocity    = msg->ref_velocity;
    tracking_info.ref_yb          = msg->ref_yb;
    tracking_info.actual_yb       = msg->actual_yb;
    tracking_info.alpha           = msg->alpha;
    tracking_info.cz              = msg->cz;
    tracking_info.ref_vxy         = msg->ref_vxy;
    tracking_info.actual_vxy      = msg->actual_vxy;
    px4_tracking_info_publisher_->publish(tracking_info);
}

void PX4Interface::controls_callback(
    const interface::msg::Controls::SharedPtr msg)
{
    static px4_msgs::msg::VehicleThrustAccSetpoint px4_control;
    std::cout << "Controls received: " << msg->timestamp << std::endl;
    px4_control.timestamp     = this->get_clock()->now().nanoseconds() / 1000;
    px4_control.rates_sp      = trans_->toFRD(msg->rates_sp);
    px4_control.thrust_acc_sp = msg->thrust_acc_sp;
    if (msg->thrust_acc_sp > 9.81)
    {
        px4_control.model_ff = msg->thrust_acc_sp / 9.81 * hover_estimate_;
    }
    else
    {
        // cubic inverse
        px4_control.model_ff =
            std::pow(msg->thrust_acc_sp / a_b_(0), 1.0 / 3.0);
    }

    printf("model_ff: %f\n", px4_control.model_ff);
    vehicle_thrust_acc_setpoint_publisher_->publish(px4_control);
    // publish_px4_control();
}

/**
 * @brief
 * Callback
 * on
 * vehicle_odometry
 */
void PX4Interface::vehicle_odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    if (!is_init_)
    {
        is_init_         = true;
        state_pub_timer_ = this->create_wall_timer(
            20ms, std::bind(&PX4Interface::publish_state, this));
    }
    this->trans_->toFLU(msg->position, msg->velocity, msg->angular_velocity,
                        msg->q, state_);
    std::copy(state_.state.begin(), state_.state.end(),
              state_.mpc_state.begin());
}

void PX4Interface::publish_state()
{
    state_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    state_publisher_->publish(state_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = this->get_clock()->now();
    odom.header.frame_id = "world";
    using interface::msg::State;
    odom.pose.pose.position.x    = state_.state[State::POS_START_INDEX];
    odom.pose.pose.position.y    = state_.state[State::POS_START_INDEX + 1];
    odom.pose.pose.position.z    = state_.state[State::POS_START_INDEX + 2];
    odom.pose.pose.orientation.w = state_.state[State::QUAT_START_INDEX];
    odom.pose.pose.orientation.x = state_.state[State::QUAT_START_INDEX + 1];
    odom.pose.pose.orientation.y = state_.state[State::QUAT_START_INDEX + 2];
    odom.pose.pose.orientation.z = state_.state[State::QUAT_START_INDEX + 3];
    state_odom_publisher_->publish(odom);
}

void PX4Interface::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position     = false;
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = true;
    msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    std::cout << "Starting PX4 Interface Node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4Interface>());

    rclcpp::shutdown();
    return 0;
}