/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <deque>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;
    uint8_t yaw_control_mode{quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TRACK};

    Desired_State_t() {};

    Desired_State_t(Odom_Data_t &odom)
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0),
          yaw_control_mode(quadrotor_msgs::msg::PositionCommand::YAW_CONTROL_TRACK) {};
};

struct Controller_Output_t
{

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]

    // Collective mass normalized thrust
    double thrust;

    // Eigen::Vector3d des_v_real;
};

class Controller
{
public:
    struct SmithState_t
    {
        rclcpp::Time stamp;
        Eigen::Quaterniond q_model;
    };

    Parameter_t &param;

    Eigen::Vector3d Kp;
    Eigen::Vector3d Kv;
    Eigen::Vector3d Kvi;
    Eigen::Vector3d Kvd;
    Eigen::Vector3d KAng;
    Eigen::Vector3d int_e_v;
    Eigen::Vector3d Gravity;
    std::queue<std::pair<rclcpp::Time, double>> timed_thrust;
    rclcpp::Time last_ctrl_timestamp_{rclcpp::Time(0, 0, RCL_SYSTEM_TIME)};
    Eigen::Vector3d last_bodyrate_{Eigen::Vector3d(0, 0, 0)};
    bool smith_initialized_{false};
    Eigen::Quaterniond smith_q_model_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d smith_omega_model_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d smith_last_bodyrate_cmd_{Eigen::Vector3d::Zero()};
    rclcpp::Time smith_last_time_{rclcpp::Time(0, 0, RCL_SYSTEM_TIME)};
    std::deque<SmithState_t> smith_history_;

    quadrotor_msgs::msg::Px4ctrlDebug debug; // debug
    quadrotor_msgs::msg::Px4ctrlDebug smith_debug; // Smith predictor shadow debug

    // Thrust-accel mapping params
    double thr_scale_compensate;
    const double rho2 = 0.998; // do not change
    double thr2acc;
    double P;

    Controller(Parameter_t &);
    double computeYawTargetBodyRate(double target_yaw, double current_yaw) const;

    /* Algorithm0 from  Zhepei Wang*/
    quadrotor_msgs::msg::Px4ctrlDebug update_alg0(
        const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu,
        Controller_Output_t &u,
        double voltage);

    Eigen::Vector3d computeLimitedTotalAccFromThrustForce(
        const Eigen::Vector3d &thrustforce,
        const double &mass) const;

    bool flatnessWithDrag(const Eigen::Vector3d &vel,
                          const Eigen::Vector3d &acc,
                          const Eigen::Vector3d &jer,
                          const double &psi,
                          const double &dpsi,
                          double &thr,
                          Eigen::Vector4d &quat,
                          Eigen::Vector3d &omg,
                          const double &mass,
                          const double &grav,
                          const double &dh,
                          const double &dv,
                          const double &cp,
                          const double &veps) const;

    void minimumSingularityFlatWithDrag(const double mass,
                                        const double grav,
                                        const Eigen::Vector3d &vel,
                                        const Eigen::Vector3d &acc,
                                        const Eigen::Vector3d &jer,
                                        const double &yaw,
                                        const double &yawd,
                                        const Eigen::Quaterniond &att_est,
                                        Eigen::Quaterniond &att,
                                        Eigen::Vector3d &omg,
                                        double &thrust) const;

    /* Algorithm1 from  Zhepei Wang*/
    quadrotor_msgs::msg::Px4ctrlDebug update_alg1(
        const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu,
        Controller_Output_t &u,
        double voltage);

    void normalizeWithGrad(
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &xd,
        Eigen::Vector3d &xNor,
        Eigen::Vector3d &xNord) const;

    void computeFlatInput(
        const Eigen::Vector3d &thr_acc,
        const Eigen::Vector3d &jer,
        const double &yaw,
        const double &yawd,
        const Eigen::Quaterniond &att_est,
        Eigen::Quaterniond &att,
        Eigen::Vector3d &omg) const;

    /*Algorithm from the rotor-drag paper*/
    void update_alg2(
        const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu,
        Controller_Output_t &u,
        double voltage);

    void computeAeroCompensatedReferenceInputs(
        const Desired_State_t &des,
        const Odom_Data_t &odom, const Parameter_t &param,
        Controller_Output_t *u, Eigen::Vector3d *drag_acc) const;

    Eigen::Quaterniond computeDesiredAttitude(
        const Eigen::Vector3d &des_acc, const double reference_heading,
        const Eigen::Quaterniond &est_q) const;

    Eigen::Vector3d computeRobustBodyXAxis(
        const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
        const Eigen::Vector3d &y_C,
        const Eigen::Quaterniond &est_q) const;

    /* Shared functions*/
    Eigen::Vector3d computePIDErrorAcc(
        const Odom_Data_t &odom, const Desired_State_t &des,
        const Parameter_t &param);

    Eigen::Vector3d computeLimitedTotalAcc(
        const Eigen::Vector3d &PIDErrorAcc,
        const Eigen::Vector3d &ref_acc,
        const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

    Eigen::Vector3d computeLimitedAngularAcc(
        const Eigen::Vector3d candirate_bodyrate);

    Eigen::Quaterniond integrateBodyrate(
        const Eigen::Quaterniond &q,
        const Eigen::Vector3d &omega_body,
        double dt) const;

    void resetSmithPredictor(
        const Eigen::Quaterniond &actual_q,
        const rclcpp::Time &stamp);

    void updateSmithPredictorModel(
        const Eigen::Quaterniond &actual_q,
        const rclcpp::Time &stamp);

    bool getSmithModelAttitudeAt(
        const rclcpp::Time &stamp,
        Eigen::Quaterniond &q_model) const;

    Eigen::Quaterniond computeSmithPredictedAttitude(
        const Eigen::Quaterniond &actual_q,
        const rclcpp::Time &stamp);

    const quadrotor_msgs::msg::Px4ctrlDebug &getSmithDebug() const;

    double computeDesiredCollectiveThrustSignal(
        const Eigen::Quaterniond &est_q,
        const Eigen::Vector3d &est_v,
        const Eigen::Vector3d &des_acc,
        const Parameter_t &param,
        double voltage);

    double AccurateThrustAccMapping(
        const double des_acc_z,
        double voltage,
        const Parameter_t &param) const;

    Eigen::Vector3d computeFeedBackControlBodyrates(
        const Eigen::Quaterniond &des_q,
        const Eigen::Quaterniond &est_q,
        const Parameter_t &param);

    bool estimateThrustModel(
        const Eigen::Vector3d &est_a,
        const double voltage,
        const Eigen::Vector3d &est_v,
        const Parameter_t &param);

    bool almostZero(const double value) const;

    bool almostZeroThrust(const double thrust_value) const;

    void resetThrustMapping(void);

private:
    static constexpr double kMinNormalizedCollectiveAcc_ = 3;
    static constexpr double kAlmostZeroValueThreshold_ = 0.001;
    static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
    static constexpr double kMaxBodyratesFeedback_ = 4;
    static constexpr double kMaxAngularAcc_ = 60;
    static constexpr bool kSmithPredictorEnabled_ = false; // 我自己加的史密斯预测器，正常情况不需要使用
    static constexpr bool kSmithPredictorDebugOnly_ = true;
    static constexpr double kSmithDelay_ = 0.04;
    static constexpr double kSmithRateTimeConstant_ = 0.04;
    static constexpr double kSmithMaxCorrectionAngle_ = 0.35;
    static constexpr double kSmithResetTimeout_ = 0.10;
    static constexpr double kSmithBlend_ = 0.5;
    static constexpr double kSmithHistoryDuration_ = 1.0;
};

#endif
