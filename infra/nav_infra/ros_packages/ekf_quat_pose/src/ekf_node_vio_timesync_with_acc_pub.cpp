#include "ekf.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unsupported/Eigen/MatrixFunctions>
// #include <geometry_msgs/Accel.h>
#include "conversion.h"

using namespace std;
using namespace Eigen;
// 20200531: time synchronization
// 20200105: ekf_node_vio.cpp and ekf_node_mocap.cpp merge into one (ekf_node_vio.cpp) and the differences between them is the odom format
// X_state: p q v gb ab   with time stamp aligned between imu and img
/*
    EKF model
    prediction:
    xt~ = xt-1 + dt*f(xt-1, ut, 0)
    sigmat~ = Ft*sigmat-1*Ft' + Vt*Qt*Vt'
    Update:
    Kt = sigmat~*Ct'*(Ct*sigmat~*Ct' + Wt*Rt*Wt')^-1
    xt = xt~ + Kt*(zt - g(xt~,0))
    sigmat = sigmat~ - Kt*Ct*sigmat~
*/
/*
   -pi ~ pi crossing problem:
   1. the model prpagation: X_state should be limited to [-pi,pi] after predicting and updating
   2. inovation crossing: (measurement - g(X_state)) should also be limited to [-pi,pi] when getting the inovation.
   z_measurement is normally in [-pi~pi]
*/

// imu frame is imu body frame

// odom: pose px,py pz orientation qw qx qy qz
// imu: acc: x y z gyro: wx wy wz

#define TimeSync 1 // time synchronize or not
#define RePub 0    // re publish the odom when repropagation

#define POS_DIFF_THRESHOLD (0.3f)

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, ahead_odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cam_odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotate_odom_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr acc_filtered_pub;
rclcpp::Time imu_back_time(0, 0, RCL_ROS_TIME), imu_front_time(0, 0, RCL_ROS_TIME);
// mmz TODO：下面两行会导致段错误，目前先强制id为 1
// const char* env_p = std::getenv("DRONE_ID");
// int drone_id = atoi(env_p);
int drone_id = 1;

// state
geometry_msgs::msg::Pose pose;
Vector3d position, orientation, velocity;

// Now set up the relevant matrices
// states X [p q pdot]  [px,py,pz, wx,wy,wz, vx,vy,vz]
size_t stateSize;            // x = [p q pdot bg ba]
size_t errorstateSize;       // x = [p q pdot bg ba]
size_t stateSize_pqv;        // x = [p q pdot]
size_t measurementSize;      // z = [p q]
size_t inputSize;            // u = [w a]
VectorXd X_state(stateSize); // x (in most literature)
VectorXd u_input;
VectorXd Z_measurement;              // z
MatrixXd StateCovariance;            // sigma
MatrixXd Kt_kalmanGain;              // Kt
VectorXd X_state_correct(stateSize); // x (in most literature)
MatrixXd StateCovariance_correct;    // sigma
// MatrixXd Ct_stateToMeasurement;                  // Ct
//  VectorXd innovation;                         // z - Hx

MatrixXd Qt;
MatrixXd Rt;
Vector3d u_gyro;
Vector3d u_acc;
Vector3d gravity(0., 0., -9.8); // need to estimate the bias 9.8099
Vector3d bg_0(0., 0., 0.);       // need to estimate the bias
Vector3d ba_0(0., 0., 0.);       // need to estimate the bias  0.1
Vector3d ng(0., 0., 0.);
Vector3d na(0., 0., 0.);
Vector3d nbg(0., 0., 0.);
Vector3d nba(0., 0., 0.);
Matrix3d rotation_imu;

// Vector3d q_last;
//! changed by wz
Quaterniond q_last;
Vector3d bg_last;
Vector3d ba_last;

// Qt imu covariance matrix  smaller believe system(imu) more
double imu_trans_x = 0.0;
double imu_trans_y = 0.0;
double imu_trans_z = 0.0;
double gyro_cov = 0.01;
double acc_cov = 0.01;
// Rt visual odomtry covariance smaller believe measurement more
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;
double scale_g;
double dt = 0.005; // second
double t_last, t_now;
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool test_odomtag_call = false;
bool odomtag_call = false;

//imu outlier reject (by dzm) 
double acc_rej_threshold;   
double gyro_rej_threshold;

double time_now, time_last;
double time_odom_tag_now;
// double diff_time;

double cutoff_freq = 20;
string world_frame_id = "world";

//zyh added
double offset_px, offset_py, offset_pz;
Eigen::Vector3d first_odom_get_;

// world frame points velocity
deque<pair<VectorXd, sensor_msgs::msg::Imu>> sys_seq;
deque<MatrixXd> cov_seq;
double dt_0_rp; // the dt for the first frame in repropagation
void seq_keep(const sensor_msgs::msg::Imu::SharedPtr &imu_msg)
{
#define seqsize 100
    if (sys_seq.size() < seqsize)
    {
        sys_seq.push_back(make_pair(X_state, *imu_msg)); // X_state before propagation and imu at that time
        cov_seq.push_back(StateCovariance);
    }
    else
    {
        sys_seq.pop_front();
        sys_seq.push_back(make_pair(X_state, *imu_msg));
        cov_seq.pop_front();
        cov_seq.push_back(StateCovariance);
    }
    imu_front_time = rclcpp::Time(sys_seq.front().second.header.stamp);
    imu_back_time = rclcpp::Time(sys_seq.back().second.header.stamp);
    // ensure that the later frame time > the former one
}
// choose the coordinate frame imu for the measurement
// 寻找odom数据对应的imu数据
bool search_proper_frame(double odom_time)
{
    if (sys_seq.size() == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ekf_node"), "sys_seq.size() == 0. if appear this error, should check the code");
        return false;
    }
    if (sys_seq.size() == 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ekf_node"), "sys_seq.size() == 1. if appear this error, should check the code");
        return false;
    }

    size_t rightframe = sys_seq.size() - 1;
    bool find_proper_frame = false;
    for (size_t i = 1; i < sys_seq.size(); i++) // TODO: it better to search from the middle instead in the front
    {
        double time_before = odom_time - rclcpp::Time(sys_seq[i - 1].second.header.stamp).seconds();
        double time_after = odom_time - rclcpp::Time(sys_seq[i].second.header.stamp).seconds();
        if ((time_before >= 0) && (time_after < 0))
        {
            if (abs(time_before) > abs(time_after))
            {
                rightframe = i;
            }
            else
            {
                rightframe = i - 1;
            }

            if (rightframe != 0)
            {
                dt_0_rp = rclcpp::Time(sys_seq[rightframe].second.header.stamp).seconds() - rclcpp::Time(sys_seq[rightframe - 1].second.header.stamp).seconds();
            }
            else
            { // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
                dt_0_rp = rclcpp::Time(sys_seq[rightframe + 1].second.header.stamp).seconds() - rclcpp::Time(sys_seq[rightframe].second.header.stamp).seconds();
            }

            find_proper_frame = true;
            break;
        }
    }
    if (!find_proper_frame)
    {
        if ((odom_time - rclcpp::Time(sys_seq[0].second.header.stamp).seconds()) <= 0) // if odom time before the first frame, set first frame
        {
            rightframe = 0;
            // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
            dt_0_rp = rclcpp::Time(sys_seq[rightframe + 1].second.header.stamp).seconds() - rclcpp::Time(sys_seq[rightframe].second.header.stamp).seconds();
        }
        if ((odom_time - rclcpp::Time(sys_seq[sys_seq.size() - 1].second.header.stamp).seconds()) >= 0) // if odom time after the last frame, set last frame
        {
            rightframe = sys_seq.size() - 1;
            dt_0_rp = rclcpp::Time(sys_seq[rightframe].second.header.stamp).seconds() - rclcpp::Time(sys_seq[rightframe - 1].second.header.stamp).seconds();
        }
        // no process, set the latest one
    }

    // set the right frame as the first frame in the queue
    for (size_t i = 0; i < rightframe; i++)
    {
        sys_seq.pop_front();
        cov_seq.pop_front();
    }

    if (find_proper_frame)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void re_propagate()
{
    for (size_t i = 1; i < sys_seq.size(); i++)
    {
        // re-prediction for the rightframe
        dt = rclcpp::Time(sys_seq[i].second.header.stamp).seconds() - rclcpp::Time(sys_seq[i - 1].second.header.stamp).seconds();

        u_gyro(0) = sys_seq[i].second.angular_velocity.x;
        u_gyro(1) = sys_seq[i].second.angular_velocity.y;
        u_gyro(2) = sys_seq[i].second.angular_velocity.z;
        u_acc(0) = sys_seq[i].second.linear_acceleration.x;
        u_acc(1) = sys_seq[i].second.linear_acceleration.y;
        u_acc(2) = sys_seq[i].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        // q_last = sys_seq[i].first.segment<3>(3);   // last X2
        //! changed by wz
        // q_last = sys_seq[i].first.segment<4>(3); // last X2
        q_last.w() = sys_seq[i].first(3);
        q_last.x() = sys_seq[i].first(4);
        q_last.y() = sys_seq[i].first(5);
        q_last.z() = sys_seq[i].first(6);

        // bg_last = sys_seq[i].first.segment<3>(9);  // last X4
        // ba_last = sys_seq[i].first.segment<3>(12); // last X5
        //! changed by wz
        bg_last = sys_seq[i].first.segment<3>(10); // last X4
        ba_last = sys_seq[i].first.segment<3>(13); // last X5

        // Ft = MatrixXd::Identity(stateSize, stateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
        //! changed by wz
        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        // X_state += dt * F_model(u_gyro, u_acc);
        //! changed by wz
        Vt = dt * diff_f_diff_n(q_last);

        // if (X_state(3) > PI)
        //     X_state(3) -= 2 * PI;
        // if (X_state(3) < -PI)
        //     X_state(3) += 2 * PI;
        // if (X_state(4) > PI)
        //     X_state(4) -= 2 * PI;
        // if (X_state(4) < -PI)
        //     X_state(4) += 2 * PI;
        // if (X_state(5) > PI)
        //     X_state(5) -= 2 * PI;
        // if (X_state(5) < -PI)
        //     X_state(5) += 2 * PI;  //! changed by wz
        X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);

        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();

#if RePub
        system_pub(X_state, rclcpp::Time(sys_seq[i].second.header.stamp).seconds()); // choose to publish the repropagation or not
#endif
    }
}

void imu_callback(const sensor_msgs::msg::Imu::ConstPtr msg)
{
    // std::cout << "get_imu!!!" << std::endl;
    // wmywmy
    sensor_msgs::msg::Imu::SharedPtr new_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
    Eigen::Vector3d temp1, temp2;

    temp1[0] = new_msg->linear_acceleration.x;
    temp1[1] = new_msg->linear_acceleration.y;
    temp1[2] = new_msg->linear_acceleration.z;
    temp2 = rotation_imu * temp1;
    new_msg->linear_acceleration.x = scale_g * temp2[0];
    new_msg->linear_acceleration.y = scale_g * temp2[1];
    new_msg->linear_acceleration.z = scale_g * temp2[2];

    temp1[0] = new_msg->angular_velocity.x;
    temp1[1] = new_msg->angular_velocity.y;
    temp1[2] = new_msg->angular_velocity.z;
    temp2 = rotation_imu * temp1;
    new_msg->angular_velocity.x = temp2[0];
    new_msg->angular_velocity.y = temp2[1];
    new_msg->angular_velocity.z = temp2[2];
    // wmywmy

    // imu outlier reject (by dzm) 
    temp1[0] = new_msg->linear_acceleration.x;
    temp1[1] = new_msg->linear_acceleration.y;
    temp1[2] = new_msg->linear_acceleration.z;
    temp2[0] = new_msg->angular_velocity.x;
    temp2[1] = new_msg->angular_velocity.y;
    temp2[2] = new_msg->angular_velocity.z;
    if (temp1.norm() > acc_rej_threshold)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ekf_node"), "u_acc.norm() is %f  reject this imu measurement !!!", temp1.norm());
        return;
    }
    if (temp2.norm() > gyro_rej_threshold)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ekf_node"), "u_gyro.norm() is %f  reject this imu measurement !!!", temp2.norm());
        return;
    }

    // seq_keep(msg);
    // nav_msgs::Odometry odom_fusion;
    // your code for propagation
    if (!first_frame_tag_odom)
    { // get the initial pose and orientation in the first frame of measurement
        if (first_frame_imu)
        {
            first_frame_imu = false;
            time_now = new_msg->header.stamp.sec + new_msg->header.stamp.nanosec * 1e-9;
            time_last = time_now;
#if TimeSync
            seq_keep(new_msg); // keep before propagation
#endif

            system_pub(X_state, new_msg->header.stamp);
            // cout << "first frame imu" << endl;
        }
        else
        {
#if TimeSync
            seq_keep(new_msg); // keep before propagation
#endif
            // cout << "\033[1;32m[ INFO] [IMU] [TimeSync] [seq_keep] [OK] \033[0m" << endl;
            time_now = new_msg->header.stamp.sec + new_msg->header.stamp.nanosec * 1e-9;
            dt = time_now - time_last;

            if (odomtag_call)
            {
                odomtag_call = false;
                // diff_time = time_now - time_odom_tag_now;
                // if(diff_time<0)
                // {
                //     cout << "diff time: " << diff_time << endl;  //???!!! exist !!!???
                //     cout << "timeimu: " << time_now - 1.60889e9 << " time_odom: " << time_odom_tag_now - 1.60889e9 << endl;
                //     // cout << "diff time: " << diff_time << endl;  //about 30ms
                // }
            }
            MatrixXd Ft;
            MatrixXd Vt;

            u_gyro(0) = new_msg->angular_velocity.x;
            u_gyro(1) = new_msg->angular_velocity.y;
            u_gyro(2) = new_msg->angular_velocity.z;
            u_acc(0) = new_msg->linear_acceleration.x;
            u_acc(1) = new_msg->linear_acceleration.y;
            u_acc(2) = new_msg->linear_acceleration.z;

            // q_last = X_state.segment<3>(3);   // last X2
            // bg_last = X_state.segment<3>(9);  // last X4
            // ba_last = X_state.segment<3>(12); // last X5
            //! changed by wz
            q_last.w() = X_state(3);
            q_last.x() = X_state(4);
            q_last.y() = X_state(5);
            q_last.z() = X_state(6);
            // cout << "q_last" << endl
            //      << q_last << endl;

            bg_last = X_state.segment<3>(10); // last X4
            // cout << "bg_last" << endl
            //  << bg_last << endl;

            ba_last = X_state.segment<3>(13); // last X5

            // cout << "ba_last" << endl
            //  << ba_last << endl;
            //! changed by wz
            // cout << "dt:" << dt << endl;
            Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
            
            // cout << "Ft" << endl
            //      << Ft << endl;
            
            Vt = dt * diff_f_diff_n(q_last);
            // cout << "Vt" << endl
            //      << Vt << endl;

            // acc_f_pub(u_acc, msg->header.stamp);
            // X_state += dt * F_model(u_gyro, u_acc);
            //! changed by wz
            // cout << "X_state_old" << endl
            //      << X_state << endl;
            X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);
                        // cout << "X_state" << endl
            //      << X_state << endl;

            // if (X_state(3) > PI)  //! changed by wz
            //     X_state(3) -= 2 * PI;
            // if (X_state(3) < -PI)
            //     X_state(3) += 2 * PI;
            // if (X_state(4) > PI)
            //     X_state(4) -= 2 * PI;
            // if (X_state(4) < -PI)
            //     X_state(4) += 2 * PI;
            // if (X_state(5) > PI)
            //     X_state(5) -= 2 * PI;
            // if (X_state(5) < -PI)
            //     X_state(5) += 2 * PI;
            // cout << "!!!!!!!!!!!!!!!!!!!!position:" << X_state(0) << " " << X_state(1) << " " << X_state(2) << endl;

            StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();

            time_last = time_now;

            // Eigen::VectorXd X_state_ahead = X_state + 0.01 * F_model(u_gyro, u_acc);
            //! changed by wz
            Eigen::VectorXd X_state_ahead = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, 0.01);

            // if(test_odomtag_call) //no frequency boost
            // {
            //     test_odomtag_call = false;
            //     system_pub(msg->header.stamp);
            // }
            system_pub(X_state, new_msg->header.stamp);
            ahead_system_pub(X_state_ahead, new_msg->header.stamp);

            // cout << "[IMU] [TimeSync] [OK]" << endl;

            // system_pub(X_state, ros::Time::now());
            // ahead_system_pub(X_state_ahead, ros::Time::now());
        }
    }
}

// Rotation from the camera frame to the IMU frame
Matrix3d Rc_i;
Vector3d tc_i; //  cam in imu frame
int cnt = 0;
Vector3d INNOVATION_;
Matrix3d Rr_i;
Vector3d tr_i; //  rigid body in imu frame
Matrix3d Rl_i;
Vector3d tl_i; //  lidar in imu frame
// msg is imu in world
// VectorXd get_pose_from_VIOodom(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     Matrix3d Rr_w; // rigid body in world
//     Vector3d tr_w;
//     Matrix3d Ri_w;
//     Vector3d ti_w;
//     Vector3d p_temp;
//     p_temp(0) = msg->pose.pose.position.x;
//     p_temp(1) = msg->pose.pose.position.y;
//     p_temp(2) = msg->pose.pose.position.z;
//     // quaternion2euler:  ZYX  roll pitch yaw
//     Quaterniond q;
//     q.w() = msg->pose.pose.orientation.w;
//     q.x() = msg->pose.pose.orientation.x;
//     q.y() = msg->pose.pose.orientation.y;
//     q.z() = msg->pose.pose.orientation.z;

//     // Euler transform
//     //  Ri_w = q.toRotationMatrix();
//     //  ti_w = p_temp;
//     Rr_w = q.toRotationMatrix();
//     tr_w = p_temp;
//     Ri_w = Rr_w * Rr_i.inverse();
//     ti_w = tr_w - Ri_w * tr_i;
//     Vector3d euler = mat2euler(Ri_w);

//     VectorXd pose = VectorXd::Random(6);
//     pose.segment<3>(0) = ti_w;
//     pose.segment<3>(3) = euler;

//     return pose;
// }

//! changed by wz
// VectorXd get_pose_from_VIOodom(const nav_msgs::msg::Odometry::ConstPtr &msg)
// {
//     // cout << "get_pose_from_VIOodom" << endl;
//     Matrix3d Rr_w; // rigid body in world
//     Vector3d tr_w;
//     Matrix3d Ri_w;
//     Vector3d ti_w;
//     Vector3d p_temp;
//     p_temp(0) = msg->pose.pose.position.x;
//     p_temp(1) = msg->pose.pose.position.y;
//     p_temp(2) = msg->pose.pose.position.z;
//     // quaternion2euler:  ZYX  roll pitch yaw
//     Quaterniond q;
//     q.w() = msg->pose.pose.orientation.w;
//     q.x() = msg->pose.pose.orientation.x;
//     q.y() = msg->pose.pose.orientation.y;
//     q.z() = msg->pose.pose.orientation.z;

//     // // mmz shit: rotate 90 #####################################
//     // // 保存原始坐标
//     // double x_orig = p_temp(0);
//     // double y_orig = p_temp(1);
//     // double z_orig = p_temp(2);

//     // // 旋转位置（顺时针90度）
//     // p_temp(0) = -y_orig;     // x' = -y
//     // p_temp(1) = x_orig;    // y' = x
//     // p_temp(2) = z_orig;     // z 不变

//     // // 构造旋转四元数（绕Z轴顺时针90度 = -π/2）
//     // Eigen::AngleAxisd rotation_vector(-M_PI/2, Eigen::Vector3d::UnitZ());
//     // Eigen::Quaterniond q_rot(rotation_vector);  // 显式转换为四元数

//     // // 应用旋转（全局坐标系）
//     // q = q_rot * q;        // 四元数乘法
//     // q.normalize();         // 归一化
//     // // ################################################################

//     // Euler transform
//     //  Ri_w = q.toRotationMatrix();
//     //  ti_w = p_temp;
//     Rr_w = q.toRotationMatrix();
//     tr_w = p_temp;
//     Ri_w = Rr_w * Rr_i.inverse();
//     ti_w = tr_w - Ri_w * tr_i;
//     // Vector3d euler = mat2euler(Ri_w);
//     // std::cout << "odom q: " << q << std::endl;
//     // std::cout << "odom euler" << euler << std::endl;
//     //! changed by wz
//     Quaterniond q_wi = Quaterniond(Ri_w);

//     // VectorXd pose = VectorXd::Random(6);
//     //! changed by wz
//     VectorXd pose = VectorXd::Random(7);
//     pose.segment<3>(0) = ti_w;
//     // pose.segment<3>(3) = euler;
//     //! changed by wz
//     pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());

//     // cout << "pose: " << pose << endl;

//     return pose;
// }

// changed by mmz 1.0
// VectorXd get_pose_from_VIOodom(const nav_msgs::msg::Odometry::ConstPtr &msg)
// {
//     // 1. 定义雷达安装参数
//     const double yaw_offset_deg = 15.0; 
//     const double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;
    
//     // 安装变换：从雷达坐标系到机体坐标系 (绕Y轴旋转-15度)
//     Eigen::AngleAxisd rotation_vector(-yaw_offset_rad, Eigen::Vector3d::UnitY());
//     Eigen::Quaterniond q_radar_to_body(rotation_vector);
    
//     // 静态变量记录初始变换（关键修改）
//     static bool is_initialized = false;
//     static Eigen::Quaterniond q_w_body_to_w_radar;  // W_body到W_radar的旋转
//     static Eigen::Vector3d t_w_body_in_w_radar;    // W_body原点在W_radar中的位置
    
//     // 2. 获取原始雷达数据
//     Vector3d p_radar_w;
//     p_radar_w(0) = msg->pose.pose.position.x;
//     p_radar_w(1) = msg->pose.pose.position.y;
//     p_radar_w(2) = msg->pose.pose.position.z;
    
//     Quaterniond q_radar_w;
//     q_radar_w.w() = msg->pose.pose.orientation.w;
//     q_radar_w.x() = msg->pose.pose.orientation.x;
//     q_radar_w.y() = msg->pose.pose.orientation.y;
//     q_radar_w.z() = msg->pose.pose.orientation.z;

//     // 3. 坐标变换：雷达坐标系->机体坐标系（仍在雷达世界系中）
//     Eigen::Quaterniond q_body_in_w_radar = q_radar_w * q_radar_to_body;
//     q_body_in_w_radar.normalize();
    
//     Eigen::Matrix3d R_body_in_w_radar = q_body_in_w_radar.toRotationMatrix();
//     // 假设安装位置偏移为零（可根据实际情况修改）
//     // Eigen::Vector3d t_body_in_radar(-0.03763, 0.0, -0.06986);
//     q_w_body_to_w_radar.normalize();
//     Eigen::Matrix3d R_b_in_l = q_w_body_to_w_radar.toRotationMatrix();
//     Eigen::Vector3d t_body_in_radar = R_b_in_l * Eigen::Vector3d(-0.03763, 0.0, -0.06986);
//     // std::cout << "t_body_in_radar: " << t_body_in_radar.transpose() << std::endl;

//     Eigen::Vector3d p_body_in_w_radar = p_radar_w + t_body_in_radar; 

//     // 4. 初始时刻建立坐标变换关系（关键修改）
//     if (!is_initialized) {
//         // 初始时刻机体在W_radar中的姿态就是安装角的逆
//         q_w_body_to_w_radar = q_body_in_w_radar;
        
//         // W_body的原点位置 = 此时机体在W_radar中的位置
//         t_w_body_in_w_radar = p_body_in_w_radar;
        
//         is_initialized = true;
        
//         // 调试输出
//         Eigen::Vector3d init_euler = q_w_body_to_w_radar.toRotationMatrix()
//                                 .eulerAngles(2,1,0).reverse() * (180/M_PI);
//         std::cout << "初始化变换: 旋转角(deg): " << init_euler.transpose() << std::endl;
//     }

//     // 5. 转换到标准世界坐标系(W_body)
//     // ---- 位置变换 ----
//     // 计算从W_body原点到当前位置的向量（在W_radar中）
//     Eigen::Vector3d delta_p_in_w_radar = p_body_in_w_radar - t_w_body_in_w_radar;
    
//     // 将该向量旋转到W_body坐标系
//     Eigen::Matrix3d R_w_radar_to_w_body = q_w_body_to_w_radar.conjugate().toRotationMatrix();
//     Eigen::Vector3d p_body_in_w_body = R_w_radar_to_w_body * delta_p_in_w_radar;
    
//     // ---- 姿态变换 ----
//     // q_w_body_to_w_radar: 从W_body到W_radar的旋转
//     // q_body_in_w_radar: 机体在W_radar中的旋转
//     // 则机体在W_body中的旋转：
//     Eigen::Quaterniond q_body_in_w_body = q_w_body_to_w_radar.conjugate() * q_body_in_w_radar;
//     q_body_in_w_body.normalize();
    
//     // 6. 处理其他传感器变换（保持原有逻辑）
//     // 注意：现在使用的是W_body坐标系中的位姿
//     Matrix3d Rr_w = q_body_in_w_body.toRotationMatrix();
//     Vector3d tr_w = p_body_in_w_body;
//     // ... 保持原有IMU变换逻辑 ...
//     Matrix3d Ri_w = Rr_w * Rr_i.inverse();
//     Vector3d ti_w = tr_w - Ri_w * tr_i;

//     // Vector3d euler = mat2euler(Ri_w);
//     // std::cout << "imu euler: " << euler * 180 / 3.14 << std::endl;
//     // std::cout << "imu trans: " << ti_w.transpose() << std::endl;
//     // std::cout << "imu z: " << ti_w(2) << std::endl;
//     // std::cout << "robot z: " << tr_w(2) << std::endl;
//     std::cout << "delta_p_in_w_radar z: " << delta_p_in_w_radar(2) << std::endl;
    
//     // 7. 准备输出
//     VectorXd pose = VectorXd::Zero(7);
//     pose.segment<3>(0) = ti_w; // 使用IMU位置（根据需求）
    
//     // 使用IMU方向（根据需求）
//     Quaterniond q_wi = Quaterniond(Ri_w); 
//     pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());
    
//     // 调试输出：确保初始姿态为零
//     if (!is_initialized) {
//         Vector3d euler = q_wi.toRotationMatrix().eulerAngles(2,1,0).reverse() * (180/M_PI);
//         std::cout << "初始姿态角(deg): " << euler.transpose() << std::endl;
//     }
    
//     return pose;
// }


// changed by mmz 2.0
VectorXd get_pose_from_VIOodom(const nav_msgs::msg::Odometry::ConstPtr &msg)
{
    // 1. 定义雷达安装参数
    // const double yaw_offset_deg = 15.5;
    // const double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;
    //
    // // 安装变换：从雷达坐标系到机体坐标系 (绕Y轴旋转-15度)
    // Eigen::AngleAxisd rotation_vector(-yaw_offset_rad, Eigen::Vector3d::UnitY());
    // Eigen::Quaterniond q_w_body_in_w_radar(rotation_vector);
    // static Eigen::Vector3d t_w_body_in_w_lidar(-0.03763, 0.0, -0.06986);    // W_body原点在W_radar中的位置

    // 由“雷达在IMU系下外参”与“机体在IMU系下外参”推导“机体在雷达系下外参”
    Eigen::Matrix3d R_w_body_in_w_radar = Rl_i.transpose() * Rr_i;
    Eigen::Quaterniond q_w_body_in_w_radar(R_w_body_in_w_radar);
    Eigen::Vector3d t_w_body_in_w_lidar = Rl_i.transpose() * (tr_i - tl_i);
    // T_ib(lidar imu) : q_w_body_in_w_radar, t_w_body_in_w_lidar, R_w_body_in_w_radar
    // Tb_i
    
    // 2.获取原始雷达数据
    // T_wi(lidar imu): p_radar_in_w_radar, q_radar_in_w_radar
    Vector3d p_radar_in_w_radar;
    p_radar_in_w_radar(0) = msg->pose.pose.position.x;
    p_radar_in_w_radar(1) = msg->pose.pose.position.y;
    p_radar_in_w_radar(2) = msg->pose.pose.position.z;
    
    Quaterniond q_radar_in_w_radar;
    q_radar_in_w_radar.w() = msg->pose.pose.orientation.w;
    q_radar_in_w_radar.x() = msg->pose.pose.orientation.x;
    q_radar_in_w_radar.y() = msg->pose.pose.orientation.y;
    q_radar_in_w_radar.z() = msg->pose.pose.orientation.z;

    // 计算姿态变换
    q_w_body_in_w_radar.normalize();
    R_w_body_in_w_radar = q_w_body_in_w_radar.toRotationMatrix();
    q_radar_in_w_radar.normalize();
    Eigen::Matrix3d R_radar_in_w_radar = q_radar_in_w_radar.toRotationMatrix();
    

    // Eigen::Quaterniond q_body_in_w_body = q_w_body_in_w_radar.conjugate() * q_radar_in_w_radar * q_w_body_in_w_radar;
    // q_body_in_w_body.normalize();
    // Eigen::Matrix3d R_body_in_w_body = q_body_in_w_body.toRotationMatrix();
    // R_wb =? R_ib.t * R_wi * R_ib
    Eigen::Matrix3d R_body_in_w_body = R_w_body_in_w_radar.transpose() * R_radar_in_w_radar * R_w_body_in_w_radar;

    /*
    T_wb = T_wi * T_ib
    = [R_wi  t_wi] * [R_ib  t_ib]
      [ 0     1  ]   [  0     1 ]
    = [R_wi * R_ib  R_wi * t_ib + t_wi]
      [     0                1        ]
    */

    // 计算平移量
    // t_wb = R_wi * t_ib + t_wi
    Vector3d t_body_in_w_body = R_w_body_in_w_radar.transpose() * (p_radar_in_w_radar + (R_radar_in_w_radar - Eigen::Matrix3d::Identity()) * t_w_body_in_w_lidar);
    
    // 6. 处理其他传感器变换（保持原有逻辑）
    // 注意：现在使用的是W_body坐标系中的位姿
    Matrix3d Rr_w = R_body_in_w_body;
    Vector3d tr_w = t_body_in_w_body;
    // ... 保持原有IMU变换逻辑 ...
    Matrix3d Ri_w = Rr_w * Rr_i.inverse();
    Vector3d ti_w = tr_w - Ri_w * tr_i;
    // Vector3d ti_w = t_body_in_w_body - tr_i;

    Vector3d p_trans = R_w_body_in_w_radar.transpose() * p_radar_in_w_radar;

    // Vector3d euler = mat2euler(Ri_w);
    // std::cout << "imu euler: " << euler * 180 / 3.14 << std::endl;
    // Vector3d euler = mat2euler(R_radar_in_w_radar);
    // std::cout << "radar euler: " << euler * 180 / 3.14 << std::endl;
    // std::cout << "imu z: " << ti_w(2) << std::endl;
    // std::cout << "body z: " << tr_w(2) << std::endl;
    // std::cout << "p_radar_in_w_radar: " << p_radar_in_w_radar.transpose() << std::endl;
    // std::cout << "p_trans: " << p_trans.transpose() << std::endl;
    // std::cout << "R_w_body_in_w_radar: " << R_w_body_in_w_radar << std::endl;
    // std::cout << "Rr_w: " << Rr_w << std::endl;
    // std::cout << "Ri_w: " << Ri_w << std::endl;
    // std::cout << "Rr_i.inverse(): " << Rr_i.inverse() << std::endl;
    
    // 7. 准备输出
    VectorXd pose = VectorXd::Zero(7);
    pose.segment<3>(0) = ti_w; // 使用IMU位置（根据需求）
    
    // 使用IMU方向（根据需求）
    Quaterniond q_wi = Quaterniond(Ri_w); 
    pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());

    // 发布imu系下的里程计
    nav_msgs::msg::Odometry imu_odom;
    imu_odom.header.stamp = rclcpp::Clock().now();
    imu_odom.header.frame_id = "world";

    imu_odom.pose.pose.orientation.w = q_wi.w();
    imu_odom.pose.pose.orientation.x = q_wi.x();
    imu_odom.pose.pose.orientation.y = q_wi.y();
    imu_odom.pose.pose.orientation.z = q_wi.z();

    imu_odom.pose.pose.position.x = ti_w(0);
    imu_odom.pose.pose.position.y = ti_w(1);
    imu_odom.pose.pose.position.z = ti_w(2);

    rotate_odom_pub->publish(imu_odom);
    
    return pose;
}

// changed by mmz 3.0
// VectorXd get_pose_from_VIOodom(const nav_msgs::msg::Odometry::ConstPtr &msg)
// {
//     // 1. 定义雷达安装参数
//     const double pitch_offset_deg = 15.0; 
//     const double pitch_offset_rad = pitch_offset_deg * M_PI / 180.0;
    
//     // 安装变换：从雷达坐标系到机体坐标系 (绕Y轴旋转-15度)
//     Eigen::AngleAxisd rotation_vector(-pitch_offset_rad, Eigen::Vector3d::UnitY());
//     Eigen::Quaterniond qb_l(rotation_vector);
//     static Eigen::Vector3d tb_l(-0.03763, 0.0, -0.06986);    // W_body原点在W_radar中的位置
//     // T_ib(lidar imu) : q_w_body_in_w_radar, t_w_body_in_w_lidar, R_w_body_in_w_radar
//     // Tb_i
    
//     // 2.获取原始雷达数据
//     // T_wi(lidar imu): p_radar_in_w_radar, q_radar_in_w_radar
//     Vector3d tl_w;
//     tl_w(0) = msg->pose.pose.position.x;
//     tl_w(1) = msg->pose.pose.position.y;
//     tl_w(2) = msg->pose.pose.position.z;
    
//     Quaterniond ql_w;
//     ql_w.w() = msg->pose.pose.orientation.w;
//     ql_w.x() = msg->pose.pose.orientation.x;
//     ql_w.y() = msg->pose.pose.orientation.y;
//     ql_w.z() = msg->pose.pose.orientation.z;

//     // 计算姿态变换
//     qb_l.normalize();
//     Eigen::Matrix3d Rb_l = qb_l.toRotationMatrix();
//     ql_w.normalize();
//     Eigen::Matrix3d Rl_w = ql_w.toRotationMatrix();
    

//     /*
//     T_wb = T_wi * T_ib
//     = [R_wi  t_wi] * [R_ib  t_ib]
//       [ 0     1  ]   [  0     1 ]
//     = [R_wi * R_ib  R_wi * t_ib + t_wi]
//       [     0                1        ]
//     */
   
//     // R_wb = R_wi * R_ib
//     Eigen::Matrix3d Rb_w = Rl_w * Rb_l;

//     // 计算平移量
//     // t_wb = R_wi * t_ib + t_wi
//     Vector3d tb_w = Rl_w * tb_l + tl_w;
    
//     // 6. 处理其他传感器变换（保持原有逻辑）
//     // 注意：现在使用的是W_body坐标系中的位姿
//     Matrix3d Rr_w = Rb_w;
//     Vector3d tr_w = tb_w;
//     // ... 保持原有IMU变换逻辑 ...
//     Matrix3d Ri_w = Rr_w * Rr_i.inverse();
//     Vector3d ti_w = tr_w - Ri_w * tr_i;

//     // 7. 准备输出
//     VectorXd pose = VectorXd::Zero(7);
//     pose.segment<3>(0) = ti_w; // 使用IMU位置（根据需求）
    
//     // 使用IMU方向（根据需求）
//     Quaterniond q_wi = Quaterniond(Ri_w); 
//     pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());
    
//     return pose;
// }


void update_lastest_state()
{
    MatrixXd Ct;
    MatrixXd Wt;
    Ct = diff_g_diff_x();
    // cout << "Ct: " << Ct << endl;
    Wt = diff_g_diff_v();
    // cout << "Wt: " << Wt << endl;

    Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + Wt * Rt * Wt.transpose()).inverse();
    // cout << "Kt_kalmanGain: " << Kt_kalmanGain << endl;
    VectorXd gg = g_model();
    // VectorXd innovation = Z_measurement - gg;
    // VectorXd innovation_t = gg;
    VectorXd innovation = VectorXd::Zero(6);
    innovation.segment<3>(0) = Z_measurement.segment<3>(0) - gg.segment<3>(0);
    Quaterniond q_gg(gg(3), gg(4), gg(5), gg(6));
    Quaterniond q_Z_measurement(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
    innovation.segment<3>(3) = (q_Z_measurement * q_gg.inverse()).vec();
    VectorXd innovation_t = gg;
    // Prevent innovation changing suddenly when euler from -Pi to Pi
    float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
    if (pos_diff > POS_DIFF_THRESHOLD)
    {
        RCLCPP_ERROR(rclcpp::get_logger("update_lastest_state"), "posintion diff too much between measurement and model prediction!!!   pos_diff setting: %f  but the diff measured is %f ", POS_DIFF_THRESHOLD, pos_diff);
        // return;
    }

    // if (innovation(3) > 6)
    //     innovation(3) -= 2 * PI;
    // if (innovation(3) < -6)
    //     innovation(3) += 2 * PI;
    // if (innovation(4) > 6)
    //     innovation(4) -= 2 * PI;
    // if (innovation(4) < -6)
    //     innovation(4) += 2 * PI;
    // if (innovation(5) > 6)
    //     innovation(5) -= 2 * PI;
    // if (innovation(5) < -6)
    //     innovation(5) += 2 * PI;
    INNOVATION_ = innovation_t.segment<3>(3);
    // X_state += Kt_kalmanGain * (innovation);
    X_state = boxplus(X_state, Kt_kalmanGain * (innovation));
    // if (X_state(3) > PI)
    //     X_state(3) -= 2 * PI;
    // if (X_state(3) < -PI)
    //     X_state(3) += 2 * PI;
    // if (X_state(4) > PI)
    //     X_state(4) -= 2 * PI;
    // if (X_state(4) < -PI)
    //     X_state(4) += 2 * PI;
    // if (X_state(5) > PI)
    //     X_state(5) -= 2 * PI;
    // if (X_state(5) < -PI)
    //     X_state(5) += 2 * PI;
    StateCovariance = StateCovariance - Kt_kalmanGain * Ct * StateCovariance;

    // ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
    // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;

    test_odomtag_call = true;
    odomtag_call = true;

    if (INNOVATION_(0) > 6 || INNOVATION_(1) > 6 || INNOVATION_(2) > 6)
        cout << "\ninnovation: \n"
             << INNOVATION_ << endl;
    if (INNOVATION_(0) < -6 || INNOVATION_(1) < -6 || INNOVATION_(2) < -6)
        cout << "\ninnovation: \n"
             << INNOVATION_ << endl;
    // monitor the position changing
    if ((innovation(0) > 1.5) || (innovation(1) > 1.5) || (innovation(2) > 1.5) ||
        (innovation(0) < -1.5) || (innovation(1) < -1.5) || (innovation(2) < -1.5))
        RCLCPP_ERROR(rclcpp::get_logger("update_lastest_state"), "posintion diff too much between measurement and model prediction!!!");
    if (cnt == 10 || cnt == 50 || cnt == 90)
    {
        // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl;
        // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl;
        // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
        // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
        // cout << "\ninnovation: \n" << INNOVATION_ << endl;
    }
    cnt++;
    if (cnt > 100)
        cnt = 101;
}

Vector3d rotation_2_lie_algebra(Matrix3d R)
{

    Eigen::Vector3d omega;
    double theta = std::acos((R.trace() - 1) / 2);

    if (theta < 1e-6)
    {
        omega << 0, 0, 0;
    }
    else
    {
        omega << R(2, 1) - R(1, 2),
            R(0, 2) - R(2, 0),
            R(1, 0) - R(0, 1);
        omega = omega * (theta / (2 * std::sin(theta)));
    }
    // Quaterniond q(R);
    // // 轴角
    // Eigen::AngleAxisd angle_axis(q);
    // omega = angle_axis.angle() * angle_axis.axis();
    return omega;
}

void vioodom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
{ // assume that the odom_tag from camera is sychronized with the imus and without delay. !!!

    // std::cout << "get_odom!!!!!!!!!!!!!!" << std::endl;

    // rclcpp::Time now = rclcpp::Clock().now();
    // auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    // rclcpp::Time rosclock_now = clock->now();

    // // mmz add test time
    // // 获取并格式化消息时间戳
    // rclcpp::Time msg_time = msg->header.stamp;
    // // 1. 正确计算延迟（纳秒级精度）
    // const double delay_ms = (rosclock_now - msg_time).seconds() * 1000.0;
    // // 2. 创建格式化函数（仅用于显示）
    // auto format_local_time_ms = [](const rclcpp::Time& t) {
    //     // 获取毫秒部分（从完整时间戳中提取）
    //     const int64_t total_ns = t.nanoseconds();
    //     const int ms = (total_ns % 1000000000) / 1000000; // 取秒内部分并转换为毫秒
        
    //     // 转换为本地时间
    //     const time_t sec = t.seconds();
    //     struct tm *tm_struct = localtime(&sec);
        
    //     // 格式化时间字符串
    //     char time_str[64];
    //     strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_struct);
        
    //     // 返回格式化的时间字符串
    //     std::ostringstream ss;
    //     ss << time_str << "." << std::setfill('0') << std::setw(3) << ms;
    //     return ss.str();
    // };
    
    // // 3. 输出完整调试信息
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_data"),
    //     "数据时间: " << format_local_time_ms(msg_time) << "|"
    //     "接收时间: " << format_local_time_ms(now) << "|"
    //     "计算延迟: " << std::fixed << std::setprecision(3) << delay_ms << "ms");

    double delaytime_ms = (imu_back_time - rclcpp::Time(msg->header.stamp)).seconds() * 1000;
    double buffertime_ms = (imu_front_time - rclcpp::Time(msg->header.stamp)).seconds() * 1000;
    if (buffertime_ms > 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vioodom_callback"), "delaytime %.2f, buf_size %d", delaytime_ms, int(sys_seq.size()));
        RCLCPP_ERROR(rclcpp::get_logger("vioodom_callback"), "odom time out of buffer time range: %.2fms, ekf may be wrong !!!!", buffertime_ms);
    } 
    // else
    // {
        // ROS_INFO("delaytime %.2f, buf_size %d", delaytime_ms, int(sys_seq.size()));
        // ROS_INFO("odom time out of buffer time range: %.2fms", buffertime_ms);
    // }

    // your code for update
    static Eigen::Vector3d last_pos(0, 0, 0);
    if (first_frame_tag_odom)
    { // system begins in first odom frame
        first_frame_tag_odom = false;
        time_odom_tag_now = rclcpp::Time(msg->header.stamp).seconds();

        VectorXd odom_pose = get_pose_from_VIOodom(msg);
        X_state.segment<3>(0) = odom_pose.segment<3>(0);
        // X_state.segment<3>(3) = odom_pose.segment<3>(3);
        //! changed by wz
        X_state.segment<4>(3) = odom_pose.segment<4>(3);

        world_frame_id = msg->header.frame_id;

        last_pos(0) = msg->pose.pose.position.x;
        last_pos(1) = msg->pose.pose.position.y;
        last_pos(2) = msg->pose.pose.position.z;

        first_odom_get_ = last_pos;

        // cout << "last_pos: "<<last_pos.transpose()<<endl;

        // cout << "\033[1;33m"
        //  << "odom_tag init"
        //  << "\033[0m" << endl;

        // cout << X_state.segment<3>(0).transpose()<<endl;
    }
    else
    {
        // cout << "\033[1;33m"
        //  << "odom_tag update"
        //  << "\033[0m" << endl;
        if (abs(last_pos(0) - msg->pose.pose.position.x) > 0.5 ||
            abs(last_pos(1) - msg->pose.pose.position.y) > 0.5 ||
            abs(last_pos(2) - msg->pose.pose.position.z) > 0.5)
        {
            // return;
        }

        last_pos(0) = msg->pose.pose.position.x;
        last_pos(1) = msg->pose.pose.position.y;
        last_pos(2) = msg->pose.pose.position.z;

        time_odom_tag_now = rclcpp::Time(msg->header.stamp).seconds();
        //    double t = clock();

        VectorXd odom_pose = get_pose_from_VIOodom(msg);
        // Eigen::Vector3d euler_odom(odom_pose(3), odom_pose(4), odom_pose(5));
        //! changed by wz
        Eigen::Quaterniond q_odom(odom_pose(3), odom_pose(4), odom_pose(5), odom_pose(6));
        Matrix3d R_odom;
        // R_odom = euler2mat(euler_odom);
        //! changed by wz
        R_odom = q_odom.toRotationMatrix();
        // double cos_theta_theshold = cos(PI / 180 * 30);
        // double cos_theta = (R_odom * Eigen::Vector3d(0, 0, 1)).dot(Eigen::Vector3d(0, 0, -1));
        // if (cos_theta > cos_theta_theshold)
        // {
        //     ROS_ERROR("flip over!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // }

#if TimeSync
        // call back to the proper time
        if (sys_seq.size() == 0)
        {
            // ROS_ERROR("sys_seq.size() == 0");
            update_lastest_state();
            cam_system_pub(msg->header.stamp);
            return;
        }
        // call back to the proper time
        if (sys_seq.size() == 1)
        {
            // ROS_ERROR("sys_seq.size() == 1");
            update_lastest_state();
            cam_system_pub(msg->header.stamp);
            return;
        }
        search_proper_frame(time_odom_tag_now);

#endif
        Z_measurement.segment<3>(0) = odom_pose.segment<3>(0);
        // Z_measurement.segment<3>(3) = odom_pose.segment<3>(3);
        //! changed by wz
        Z_measurement.segment<4>(3) = odom_pose.segment<4>(3);

        // cam_system_pub(msg->header.stamp);

#if !TimeSync // no aligned
        MatrixXd Ct;
        MatrixXd Wt;
        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + Wt * Rt * Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
        VectorXd innovation_t = gg;

        // Prevent innovation changing suddenly when euler from -Pi to Pi
        float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
        if (pos_diff > POS_DIFF_THRESHOLD)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vioodom_callback"), "posintion diff too much between measurement and model prediction!!!   pos_diff setting: %f  but the diff measured is %f ", POS_DIFF_THRESHOLD, pos_diff);
            return;
        }
        if (innovation(3) > 6)
            innovation(3) -= 2 * PI;
        if (innovation(3) < -6)
            innovation(3) += 2 * PI;
        if (innovation(4) > 6)
            innovation(4) -= 2 * PI;
        if (innovation(4) < -6)
            innovation(4) += 2 * PI;
        if (innovation(5) > 6)
            innovation(5) -= 2 * PI;
        if (innovation(5) < -6)
            innovation(5) += 2 * PI;
        INNOVATION_ = innovation_t.segment<3>(3);
        X_state += Kt_kalmanGain * (innovation);
        1.4571 X_state(3) -= 2 * PI;
        if (X_state(3) < -PI)
            X_state(3) += 2 * PI;
        if (X_state(4) > PI)
            X_state(4) -= 2 * PI;
        if (X_state(4) < -PI)
            X_state(4) += 2 * PI;
        if (X_state(5) > PI)
            X_state(5) -= 2 * PI;
        if (X_state(5) < -PI)
            X_state(5) += 2 * PI;
        StateCovariance = StateCovariance - Kt_kalmanGain * Ct * StateCovariance;

        // ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
        // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;

        test_odomtag_call = true;
        odomtag_call = true;

        if (INNOVATION_(0) > 6 || INNOVATION_(1) > 6 || INNOVATION_(2) > 6)
            cout << "\ninnovation: \n"
                 << INNOVATION_ << endl;
        if (INNOVATION_(0) < -6 || INNOVATION_(1) < -6 || INNOVATION_(2) < -6)
            cout << "\ninnovation: \n"
                 << INNOVATION_ << endl;
        // monitor the position changing
        if ((innovation(0) > 1.5) || (innovation(1) > 1.5) || (innovation(2) > 1.5) ||
            (innovation(0) < -1.5) || (innovation(1) < -1.5) || (innovation(2) < -1.5))
            RCLCPP_ERROR(rclcpp::get_logger("vioodom_callback"), "posintion diff too much between measurement and model prediction!!!");
        if (cnt == 10 || cnt == 50 || cnt == 90)
        {
            // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl;
            // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
            // cout << "\ninnovation: \n" << INNOVATION_ << endl;
        }
        cnt++;
        if (cnt > 100)
            cnt = 101;

#else // time sync
        MatrixXd Ct;
        MatrixXd Wt;

        // re-prediction for the rightframe
        dt = dt_0_rp;

        u_gyro(0) = sys_seq[0].second.angular_velocity.x;
        u_gyro(1) = sys_seq[0].second.angular_velocity.y;
        u_gyro(2) = sys_seq[0].second.angular_velocity.z;
        u_acc(0) = sys_seq[0].second.linear_acceleration.x;
        u_acc(1) = sys_seq[0].second.linear_acceleration.y;
        u_acc(2) = sys_seq[0].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        X_state = sys_seq[0].first;
        StateCovariance = cov_seq[0];

        // q_last = sys_seq[0].first.segment<3>(3);   // last X2
        // bg_last = sys_seq[0].first.segment<3>(9);  // last X4
        // ba_last = sys_seq[0].first.segment<3>(12); // last X5
        //! changed by wz
        // q_last = sys_seq[0].first.segment<4>(3);   // last X2
        q_last.w() = sys_seq[0].first(3);
        q_last.x() = sys_seq[0].first(4);
        q_last.y() = sys_seq[0].first(5);
        q_last.z() = sys_seq[0].first(6);

        bg_last = sys_seq[0].first.segment<3>(10); // last X4
        ba_last = sys_seq[0].first.segment<3>(13); // last X5

        // Ft = MatrixXd::Identity(stateSize, stateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
        //! changed by wz
        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        // std::cout << "Ft" << std::endl
        //           << Ft << std::endl;

        Vt = dt * diff_f_diff_n(q_last);

        // std::cout << "Vt" << std::endl
        //   << Vt << std::endl;

        // X_state += dt * F_model(u_gyro, u_acc);
        //! changed by wz
        X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);
        // std::cout << "X_state" << std::endl
        //   << X_state << std::endl;
        // if (X_state(3) > PI)  //! changed by wz
        //     X_state(3) -= 2 * PI;
        // if (X_state(3) < -PI)
        //     X_state(3) += 2 * PI;
        // if (X_state(4) > PI)
        //     X_state(4) -= 2 * PI;
        // if (X_state(4) < -PI)
        //     X_state(4) += 2 * PI;
        // if (X_state(5) > PI)
        //     X_state(5) -= 2 * PI;
        // if (X_state(5) < -PI)
        //     X_state(5) += 2 * PI;
        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
        // std::cout << "StateCovariance" << std::endl
        //           << StateCovariance << std::endl;

        // re-update for the rightframe
        Ct = diff_g_diff_x();
        // std::cout << "Ct" << std::endl
        //           << Ct << std::endl;
        Wt = diff_g_diff_v();

        // std::cout << "Wt" << std::endl
        //           << Wt << std::endl;

        Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + Wt * Rt * Wt.transpose()).inverse();
        // std::cout << "Kt_kalmanGain" << std::endl
        //           << Kt_kalmanGain << std::endl;

        VectorXd gg = g_model();
        // std::cout << "gg" << std::endl
        //           << gg << std::endl;
        // std::cout << "Z_measurement" << std::endl
        //           << Z_measurement << std::endl;
        // VectorXd innovation = Z_measurement - gg;
        VectorXd innovation = VectorXd::Zero(6);
        innovation.segment<3>(0) = Z_measurement.segment<3>(0) - gg.segment<3>(0);
        // std::cout << "innovation_first" << std::endl
        //           << innovation << std::endl;
        Quaterniond q_gg(gg(3), gg(4), gg(5), gg(6));
        q_gg.normalized();
        Quaterniond q_Z_measurement(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
        q_Z_measurement.normalized();
        // Quaterniond error_q = q_Z_measurement * q_gg.inverse();
        Quaterniond error_q = q_gg.inverse() * q_Z_measurement;
        error_q.normalized();
        // cout << "error_q.vec()" << endl
        //      << error_q.vec() << endl;
        Matrix3d error_R = error_q.toRotationMatrix();
        // 旋转矩阵李代数
        innovation.segment<3>(3) = rotation_2_lie_algebra(error_R);
        // std::cout << "innovation" << std::endl
        //           << innovation << std::endl;
        // std::cout << "\033[1;32m innovation" << std::endl
        //           << innovation << "\033[0m" << std::endl;
        // std::cout << "\033[1;33m position diff: " << sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2)) << std::endl;
        // std::cout << "angle diff: " << sqrt(innovation(3) * innovation(3) + innovation(4) * innovation(4) + innovation(5) * innovation(5)) << "\033[0m" << std::endl;
        VectorXd innovation_t = gg;

        float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
        if (pos_diff > POS_DIFF_THRESHOLD)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vioodom_callback"), "posintion diff too much between measurement and model prediction!!!   pos_diff setting: %f  but the diff measured is %f ", POS_DIFF_THRESHOLD, pos_diff);
            // return;
        }

        // Prevent innovation changing suddenly when euler from -Pi to Pi
        // if (innovation(3) > 6)
        //     innovation(3) -= 2 * PI;
        // if (innovation(3) < -6)w = angle_axis.angle() * angle_axis.axis();
        //     innovation(3) += 2 * PI;
        // if (innovation(4) > 6)
        //     innovation(4) -= 2 * PI;
        // if (innovation(4) < -6)
        //     innovation(4) += 2 * PI;
        // if (innovation(5) > 6)
        //     innovation(5) -= 2 * PI;
        // if (innovation(5) < -6)
        //     innovation(5) += 2 * PI;

        // Quaterniond test_q = Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6)) * error_q;
        // std::cout << "\033[1;31m test_q" << std::endl
        //           << test_q << "\033[0m" << std::endl;

        INNOVATION_ = innovation_t.segment<3>(3);

        VectorXd dx(errorstateSize);
        dx = VectorXd::Zero(errorstateSize);
        dx += Kt_kalmanGain * (innovation);
        // dx += (innovation);
        // dx.segment<6>(0) = innovation.segment<6>(0);
        // std::cout << "dx" << std::endl
        //           << dx << std::endl;

        // X_state += Kt_kalmanGain * (innovation);
        X_state = boxplus(X_state, dx);
        // std::cout << "odom callback q: " << " q.x " << X_state(4) << " q.y " << X_state(5) << " q.z " << X_state(6) << " q.w " << X_state(3) << std::endl;
        Vector3d X_state_p = X_state.segment<3>(0);
        VectorXd final_innovation = VectorXd::Zero(6);
        final_innovation.segment<3>(0) = Z_measurement.segment<3>(0) - X_state_p;
        // std::cout << "\033[1;32m Z_measurement.segment<3>(0)" << std::endl
        //           << Z_measurement.segment<3>(0) << "\033[0m" << std::endl;
        // std::cout << "X_state_p" << std::endl
        //           << X_state_p << std::endl;

        Quaterniond q_X_state_q(X_state(3), X_state(4), X_state(5), X_state(6));
        Quaterniond q_Z_measurement_(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
        Quaterniond error_q_ = q_Z_measurement_ * q_X_state_q.inverse();
        Matrix3d error_R_ = error_q_.toRotationMatrix();
        final_innovation.segment<3>(3) = rotation_2_lie_algebra(error_R_);
        // std::cout << "\033[1;32m final_innovation" << std::endl
        //           << final_innovation << "\033[0m" << std::endl;
        // std::cout << "\033[1;33m position diff: " << sqrt(final_innovation(0) * final_innovation(0) + final_innovation(1) * final_innovation(1) + final_innovation(2) * final_innovation(2)) << std::endl;
        // std::cout << "angle diff: " << sqrt(final_innovation(3) * final_innovation(3) + final_innovation(4) * final_innovation(4) + final_innovation(5) * final_innovation(5)) << "\033[0m" << std::endl;

        // std::cout << "X_state_update" << std::endl
        //           << X_state << std::endl;
        // if (X_state(3) > PI)
        //     X_state(3) -= 2 * PI;
        // if (X_state(3) < -PI)
        //     X_state(3) += 2 * PI;
        // if (X_state(4) > PI)
        //     X_state(4) -= 2 * PI;
        // if (X_state(4) < -PI)
        //     X_state(4) += 2 * PI;
        // if (X_state(5) > PI)
        //     X_state(5) -= 2 * PI;
        // if (X_state(5) < -PI)
        //     X_state(5) += 2 * PI;
        StateCovariance = StateCovariance - Kt_kalmanGain * Ct * StateCovariance;

        // system_pub(X_state, sys_seq[0].second.header.stamp); // choose to publish the repropagation or not
        // std::cout << "re_propagate" << std::endl;

        re_propagate();

#endif
        cam_system_pub(msg->header.stamp);
    }
}
Matrix3d lie_algebra_2_rotation(Vector3d v)
{

    Eigen::Matrix3d R;
    double theta = v.norm();

    if (theta < 1e-6)
    {
        R = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Matrix3d skew;
        skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        R = Eigen::Matrix3d::Identity() + skew / theta * std::sin(theta) + skew * skew / theta / theta * (1 - std::cos(theta));
        // R = skew.exp();
    }

    return R;
}

Quaterniond rotation_matrix_param_to_quaternion(
    const std::vector<double> &rotation_matrix_param,
    const std::string &param_label)
{
    if (rotation_matrix_param.size() == 9)
    {
        Matrix3d rotation_matrix;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                rotation_matrix(row, col) = rotation_matrix_param[row * 3 + col];
            }
        }

        Quaterniond rotation_quaternion = mat2quaternion(rotation_matrix);
        rotation_quaternion.normalize();
        cout << param_label << " loaded from rotation matrix." << endl;
        return rotation_quaternion;
    }

    cout << param_label << " invalid, fallback to identity." << endl;
    return Quaterniond::Identity();
}

VectorXd boxplus(VectorXd x, VectorXd dx)
{
    VectorXd x_plus(x.rows());
    x_plus(0) = x(0) + dx(0);
    x_plus(1) = x(1) + dx(1);
    x_plus(2) = x(2) + dx(2);

    Vector3d dv(dx(3), dx(4), dx(5));
    Matrix3d dR = lie_algebra_2_rotation(dv);
    Quaterniond x_q(x(3), x(4), x(5), x(6));
    Matrix3d x_R = x_q.toRotationMatrix();
    Matrix3d x_R_plus = x_R * dR;
    Quaterniond x_q_plus(x_R_plus);
    x_q_plus.normalized();
    x_plus(3) = x_q_plus.w();
    x_plus(4) = x_q_plus.x();
    x_plus(5) = x_q_plus.y();
    x_plus(6) = x_q_plus.z();

    x_plus(7) = x(7) + dx(6);
    x_plus(8) = x(8) + dx(7);
    x_plus(9) = x(9) + dx(8);

    x_plus(10) = x(10) + dx(9);
    x_plus(11) = x(11) + dx(10);
    x_plus(12) = x(12) + dx(11);

    x_plus(13) = x(13) + dx(12);
    x_plus(14) = x(14) + dx(13);
    x_plus(15) = x(15) + dx(14);

    return x_plus;
}

Quaterniond q_gt, q_gt0;
bool first_gt = true;
void gt_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
{
    q_gt.w() = msg->pose.pose.orientation.w;
    q_gt.x() = msg->pose.pose.orientation.x;
    q_gt.y() = msg->pose.pose.orientation.y;
    q_gt.z() = msg->pose.pose.orientation.z;

    if (first_gt && !first_frame_tag_odom)
    {
        first_gt = false;
        q_gt0 = q_gt;
        // q_gt0 = q_gt0.normalized();
    }
}

int main(int argc, char **argv)
{
    std::cout << "start!" << std::endl;
    int core_id = 5;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
    {
        std::cerr << "Failed to set CPU affinity for thread: ekf "<< std::endl;
    } 
    else 
    {
        std::cout << "Successfully set CPU affinity to core " << core_id << std::endl;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ekf");

    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu", imu_qos, imu_callback);
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto vioodom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "bodyodometry", odom_qos, vioodom_callback);
    auto gt_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "gt_", 40, gt_callback);

    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1000);
    ahead_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("ahead_ekf_odom", 1000);
    cam_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("cam_ekf_odom", 1000);
    rotate_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("rotate_lio_odom", 1000);
    acc_filtered_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("acc_filtered", 1000);

    node->declare_parameter("gyro_cov", 0.0);
    node->declare_parameter("acc_cov", 0.0);
    node->declare_parameter("position_cov", 0.0);
    node->declare_parameter("q_rp_cov", 0.0);
    node->declare_parameter("q_yaw_cov", 0.0);
    node->declare_parameter("imu_trans_x", 0.0);
    node->declare_parameter("imu_trans_y", 0.0);
    node->declare_parameter("imu_trans_z", 0.0);
    node->declare_parameter("cutoff_freq", 0.0);
    node->declare_parameter("offset_px", 0.0);
    node->declare_parameter("offset_py", 0.0);
    node->declare_parameter("offset_pz", 0.0);
    node->declare_parameter("acc_rej_threshold", 0.0);
    node->declare_parameter("gyro_rej_threshold", 0.0);

    node->get_parameter("gyro_cov", gyro_cov);
    node->get_parameter("acc_cov", acc_cov);
    node->get_parameter("position_cov", position_cov);
    node->get_parameter("q_rp_cov", q_rp_cov);
    node->get_parameter("q_yaw_cov", q_yaw_cov);
    node->get_parameter("imu_trans_x", imu_trans_x);
    node->get_parameter("imu_trans_y", imu_trans_y);
    node->get_parameter("imu_trans_z", imu_trans_z);
    node->get_parameter("cutoff_freq", cutoff_freq);
    node->get_parameter("offset_px", offset_px);
    node->get_parameter("offset_py", offset_py);
    node->get_parameter("offset_pz", offset_pz);
    node->get_parameter("acc_rej_threshold", acc_rej_threshold);
    node->get_parameter("gyro_rej_threshold", gyro_rej_threshold);

    cout << "Q:" << gyro_cov << " " << acc_cov << " R: " << position_cov << " " << q_rp_cov << " " << q_yaw_cov << endl;

    std::vector<double> Rri;
    std::vector<double> tri, Rli_rotation_matrix, tli, rotationimu;
    node->declare_parameter("Rr_i", std::vector<double>{1.0, 0.0, 0.0, 0.0});
    node->declare_parameter("tr_i", std::vector<double>{0.0, 0.0, 0.0});
    node->declare_parameter("Rl_i_rotation_matrix", std::vector<double>{});
    node->declare_parameter("tl_i", std::vector<double>{0.0, 0.0, 0.0});
    node->declare_parameter("scale_g", 0.0);
    node->declare_parameter("rotation_imu", std::vector<double>{1.0, 0.0, 0.0, 
                                                                0.0, 1.0, 0.0, 
                                                                0.0, 0.0, 1.0});

    node->get_parameter("Rr_i", Rri);
    node->get_parameter("tr_i", tri);
    node->get_parameter("Rl_i_rotation_matrix", Rli_rotation_matrix);
    node->get_parameter("tl_i", tli);
    node->get_parameter("scale_g", scale_g);
    node->get_parameter("rotation_imu", rotationimu);

    Quaterniond q_r_i(Rri.at(0), Rri.at(1), Rri.at(2), Rri.at(3));
    q_r_i.normalize();
    Rr_i = q_r_i.toRotationMatrix();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotation_imu(i, j) = rotationimu[i * 3 + j];
        }
    }
    cout << "rotation_imu = " << endl 
         << rotation_imu << endl;
    cout << "scale_g = " << scale_g << endl;
    tr_i << tri.at(0), tri.at(1), tri.at(2);

    const bool use_legacy_lidar_in_imu =
        Rli_rotation_matrix.empty() &&
        std::abs(tli.at(0)) < 1.0e-9 &&
        std::abs(tli.at(1)) < 1.0e-9 &&
        std::abs(tli.at(2)) < 1.0e-9;

    if (use_legacy_lidar_in_imu)
    {
        const double yaw_offset_deg = 15.5;
        const double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;
        Eigen::AngleAxisd rotation_vector(-yaw_offset_rad, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d R_w_body_in_w_radar_legacy = Eigen::Quaterniond(rotation_vector).toRotationMatrix();
        Eigen::Vector3d t_w_body_in_w_lidar_legacy(-0.03763, 0.0, -0.06986);

        Rl_i = Rr_i * R_w_body_in_w_radar_legacy.transpose();
        tl_i = tr_i - Rl_i * t_w_body_in_w_lidar_legacy;

        cout << "Use legacy lidar-body extrinsic to back-compute Rl_i/tl_i." << endl;
    }
    else
    {
        Quaterniond q_l_i = rotation_matrix_param_to_quaternion(Rli_rotation_matrix, "Rl_i_rotation_matrix");
        Rl_i = q_l_i.toRotationMatrix();
        tl_i << tli.at(0), tli.at(1), tli.at(2);
    }

    cout << "Rr_i: " << endl
         << Rr_i << endl;
    cout << "tr_i: " << endl
         << tr_i << endl;
    cout << "Rl_i: " << endl
         << Rl_i << endl;
    cout << "tl_i: " << endl
         << tl_i << endl;
    cout << "R_body_in_lidar: " << endl
         << Rl_i.transpose() * Rr_i << endl;
    cout << "t_body_in_lidar: " << endl
         << (Rl_i.transpose() * (tr_i - tl_i)).transpose() << endl;

    initsys();
    cout << "initsys" << endl;

    // rclcpp::spin(node);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        // std::cout<< "running" << std::endl;
    }
    rclcpp::shutdown();
}

void acc_f_pub(Vector3d acc, rclcpp::Time stamp)
{
    geometry_msgs::msg::PoseStamped Accel_filtered;
    Accel_filtered.header.frame_id = "world";
    Accel_filtered.header.stamp = stamp;
    // Accel_filtered.header.stamp = ros::Time::now();
    Vector3d Acc_ = get_filtered_acc(acc);
    Accel_filtered.pose.position.x = Acc_[0];
    Accel_filtered.pose.position.y = Acc_[1];
    Accel_filtered.pose.position.z = Acc_[2];

    Quaterniond q;
    // q = euler2quaternion(X_state.segment<3>(3));
    //! changed by wz
    q = X_state.segment<4>(3);
    // q = q.normalized();
    Accel_filtered.pose.orientation.w = q.w();
    Accel_filtered.pose.orientation.x = q.x();
    Accel_filtered.pose.orientation.y = q.y();
    Accel_filtered.pose.orientation.z = q.z();

    // Accel_filtered.pose.orientation.w = q_gt.w();
    // Accel_filtered.pose.orientation.x = q_gt.x();
    // Accel_filtered.pose.orientation.y = q_gt.y();
    // Accel_filtered.pose.orientation.z = q_gt.z();

    cout << "q_gt0: " << quaternion2euler(q_gt0) << endl;
    cout << "q_gt: " << quaternion2euler(q_gt) << endl;
    cout << "q_vio: " << mat2euler(q_gt0.toRotationMatrix() * euler2quaternion(X_state.segment<3>(3)).toRotationMatrix()) << endl;
    cout << "q_gt0*vio != q_gt: " << quaternion2euler(q_gt0 * q) << endl;
    // cout << "q_gt0*vio*q_gt0^-1 = q_gt: " << quaternion2euler(q_gt0 * q * q_gt0.inverse()) << endl;   //q1*euler2quaternion(V)*q1.inverse()  = q1.toRotationMatrix() * V  TODO why?

    acc_filtered_pub->publish(Accel_filtered);
}

void ahead_system_pub(const Eigen::VectorXd &X_state_in, rclcpp::Time stamp)
{
    nav_msgs::msg::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";

    Quaterniond q;
    // q = euler2quaternion(X_state_in.segment<3>(3));
    //! changed by wz
    q = X_state_in.segment<4>(3);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    // odom_fusion.twist.twist.linear.x = X_state_in(6);
    // odom_fusion.twist.twist.linear.y = X_state_in(7);
    // odom_fusion.twist.twist.linear.z = X_state_in(8);
    //! changed by wz
    odom_fusion.twist.twist.linear.x = X_state_in(7);
    odom_fusion.twist.twist.linear.y = X_state_in(8);
    odom_fusion.twist.twist.linear.z = X_state_in(9);

    Vector3d pos_center(X_state_in(0), X_state_in(1), X_state_in(2)), pos_center2;
    pos_center2 = pos_center + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    odom_fusion.pose.pose.position.x = pos_center2(0);
    odom_fusion.pose.pose.position.y = pos_center2(1);
    odom_fusion.pose.pose.position.z = pos_center2(2);

    ahead_odom_pub->publish(odom_fusion);
}

void system_pub(const Eigen::VectorXd &X_state_in, rclcpp::Time stamp)
{
    nav_msgs::msg::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = "world";
    odom_fusion.child_frame_id = "drone_" + to_string(drone_id);
    // odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";

    Quaterniond q;
    // q = euler2quaternion(X_state_in.segment<3>(3));
    //! changed by wz
    q.w() = X_state_in(3);
    q.x() = X_state_in(4);
    q.y() = X_state_in(5);
    q.z() = X_state_in(6);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    // std::cout << " q.x " << q.x() << " q.y " << q.y() << " q.z " << q.z() << " q.w " << q.w() << std::endl;

    // odom_fusion.twist.twist.linear.x = X_state_in(6);
    // odom_fusion.twist.twist.linear.y = X_state_in(7);
    // odom_fusion.twist.twist.linear.z = X_state_in(8);
    //! changed by wz
    odom_fusion.twist.twist.linear.x = X_state_in(7);
    odom_fusion.twist.twist.linear.y = X_state_in(8);
    odom_fusion.twist.twist.linear.z = X_state_in(9);

    Vector3d pos_center(X_state_in(0), X_state_in(1), X_state_in(2)),pos_center1, pos_center2;
    Vector3d pos_center_filter;

    // static bool first_flag = true;
    // static bool second_flag = true;
    static Vector3d last_pos_center(pos_center);
    static Vector3d last_last_pos_center(pos_center);
    static Vector3d current_pos_center(pos_center);

    double sample_freq = 200;
    double freq = sample_freq / cutoff_freq;
    double ohm = tan(PI / freq);
    double c = 1.0 + 2.0 * cos(PI / 4.0) * ohm + ohm * ohm;
    double b0 = ohm * ohm / c;
    double b1 = 2.0 * b0;
    double b2 = b0;
    double a1 = 2.0 * (ohm * ohm - 1.0) / c;
    double a2 = (1.0 - 2.0 * cos(PI / 4.0) * ohm + ohm * ohm) / c;
    // if (first_flag)
    // {
    //     last_last_pos_center = pos_center;
    //     first_flag = false;
    //     return;
    // }
    // if (!first_flag & second_flag)
    // {
    //     last_pos_center = pos_center;
    //     second_flag = false;
    //     return;
    // }
    current_pos_center = pos_center - a1 * last_pos_center - a2 * last_last_pos_center;

    pos_center_filter = b0 * current_pos_center + b1 * last_pos_center + b2 * last_last_pos_center;

    last_last_pos_center = last_pos_center;

    last_pos_center = current_pos_center;
    // cout << "pos_center!!!!!!!!!!!!!!" << endl
    //  << pos_center << endl;

    pos_center1 = pos_center + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    pos_center2 = pos_center_filter + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    // cout << "q.toRotationMatrix()" << endl
    //      << q.toRotationMatrix() << endl;
    // cout << "imu_trans:" << imu_trans_x << " " << imu_trans_y << " " << imu_trans_z << endl;
    // cout << "pos_center2???????????????????????????" << endl
    //      << pos_center2 << endl;
    if(cutoff_freq < 1.0e-5)
    {
        odom_fusion.pose.pose.position.x = pos_center1(0);
        odom_fusion.pose.pose.position.y = pos_center1(1);
        odom_fusion.pose.pose.position.z = pos_center1(2);
    } else {
        odom_fusion.pose.pose.position.x = pos_center2(0);
        odom_fusion.pose.pose.position.y = pos_center2(1);
        odom_fusion.pose.pose.position.z = pos_center2(2);
    }

    odom_fusion.pose.pose.position.x += offset_px;
    odom_fusion.pose.pose.position.y += offset_py;
    odom_fusion.pose.pose.position.z += offset_pz;

    static int cnt = 0;
    if(cnt < 1000){
        cnt++;
        return;
    } 
    
    // std::cout << "pub final odom!!!" << std::endl;
    odom_pub->publish(odom_fusion);
}

void cam_system_pub(rclcpp::Time stamp)
{
    nav_msgs::msg::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";
    // odom_fusion.pose.pose.position.x = Z_measurement(0);
    // odom_fusion.pose.pose.position.y = Z_measurement(1);
    // odom_fusion.pose.pose.position.z = Z_measurement(2);
    odom_fusion.pose.pose.position.x = Z_measurement(0);
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;

    // q = euler2quaternion(Z_measurement.segment<3>(3));
    //! changed by wz
    q = Z_measurement.segment<4>(3);

    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    // odom_fusion.twist.twist.linear.x = Z_measurement(3);
    // odom_fusion.twist.twist.linear.y = Z_measurement(4);
    // odom_fusion.twist.twist.linear.z = Z_measurement(5);

    // odom_fusion.twist.twist.angular.x = INNOVATION_(0);
    // odom_fusion.twist.twist.angular.y = INNOVATION_(1);
    // odom_fusion.twist.twist.angular.z = INNOVATION_(2);
    // Vector3d pp, qq, v, bg, ba;
    // getState(pp, qq, v, bg, ba);
    // odom_fusion.twist.twist.angular.x = ba(0);
    // odom_fusion.twist.twist.angular.y = ba(1); ///??????why work??????????//////
    // odom_fusion.twist.twist.angular.z = ba(2);
    // odom_fusion.twist.twist.angular.x = diff_time;
    // odom_fusion.twist.twist.angular.y = dt;
    cam_odom_pub->publish(odom_fusion);
}

// process model
void initsys()
{
    //  camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    // set the cam2imu params
    Rc_i = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    // cout << "R_cam" << endl << Rc_i << endl;
    tc_i << 0.05, 0.05, 0;

    //  rigid body position in the IMU frame = (0, 0, 0.04)
    // rigid body orientaion in the IMU frame = Quaternion(1, 0, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //						 	             0, 1, 0,
    //                                       0, 0, 1;

    // states X [p q pdot bg ba]  [px,py,pz, wx,wy,wz, vx,vy,vz bgx,bgy,bgz bax,bay,baz]
    // stateSize = 15;                      // x = [p q pdot bg ba]

    stateSize = 16; //! changed by wz

    errorstateSize = 15; // ! changed by wz

    // stateSize_pqv = 9;                   // x = [p q pdot]

    stateSize_pqv = 10; // ! changed by wz

    // measurementSize = 6;                 // z = [p q]

    measurementSize = 7; //! changed by wz

    inputSize = 6;                       // u = [w a]
    X_state = VectorXd::Zero(stateSize); // x
    // velocity
    // X_state(6) = 0;
    // X_state(7) = 0;
    // X_state(8) = 0;
    X_state(3) = 1.0; //! changed by wz
    X_state(4) = 0;
    X_state(5) = 0;
    X_state(6) = 0;
    X_state(7) = 0;
    X_state(8) = 0;
    X_state(9) = 0;
    // bias
    X_state.segment<3>(10) = bg_0; //! changed by wz
    X_state.segment<3>(13) = ba_0;
    u_input = VectorXd::Zero(inputSize);
    Z_measurement = VectorXd::Zero(measurementSize); // z
    // StateCovariance = MatrixXd::Identity(stateSize, stateSize);     // sigma
    //! changed by wz
    StateCovariance = MatrixXd::Identity(errorstateSize, errorstateSize); // sigma

    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize); // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct
    X_state_correct = X_state;
    StateCovariance_correct = StateCovariance;

    Qt = MatrixXd::Identity(inputSize, inputSize); // 6x6 input [gyro acc]covariance
    // Rt = MatrixXd::Identity(measurementSize, measurementSize); // 6x6 measurement [p q]covariance
    //! changed by wz
    Rt = MatrixXd::Identity(measurementSize - 1, measurementSize - 1); // 6x6 measurement [p q]covariance
    // MatrixXd temp_Rt = MatrixXd::Identity(measurementSize, measurementSize);

    // You should also tune these parameters
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // //Rt visual odomtry covariance smaller believe measurement more
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
}

// void getState(Vector3d &p, Vector3d &q, Vector3d &v, Vector3d &bg, Vector3d &ba)
// {
//     p = X_state.segment<3>(0);
//     q = X_state.segment<3>(3);
//     v = X_state.segment<3>(6);
//     bg = X_state.segment<3>(9);
//     ba = X_state.segment<3>(12);
// }
//! changed by wz
void getState(Vector3d &p, Quaterniond &q, Vector3d &v, Vector3d &bg, Vector3d &ba)
{
    p = X_state.segment<3>(0);
    // q = X_state.segment<4>(3);
    q = Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6));
    v = X_state.segment<3>(7);
    bg = X_state.segment<3>(10);
    ba = X_state.segment<3>(13);
}

// VectorXd get_filtered_acc(Vector3d acc)
// {
//     Vector3d q, ba;
//     q = X_state.segment<3>(3);
//     ba = X_state.segment<3>(12);

//     // return (euler2mat(q)*(acc-ba-na));
//     // return (q_gt.toRotationMatrix()*(acc-ba-na));  //false
//     return ((acc - ba - na));
//     // return ((acc-na));
//     // return (euler2mat(q)*(acc));
// }
//! changed by wz
VectorXd get_filtered_acc(Vector3d acc)
{
    Vector3d ba;
    ba = X_state.segment<3>(13);
    return ((acc - ba - na));
}

// VectorXd F_model(Vector3d gyro, Vector3d acc)
// {
//     // IMU is in FLU frame
//     // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
//     VectorXd f(VectorXd::Zero(stateSize));
//     Vector3d p, q, v, bg, ba;
//     getState(p, q, v, bg, ba);
//     f.segment<3>(0) = v;
//     f.segment<3>(3) = w_Body2Euler(q) * (gyro - bg - ng);
//     f.segment<3>(6) = gravity + euler2mat(q) * (acc - ba - na);
//     f.segment<3>(9) = nbg;
//     f.segment<3>(12) = nba;

//     return f;
// }

//! changed by wz
VectorXd F_model(Vector3d gyro, Vector3d acc)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd f(VectorXd::Zero(stateSize));
    Vector3d p, v, bg, ba;
    Quaterniond q;
    getState(p, q, v, bg, ba);
    f.segment<3>(0) = v;                          // 0,1,2
    f.segment<3>(4) = q * (gyro - bg - ng) * 0.5; // 4,5,6
    f.segment<3>(7) = gravity + q * (acc - ba - na);
    f.segment<3>(10) = nbg;
    f.segment<3>(13) = nba;

    return f;
}

//? added by wz
VectorXd upate_state_Quaterniond_F_model(VectorXd X_state, Vector3d gyro, Vector3d acc, double dt)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd f(VectorXd::Zero(stateSize));

    VectorXd upate_X_state(VectorXd::Zero(stateSize));

    Vector3d p, v, bg, ba;
    Quaterniond q;
    getState(p, q, v, bg, ba);

    f.segment<3>(0) = v;                          // 0,1,2
    f.segment<3>(4) = q * (gyro - bg - ng) * 0.5; // 4,5,6
    f.segment<3>(7) = gravity + q * (acc - ba - na);
    f.segment<3>(10) = nbg;
    f.segment<3>(13) = nba;

    upate_X_state.segment<3>(0) = X_state.segment<3>(0) + v * dt + 0.5 * (gravity + q * (acc - ba - na)) * dt * dt;
    // upate_X_state.segment<3>(0) = X_state.segment<3>(0) + v * dt;
    Quaterniond delta_q(1.0, 0.5 * (gyro(0) - bg(0) - ng(0)) * dt, 0.5 * (gyro(1) - bg(1) - ng(1)) * dt, 0.5 * (gyro(2) - bg(2) - ng(2)) * dt);
    Quaterniond upate_q = (q * delta_q).normalized();
    upate_X_state(3) = upate_q.w();
    upate_X_state(4) = upate_q.x();
    upate_X_state(5) = upate_q.y();
    upate_X_state(6) = upate_q.z();
    upate_X_state.segment<3>(7) = X_state.segment<3>(7) + (gravity + q * (acc - ba - na)) * dt;
    // std::cout << "vx: " << upate_X_state(7) << "vy: " << upate_X_state(8) << "vz: " << upate_X_state(9) << std::endl;
    // std::cout << "gravity: " << gravity << std::endl;
    // std::cout << "q: " << q << std::endl;
    // Matrix3d R_temp = q.toRotationMatrix();
    // Vector3d euler = mat2euler(R_temp);
    // std::cout << "update euler: " << euler * 180 / 3.14 << std::endl;
    // std::cout << "acc befor q : " << (acc - ba - na)(2) << std::endl;
    // std::cout << "acc after q : " << (q * (acc - ba - na))(2) << std::endl;
    // std::cout << "acc : " << acc << std::endl;
    // std::cout << "ba + na : " << ba + na << std::endl;
    // std::cout << "ba : " << ba(1) << std::endl;
    // std::cout << "na : " << na(1) << std::endl;
    // std::cout << "delta v: " << (gravity + q * (acc - ba - na)) << std::endl;
    upate_X_state.segment<3>(10) = X_state.segment<3>(10) + nbg * dt;
    upate_X_state.segment<3>(13) = X_state.segment<3>(13) + nba * dt;
    // std::cout << "ba : " << X_state.segment<3>(13) << std::endl;
    return upate_X_state;
}

// VectorXd g_model()
// {
//     VectorXd g(VectorXd::Zero(measurementSize));

//     g.segment<6>(0) = X_state.segment<6>(0);

//     // if(g(3) > PI)  g(3) -= 2*PI;
//     // if(g(3) < -PI) g(3) += 2*PI;
//     // if(g(4) > PI)  g(4) -= 2*PI;
//     // if(g(4) < -PI) g(4) += 2*PI;
//     // if(g(5) > PI)  g(5) -= 2*PI;
//     // if(g(5) < -PI) g(5) += 2*PI;

//     return g;
// }

//! changed by wz
VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    g.segment<7>(0) = X_state.segment<7>(0);

    // if(g(3) > PI)  g(3) -= 2*PI;
    // if(g(3) < -PI) g(3) += 2*PI;
    // if(g(4) > PI)  g(4) -= 2*PI;
    // if(g(4) < -PI) g(4) += 2*PI;
    // if(g(5) > PI)  g(5) -= 2*PI;
    // if(g(5) < -PI) g(5) += 2*PI;

    return g;
}

// F_model G_model Jocobian
// diff_f()/diff_x (x_t-1  ut  noise=0)   At     Ft = I+dt*At
// MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
// {
//     double cr = cos(q_last(0));
//     double sr = sin(q_last(0));
//     double cp = cos(q_last(1));
//     double sp = sin(q_last(1));
//     double cy = cos(q_last(2));
//     double sy = sin(q_last(2));

//     // ng na = 0 nbg nba = 0
//     double Ax = acc(0) - ba_last(0);
//     double Ay = acc(1) - ba_last(1);
//     double Az = acc(2) - ba_last(2);
//     // double Wx = gyro(0) - bg_last(0);
//     double Wy = gyro(1) - bg_last(1);
//     double Wz = gyro(2) - bg_last(2);

//     MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(stateSize, stateSize));
//     MatrixXd diff_f_diff_x_jacobian_pqv(MatrixXd::Zero(stateSize_pqv, stateSize_pqv));

//     diff_f_diff_x_jacobian_pqv << 0, 0, 0, 0, 0, 0, 1, 0, 0,
//         0, 0, 0, 0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 0, 0, 0, 1,
//         0, 0, 0, (sp * (Wy * cr - Wz * sr)) / cp, (Wz * cr + Wy * sr) / (cp * cp), 0, 0, 0, 0,
//         0, 0, 0, (-Wz * cr - Wy * sr), 0, 0, 0, 0, 0,
//         0, 0, 0, (Wy * cr - Wz * sr) / cp, (sp * (Wz * cr + Wy * sr)) / (cp * cp), 0, 0, 0, 0,
//         0, 0, 0, (Ay * (sr * sy + cr * cy * sp) + Az * (cr * sy - cy * sp * sr)), (Az * cp * cr * cy - Ax * cy * sp + Ay * cp * cy * sr), (Az * (cy * sr - cr * sp * sy) - Ay * (cr * cy + sp * sr * sy) - Ax * cp * sy), 0, 0, 0,
//         0, 0, 0, (-Ay * (cy * sr - cr * sp * sy) - Az * (cr * cy + sp * sr * sy)), (Az * cp * cr * sy - Ax * sp * sy + Ay * cp * sr * sy), (Az * (sr * sy + cr * cy * sp) - Ay * (cr * sy - cy * sp * sr) + Ax * cp * cy), 0, 0, 0,
//         0, 0, 0, (Ay * cp * cr - Az * cp * sr), (-Ax * cp - Az * cr * sp - Ay * sp * sr), 0, 0, 0, 0;

//     diff_f_diff_x_jacobian.block<9, 9>(0, 0) = diff_f_diff_x_jacobian_pqv;
//     diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -w_Body2Euler(q_last);
//     diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -euler2mat(q_last);

//     return diff_f_diff_x_jacobian;

//     // cp != 0 pitch != 90° !!!!!!!!!
// }

//? added by wz
Matrix3d hat(Vector3d v)
{
    Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return v_hat;
}

//! changed by wz
MatrixXd diff_f_diff_x(Quaterniond q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{

    // MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(stateSize, stateSize));
    // MatrixXd diff_f_diff_x_jacobian_pqv(MatrixXd::Zero(stateSize_pqv, stateSize_pqv));
    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(errorstateSize, errorstateSize));
    diff_f_diff_x_jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(); // dp/dv
    diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
    diff_f_diff_x_jacobian.block<3, 3>(6, 3) = -q_last.toRotationMatrix() * hat(acc - ba_last); //!!!!
    diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -q_last.toRotationMatrix();
    return diff_f_diff_x_jacobian;
}

// diff_f()/diff_n (x_t-1  ut  noise=0)  Ut    Vt = dt*Ut
// MatrixXd diff_f_diff_n(Vector3d q_last)
// {
//     MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(stateSize, inputSize));
//     diff_f_diff_n_jacobian.block<3, 3>(3, 0) = -w_Body2Euler(q_last);
//     diff_f_diff_n_jacobian.block<3, 3>(6, 3) = -euler2mat(q_last);

//     return diff_f_diff_n_jacobian;
// }

//! changed by wz
MatrixXd diff_f_diff_n(Quaterniond q_last)
{
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(errorstateSize, inputSize));
    diff_f_diff_n_jacobian.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    diff_f_diff_n_jacobian.block<3, 3>(6, 3) = -q_last.toRotationMatrix();

    return diff_f_diff_n_jacobian;
}

// // diff_g()/diff_x  (xt~ noise=0)  Ct
// MatrixXd diff_g_diff_x()
// {
//     MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize, stateSize));
//     diff_g_diff_x_jacobian.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
//     diff_g_diff_x_jacobian.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3);

//     return diff_g_diff_x_jacobian;
// }

//! changed by wz
MatrixXd diff_g_diff_x()
{
    // MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize, stateSize));
    // diff_g_diff_x_jacobian.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
    // diff_g_diff_x_jacobian.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3);

    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize - 1, errorstateSize));
    diff_g_diff_x_jacobian.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
    diff_g_diff_x_jacobian.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3);

    return diff_g_diff_x_jacobian;
}

// // diff_g()/diff_v  (xt~ noise=0) Wt
// MatrixXd diff_g_diff_v()
// {
//     MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize, measurementSize));

//     return diff_g_diff_v_jacobian;
// }
//! changed by wz
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize - 1, measurementSize - 1));

    return diff_g_diff_v_jacobian;
}