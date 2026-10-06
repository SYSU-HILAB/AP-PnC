#include "ekf.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Eigen>
#include <random>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    // 初始化ROS2系统
    rclcpp::init(argc, argv);

    // 创建节点对象
    auto node = rclcpp::Node::make_shared("ekf_add_noise");

    // 声明并获取参数（带默认值）
    double imu_noise = 5.05;
    node->declare_parameter("imu_noise", imu_noise);  // 声明参数并设置默认值
    node->get_parameter("imu_noise", imu_noise);       // 获取实际参数值
    cout << "imu_noise: " << imu_noise << endl;

    // 初始化随机数生成器（使用硬件随机种子）
    random_device rd;
    default_random_engine generator(rd());
    normal_distribution<double> distribution_imu(0.0, imu_noise);

    // 创建带TCP_NODELAY选项的QoS配置
    rclcpp::QoS imu_qos = rclcpp::SensorDataQoS();

    // 创建带噪声IMU消息的发布者
    auto imu_noise_pub = node->create_publisher<sensor_msgs::msg::Imu>(
        "imu_noise", imu_qos);

    // 创建原始IMU订阅者（使用lambda作为回调）
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", imu_qos,
        // "/livox/imu", imu_qos,
        [&imu_noise_pub, &generator, &distribution_imu](
            const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            
            // 生成三维高斯噪声
            Vector3d noise(
                distribution_imu(generator),
                distribution_imu(generator),
                distribution_imu(generator)
            );

            // 创建带噪声的IMU消息副本
            sensor_msgs::msg::Imu imu_with_noise = *msg;
            imu_with_noise.linear_acceleration.x += noise(0);
            imu_with_noise.linear_acceleration.y += noise(1);
            imu_with_noise.linear_acceleration.z += noise(2);

            // 发布带噪声的IMU消息
            imu_noise_pub->publish(imu_with_noise);
        });

    // 启动节点自旋（阻塞等待回调）
    rclcpp::spin(node);

    // 清理资源（理论上不会执行到这里）
    rclcpp::shutdown();
    return 0;
}