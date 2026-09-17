"""nav-sidecar launch: the single navigation gateway container profile.

    ros2 launch bringup nav_sidecar.launch.py profile:=sim
    ros2 launch bringup nav_sidecar.launch.py profile:=real

sim  : mavros2 only (PX4 SITL over TCP 4560), odometry from
       /mavros/local_position/odom (PX4 EKF2).
real : livox_ros_driver2 (Mid-360) + fast_lio + ekf_quat + mavros2 (FCU
       serial), odometry from /ekf_quat/ekf_odom.

The container image is identical for both profiles; only this bringup
selection differs (compose command override or k3s ConfigMap).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# This mavros build creates plugin topics under a child node
# (/mavros/mavros/*); remap the consumed ones to canonical /mavros/...
# (pattern from the agent_am_demo L0 gateway).
IMU_REMAPPINGS = [
    ("/mavros/mavros/data", "/mavros/imu/data"),
    ("/mavros/mavros/data_raw", "/mavros/imu/data_raw"),
    ("/mavros/mavros/attitude", "/mavros/imu/attitude"),
]


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    nav_config_dir = os.path.join(bringup_share, "config", "nav")

    profile = LaunchConfiguration("profile")
    is_sim = PythonExpression(["'", profile, "'.lower() == 'sim'"])
    is_real = PythonExpression(["'", profile, "'.lower() == 'real'"])

    mavros_sim = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        remappings=IMU_REMAPPINGS,
        parameters=[os.path.join(nav_config_dir, "mavros_sim.yaml")],
        condition=IfCondition(is_sim),
    )

    mavros_real = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        remappings=IMU_REMAPPINGS,
        parameters=[os.path.join(nav_config_dir, "mavros_real.yaml")],
        condition=IfCondition(is_real),
    )

    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            # 1 = livox CustomMsg (FAST-LIO mid360 profile expects it).
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": os.path.join(nav_config_dir,
                                             "mid360_config.json"),
        }],
        condition=IfCondition(is_real),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[os.path.join(nav_config_dir, "fastlio_mid360.yaml")],
        condition=IfCondition(is_real),
    )

    ekf_quat = Node(
        package="ekf_quat_pose",
        executable="ekf_quat",
        name="ekf_quat",
        output="screen",
        parameters=[os.path.join(nav_config_dir, "ekf.yaml")],
        remappings=[
            ("imu", "/mavros/imu/data"),
            ("bodyodometry", "/Odometry"),
            ("ekf_odom", "/ekf_quat/ekf_odom"),
        ],
        condition=IfCondition(is_real),
    )

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="sim",
                              description="sim | real"),
        mavros_sim,
        mavros_real,
        livox,
        fast_lio,
        ekf_quat,
    ])
