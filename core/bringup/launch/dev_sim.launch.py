"""Interactive ROS2 plant and analytic LiDAR; no hardware or offline recorder."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = Path(get_package_share_directory("bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Absolute run-local practical plant config"),
        DeclareLaunchArgument("lidar_config", default_value=str(bringup / "config/lidar_sim.yaml")),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(package="simple_sim", executable="simple_sim_node", name="simple_sim", output="screen",
             parameters=[{"config": ParameterValue(LaunchConfiguration("config"), value_type=str)}]),
        Node(package="lidar_sim", executable="lidar_sim_node", name="lidar_sim", output="screen",
             parameters=[{"use_sim_time": True,
                          "config": ParameterValue(LaunchConfiguration("lidar_config"), value_type=str)}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(bringup / "rviz/dev_sim.rviz")],
             parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
