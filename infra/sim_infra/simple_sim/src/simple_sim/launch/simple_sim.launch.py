import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.join(os.path.dirname(__file__), "..")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz2 visualization"
    )

    simple_sim_node = Node(
        package="simple_sim",
        executable="simple_sim_node",
        name="simple_sim",
        output="screen",
        parameters=[os.path.join(pkg_dir, "config", "simple_sim.yaml")],
    )

    rviz_cmd = ExecuteProcess(
        cmd=["rviz2", "-d", os.path.join(pkg_dir, "rviz", "simple_sim.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([rviz_arg, simple_sim_node, rviz_cmd])
