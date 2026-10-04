"""Tracking-only lockstep simulation, assembled by bringup.

The simulator calls planner/NMPC synchronously in-process. Do not launch the
asynchronous flight planner/NMPC nodes alongside this simulation.
"""

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
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Absolute override; empty uses AP_PNC_DIR bringup config",
            ),
            DeclareLaunchArgument(
                "controller", default_value="", description="Optional nmpc|se3 override"
            ),
            Node(
                package="simple_sim",
                executable="simple_sim_node",
                name="simple_sim",
                output="screen",
                parameters=[
                    {
                        "config": ParameterValue(LaunchConfiguration("config"), value_type=str),
                        "controller": ParameterValue(
                            LaunchConfiguration("controller"), value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(bringup / "rviz" / "simple_sim.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
