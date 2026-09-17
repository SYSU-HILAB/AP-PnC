# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
# Tail-sitter UAV.

"""Planning stack launch (trajectory_server + traj_buffer).

Nodes read their configuration directly from bringup/config/planning.yaml
($AP_PNC_PLANNING_YAML override, cwd-relative fallback); no ROS parameters
are passed here. Use `debug:=true` to run the planner under gdbserver :7777.
"""

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    debug = LaunchConfiguration("debug")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "debug", default_value="false",
            description="Run global_planning_node under gdbserver :7777",
        ),
        launch_ros.actions.Node(
            package="traj_buffer",
            executable="traj_buffer_node",
            name="traj_buffer_node",
        ),
        launch_ros.actions.Node(
            package="trajectory_server",
            executable="trajectory_server_node",
            name="global_planning_node",
            prefix=["gdbserver :7777"],
            condition=launch.conditions.IfCondition(debug),
            output="screen",
        ),
        launch_ros.actions.Node(
            package="trajectory_server",
            executable="trajectory_server_node",
            name="global_planning_node",
            condition=launch.conditions.UnlessCondition(debug),
            output="screen",
        ),
    ])
