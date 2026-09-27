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

"""Planning stack launch (planner_node).

The planner package packs the whole planning line into one process: the
trajectory optimizer, the reference buffer module, and the ROS interfaces.
Configuration is read from <AP_PNC_DIR>/bringup/config/planning.yaml.
Use `debug:=true` to run planner_node under gdbserver :7777.
"""

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    debug = LaunchConfiguration("debug")

    node_args = dict(
        package="planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "debug", default_value="false",
            description="Run planner_node under gdbserver :7777",
        ),
        launch_ros.actions.Node(
            **node_args,
            prefix=["gdbserver :7777"],
            condition=launch.conditions.IfCondition(debug),
        ),
        launch_ros.actions.Node(
            **node_args,
            condition=launch.conditions.UnlessCondition(debug),
        ),
    ])
