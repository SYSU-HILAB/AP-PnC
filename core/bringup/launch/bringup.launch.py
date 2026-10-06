# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
# Tail-sitter UAV.

"""Core control stack bringup.

Runs the whole core line as composable components in a single process:
  planner::node::PlannerNode  (trajectory optimization + reference buffer)
  NMPC                        (acados NMPC -> /nmpc/control)

`component_container_mt` is used on purpose: the planner blocks for a few
hundred ms during a solve, which must not stall the NMPC control loop.
`use_intra_process_comms` makes the in-container reference/control handoff
zero-copy; all core topics use the default (volatile) durability, which
intra-process requires.

Configuration is read from <AP_PNC_DIR>/bringup/config/{planning,nmpc}.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="core_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="planner",
                plugin="planner::node::PlannerNode",
                name="planner_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="nmpc",
                plugin="NMPC",
                name="nmpc_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
