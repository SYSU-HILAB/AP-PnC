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

"""Core control stack launch (nmpc) — research code container.

nmpc reads its configuration from <AP_PNC_DIR>/bringup/config/nmpc.yaml;
no ROS parameters are passed here.

The base services (mavros, px4ctrl, LIO) run in the nav_infra container;
nmpc consumes /cur_state and feeds /mpc_controls over the shared ROS 2
graph. px4ctrl (nav_infra) owns arming/offboard and the PASS_THROUGH
handover: /px4ctrl/toggle_pass_through switches its remote rates source
(internal NMPC controls or an external stream).
"""

import launch
import launch_ros.actions


def generate_launch_description():
    nmpc_node = launch_ros.actions.Node(
        package="nmpc",
        executable="nmpc_node",
        name="nmpc_node",
        output="screen",
    )

    return launch.LaunchDescription([
        nmpc_node,
    ])
