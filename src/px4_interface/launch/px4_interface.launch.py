# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

import launch
import launch_ros.actions


def generate_launch_description():
    px4_interface = "px4_interface"
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package=f"{px4_interface}",
                executable="px4_interface",
                name="px4_interface",
            ),
        ]
    )
