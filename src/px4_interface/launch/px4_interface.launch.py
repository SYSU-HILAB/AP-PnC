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

import os

import launch
import launch_ros.actions
import toml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

AP_PNC_DIR = os.environ.get("AP_PNC_DIR")
toml_file_path = os.path.join(AP_PNC_DIR, "global.toml")

with open(toml_file_path, "r") as f:
    data = toml.load(f)
    pos_init_dis_threshold = data["px4_interface"]["pos_init_dis_threshold"]


def generate_launch_description():
    px4_interface = "px4_interface"
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package=f"{px4_interface}",
                executable="px4_interface",
                name="px4_interface",
                parameters=[
                    {
                        "position_init_distance_threshold": pos_init_dis_threshold,
                    }
                ],
            ),
        ]
    )
