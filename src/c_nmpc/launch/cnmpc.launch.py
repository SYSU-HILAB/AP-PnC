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
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import toml
import os

AP_PNC_DIR = os.environ.get('AP_PNC_DIR')
toml_file_path = os.path.join(AP_PNC_DIR, 'global.toml')

with open(toml_file_path, 'r') as f:
    data = toml.load(f)
    cz_rate = data['c_nmpc']['cz_rate']

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='c_nmpc',
            executable='c_nmpc_node',
            name='c_nmpc_node',
            parameters=[
                {
                  "cz_rate": cz_rate
                }
            ],
            environment={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'ROS_DISABLE_LOANED_MESSAGES': '0',
                'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
                    get_package_share_directory('traj_buffer'),
                    'fastdds.xml')
            }
        ),
        ])
