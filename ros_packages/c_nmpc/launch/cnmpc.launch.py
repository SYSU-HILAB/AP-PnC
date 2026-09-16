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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='c_nmpc',
            executable='c_nmpc_node',
            name='c_nmpc_node',
            environment={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'ROS_DISABLE_LOANED_MESSAGES': '0',
                'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
                'FASTRTPS_DEFAULT_PROFILES_FILE': (
                    os.path.join(get_package_share_directory('bringup'), 'config', 'fastdds.xml')
                    if 'bringup' in [pkg for pkg in [os.path.basename(p) for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')]]
                    else (
                        os.path.join(get_package_share_directory('bringup'), 'config', 'fastdds.xml')
                        if os.path.exists(os.path.join(os.environ.get('AP_PNC_DIR', ''), 'bringup', 'config', 'fastdds.xml'))
                        else ''
                    )
                )
            }
        ),
        ])
