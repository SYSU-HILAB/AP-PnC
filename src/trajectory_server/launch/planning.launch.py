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
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import toml
import os

tailsitter_planning_dir = os.environ.get('AP_PNC_DIR')
toml_file_path = os.path.join(tailsitter_planning_dir, 'global.toml')

with open(toml_file_path, 'r') as f:
    data = toml.load(f)
    circled = data['setup']['circled']

def generate_launch_description():
    gcopter_params = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'global_planning.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='trajectory_server',
            executable='trajectory_server_node',
            name='global_planning_node',
            parameters=[gcopter_params],
            # remappings=[
            #     ('/voxel_map', '/mock_map'),
            #     ('/move_base_simple/goal', '/goal_pose')
            # ]
        ),
        launch_ros.actions.Node(
            package='traj_buffer',
            executable='traj_buffer_node',
            name='traj_buffer_node',
            parameters=[
                {
                    "circled": circled,
                }
            ]
        ),
    ])
