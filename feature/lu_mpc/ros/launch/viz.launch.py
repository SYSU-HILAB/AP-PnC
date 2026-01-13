# -*- coding: future-fstrings -*-
#
# Launch file for nMPC Visualization
# Created on 2025-01-10
# Updated on 2025-01-14: Renamed viz_node → simulation_visualizer
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteCommand
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch the nMPC simulation visualizer and RViz2.

    Uses SimulationVisualizer which runs the full nMPC controller
    and CasADi simulator for closed-loop visualization.
    """

    # Get the path to the RViz config
    rviz_config_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'nmpc_viz.rviz'
    )

    # nMPC Simulation Visualizer Node
    nmpc_viz_node = Node(
        package='feature_lu_mpc_ros',
        executable='simulation_visualizer',
        name='simulation_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': False}
        ],
        condition=None
    )

    return LaunchDescription([
        nmpc_viz_node,
        rviz_node,
    ])
