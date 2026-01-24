# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.

import launch
import launch_ros.actions


def generate_launch_description():
    """
    Launch file for latency testing.
    Environment variables are set by the bash wrapper script.
    """

    return launch.LaunchDescription([
        # Declare launch argument for max messages
        launch.actions.DeclareLaunchArgument(
            'max_messages',
            default_value='0',
            description='Maximum messages to receive (0 = unlimited)'
        ),

        # Latency test node
        launch_ros.actions.Node(
            package='latency_test',
            executable='latency_test_node',
            name='latency_test_node',
            parameters=[{
                'max_messages': launch.substitutions.LaunchConfiguration('max_messages')
            }],
            output='screen'
        ),
    ])
