import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "c_nmpc"

    position_arg = DeclareLaunchArgument(
        'target_position',
        default_value='[0.0, 0.0, 2.0]',
        description='Target Position parameter as a double array'
    )
    position = LaunchConfiguration('target_position')
    
    return launch.LaunchDescription([
        position_arg,
        launch_ros.actions.Node(
            package='px4_interface', 
            executable='fixed_ref', 
            name='fixed_ref',
            parameters=[
                {
                    "pos": position,
                }
            ]
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),
        
        launch_ros.actions.Node(
            package=f'{pkg_name}', 
            executable=f'{pkg_name}_node', 
            name=f'{pkg_name}_node',
            parameters=[{
                'mass': 1.152,
            }]
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),

        ])
