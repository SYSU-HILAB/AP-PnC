import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "c_nmpc"

    position_arg = DeclareLaunchArgument(
        'tgt_pos',
        default_value='[0.0, 0.0, 1.0]',
        description='Target Position parameter as a double array'
    )
    quat_arg = DeclareLaunchArgument(
        'tgt_quat',
        default_value='[1.0, 0.0, 0.0, 0.0]',
        description='Target Quat parameter as a double array'
    )
    position = LaunchConfiguration('tgt_pos')
    quat = LaunchConfiguration('tgt_quat')
    
    px4_interface = "px4_interface"
    return launch.LaunchDescription([
        position_arg,
        quat_arg,
        launch_ros.actions.Node(
            package=f'{px4_interface}', 
            executable='fixed_ref', 
            name='fixed_ref',
            parameters=[
                {
                    "pos": position,
                    "quat": quat
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
        # launch_ros.actions.Node(
        #     package=f'{px4_interface}', 
        #     executable=f'{px4_interface}', 
        #     name=f'{px4_interface}',
        #     # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        # ),
        ])
