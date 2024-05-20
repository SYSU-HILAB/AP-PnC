import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    px4_interface = "px4_interface"
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=f'{px4_interface}', 
            executable='px4_interface', 
            name='px4_interface',
            parameters=[
                {
                    # Actuallly, if it is not NED, it will be FRD
                    "world_frame": "NED",
                }
            ]
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),
        
        ])
