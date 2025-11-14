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
    pos_init_dis_threshold = data['px4_interface']['pos_init_dis_threshold']
    real_world = data['px4_interface']['real_world']


def generate_launch_description():

    frame_arg = DeclareLaunchArgument(
        'frame',
        default_value='FRD',
        description='Target Quat parameter as a double array'
    )
    frame = LaunchConfiguration('frame')
    px4_interface = 'px4_interface'
    return launch.LaunchDescription([
        frame_arg,
        launch_ros.actions.Node(
            package=f'{px4_interface}',
            executable='px4_interface',
            name='px4_interface',
            parameters=[
                {
                    # Actuallly, if it is not NED, it will be FRD
                    "world_frame": frame,
                    "position_init_distance_threshold": pos_init_dis_threshold,
                    "real_world": real_world,
                }
            ]
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),

        ])
