import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration


def generate_launch_description():
    gcopter_params = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'global_planning.yaml'
    )
    pkg_name = "tailsitter_simulator"
    sim_params = os.path.join(
        get_package_share_directory(f'{pkg_name}'),
        'config',
        'interface_config.yaml'
    )
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
    
    return launch.LaunchDescription([
        position_arg,
        quat_arg,
        launch_ros.actions.Node(
            package='trajectory_server',
            executable='trajectory_server_node',
            name='global_planning_node',
            parameters=[gcopter_params],
            # prefix=['gdbserver localhost:3000'],
            remappings=[
                ('/voxel_map', '/mock_map'),
                ('/move_base_simple/goal', '/goal_pose')
            ]
        ),
        launch_ros.actions.Node(
            package='traj_buffer', 
            executable='traj_buffer', 
            name='traj_buffer',
            parameters=[
                {
                    "pos": position,
                    "quat": quat
                }
            ]            
        ),
    ])
