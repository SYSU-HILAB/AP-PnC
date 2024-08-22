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
    
    return launch.LaunchDescription([
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
                    "circled": False,
                    "real_world": True,
                    "exp_coefs": [0.08869374325729838, 0.11713376801189856, 0.12567841873810845, 0.15005373297969835, 0.1324185630496418, 0.24883612641861838, 0.20757431730331458, 0.5001433323539398, 0.3877382239367887, 0.08974898500006634, 0.14520286916801822, 0.08072293755821285],
                    
                    "sim_coefs": [-0.006025408485039036, 0.10552418930866049, 0.11435379292303376, 0.1204375356702782, 0.12346230053980424, 0.13989344737753323, 0.14292038764494552, 0.14876465430137734, 0.1076285085723879, 0.12210946689115867, 0.051637288133733104]
                }
            ]            
        ),
    ])
