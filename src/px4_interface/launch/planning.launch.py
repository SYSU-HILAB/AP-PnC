import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


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
            package='px4_interface', 
            executable='traj_buffer', 
            name='traj_buffer_node',
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),

        launch_ros.actions.Node(
            package=f'{pkg_name}', 
            executable=f'{pkg_name}_interface', 
            name='traj_buffer_node',
            parameters=[sim_params]
            # arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        ),
        # launch_ros.actions.Node(
        #     package='plotjuggler',
        #     executable='plotjuggler',
        # ),
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='mincoRviz',
        #     arguments=['-d', gcopter_rviz],
        # ),
    ])
