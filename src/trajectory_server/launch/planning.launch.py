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
    mockamap_params = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'perlin3d.yaml'
    )

    gcopter_rviz = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'gcopter.rviz'
    )
    gcopter_plt = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'pltjuggler.xml'
    )
    print(gcopter_rviz)
    print(f"gcopter_params {gcopter_params}")  # the result shows that in install folder
    print(f"mockamap_params {mockamap_params}")  # the result shows that in install folder
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
        # launch_ros.actions.Node(
        #     package='rqt_plot',
        #     executable='rqt_plot',
        #     name='pos_vel_plot',
        #     arguments=['/visualizer/speed', '/visualizer/total_thrust', '/visualizer/tilt_angle', '/visualizer/body_rate']
        # ),

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
