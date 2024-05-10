import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    gcopter_params = os.path.join(
        get_package_share_directory('gcopter'),
        'config',
        'global_planning.yaml'
        )
    mockamap_params = os.path.join(
        get_package_share_directory('gcopter'),
        'config',
        'perlin3d.yaml'
        )
    
    gcopter_rviz = os.path.join(
        get_package_share_directory('gcopter'),
        'config',
        'gcopter.rviz'
        )
    gcopter_plt = os.path.join(
        get_package_share_directory('gcopter'),
        'config',
        'pltjuggler.xml'
        )
    print(gcopter_rviz)
    print(f"gcopter_params {gcopter_params}") # the result shows that in install folder
    print(f"mockamap_params {mockamap_params}") # the result shows that in install folder
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rqt_plot', 
            executable='rqt_plot', 
            name='pos_vel_plot',
            arguments=['/visualizer/speed/data', '/visualizer/total_thrust/data', '/visualizer/tilt_angle/data', '/visualizer/body_rate']
        ),
        launch_ros.actions.Node(
            package='rqt_plot', 
            executable='rqt_plot', 
            name='pos_vel_plot',
            arguments=['/qw', '/qx', '/qy', '/qz']
        )
    ])