# Debug launch file with gdbserver support
# Run with VSCode "cpp: plan (gdbserver)" configuration

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import toml

# Get AP_PNC_DIR from environment or derive from script location
tailsitter_planning_dir = os.environ.get('AP_PNC_DIR')
if not tailsitter_planning_dir:
    # Fallback: derive from package share directory
    trajectory_server_share = get_package_share_directory('trajectory_server')
    tailsitter_planning_dir = os.path.abspath(os.path.join(trajectory_server_share, '..', '..', '..', '..'))

toml_file_path = os.path.join(tailsitter_planning_dir, 'global.toml')

def generate_launch_description():
    gcopter_params = os.path.join(
        get_package_share_directory('trajectory_server'),
        'config',
        'global_planning.yaml'
    )

    return launch.LaunchDescription([
        # Trajectory buffer node (normal, no gdb)
        launch_ros.actions.Node(
            package='traj_buffer',
            executable='traj_buffer_node',
            name='traj_buffer_node',
        ),
        # Trajectory server node with gdbserver
        # Use prefix to run with gdbserver listening on port 7777
        launch_ros.actions.Node(
            package='trajectory_server',
            executable='trajectory_server_node',
            name='global_planning_node',
            parameters=[gcopter_params],
            prefix=['gdbserver :7777'],
            output='screen'
        ),
    ])
