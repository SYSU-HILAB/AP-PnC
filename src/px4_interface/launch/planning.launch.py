import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions  import LaunchConfiguration
import toml
import os

tailsitter_planning_dir = os.environ.get('TAILSITTER_PLANNING_DIR')
toml_file_path = os.path.join(tailsitter_planning_dir, 'global.toml')

with open(toml_file_path, 'r') as f:
    data = toml.load(f)
    circled = data['setup']['circled']
    real_world = data['setup']['real_world']
    acc_aero_fix_factor = data['setup']['acc_aero_fix_factor']
    deacc_aero_fix_factor = data['setup']['deacc_aero_fix_factor']

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
                    "acc_aero_fix_factor": acc_aero_fix_factor,
                    "deacc_aero_fix_factor": deacc_aero_fix_factor,
                    "circled": circled,
                    "real_world": real_world,
                    "exp_coefs": [0.1387766698624702, 0.15086446439894022, 0.14814087043980925, 0.15630179261251512, 0.15017800123438266, 0.18901333957728833, 0.20970763482210145, 0.47247723679240644, 0.3700733706893793, 0.10410391107861101, 0.1361060926019335, 0.08479930823050803],
                    
                    "sim_coefs": [-0.006087762933612023, 0.1057090785564747, 0.11407502989730964, 0.12072394348777712, 0.12289630382134968, 0.14100206411911037, 0.14238579567865853, 0.14897561406276877, 0.10751786530044145, 0.12219391908369212, 0.05160556974143099]
                }
            ]            
        ),
    ])
