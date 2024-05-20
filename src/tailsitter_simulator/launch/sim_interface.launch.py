from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "tailsitter_simulator"
    sim_params = os.path.join(
        get_package_share_directory(f'{pkg_name}'),
        'config',
        'interface_config.yaml'
    )
    hover_param = os.path.join(
        get_package_share_directory(f'{pkg_name}'),
        'config',
        'hover_example.yaml'
    )

    traj_buffer_pkg = "traj_buffer"
    return LaunchDescription([
        Node(
            package=f'{pkg_name}',     # 替换为你的包名
            executable=f'{pkg_name}_interface',     # 替换为你的节点名
            name=f'{pkg_name}_interface',      # 节点的运行时名称，可以自定义
            output='screen',          # 日志输出到屏幕
            parameters=[sim_params]  # 参数列表，根据需要添加或修改
        ),
        # Node(
        #     package=f'{traj_buffer_pkg}',     # 替换为你的包名
        #     executable=f'{traj_buffer_pkg}_node',     # 替换为你的节点名
        #     name=f'{traj_buffer_pkg}_node',      # 节点的运行时名称，可以自定义
        #     output='screen',          # 日志输出到屏幕
        #     # parameters=[hover_param]  # 参数列表，根据需要添加或修改
        # )
    ])