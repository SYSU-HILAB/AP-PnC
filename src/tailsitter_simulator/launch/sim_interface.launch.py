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
    return LaunchDescription([
        Node(
            package=f'{pkg_name}',     # 替换为你的包名
            executable=f'{pkg_name}_interface',     # 替换为你的节点名
            name=f'{pkg_name}_interface',      # 节点的运行时名称，可以自定义
            output='screen',          # 日志输出到屏幕
            parameters=[sim_params]  # 参数列表，根据需要添加或修改
        ),
        Node(
            package=f'{pkg_name}',     # 替换为你的包名
            executable=f'{pkg_name}_hover',     # 替换为你的节点名
            name=f'{pkg_name}_hover',      # 节点的运行时名称，可以自定义
            output='screen',          # 日志输出到屏幕
            parameters=[hover_param]  # 参数列表，根据需要添加或修改
        )
    ])