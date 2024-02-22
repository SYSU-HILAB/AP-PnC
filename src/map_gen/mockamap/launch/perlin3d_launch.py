import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    perlin3d = os.path.join(
        get_package_share_directory('mockamap'),
        'config',
        'perlin3d.yaml'
        )
    print(perlin3d) # the result shows that in install folder
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mockamap',  # 替换为你的包名
            executable='mockamap_node',  # 替换为你的节点名
            name='mockamap',
            parameters=[perlin3d]  # 指定参数文件路径
        )
    ])