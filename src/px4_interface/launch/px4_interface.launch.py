import os

import launch
import launch_ros.actions
import toml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

AP_PNC_DIR = os.environ.get("AP_PNC_DIR")
toml_file_path = os.path.join(AP_PNC_DIR, "global.toml")

with open(toml_file_path, "r") as f:
    data = toml.load(f)
    pos_init_dis_threshold = data["px4_interface"]["pos_init_dis_threshold"]


def generate_launch_description():
    px4_interface = "px4_interface"
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package=f"{px4_interface}",
                executable="px4_interface",
                name="px4_interface",
                parameters=[
                    {
                        "position_init_distance_threshold": pos_init_dis_threshold,
                    }
                ],
            ),
        ]
    )


# TODO gdb launch
# from launch_ros.actions import Node
# from launch.actions import OpaqueFunction
# from launch.substitutions import LaunchConfiguration
# import os

# # 1. 定义一个用于接收调试端口的启动参数
# def launch_setup(context, *args, **kwargs):
#     # 获取用户在 ros2 launch 命令行中指定的端口号
#     debug_port = LaunchConfiguration('debug_port').perform(context)

#     # 检查端口是否被指定，如果指定了则使用 gdbserver
#     prefix_command = []
#     if debug_port:
#         print(f"--- Launching node in GDB Server mode on port {debug_port} ---")
#         # gdbserver 的格式：gdbserver <address>:<port>
#         prefix_command = ['gdbserver', f'localhost:{debug_port}']

#     target_node = Node(
#         package='<your_package_name>',
#         executable='<your_executable_name>',
#         name='my_debug_node',
#         output='screen',
#         # 2. 将 prefix 属性设置为 gdbserver 命令
#         # 注意：这里 gdbserver 不需要 --args，launch 系统会自动处理后续参数
#         prefix=prefix_command,
#     )

#     return [target_node]

# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import TextSubstitution

# def generate_launch_description():

#     # 3. 声明一个 launch 参数，用于在命令行指定端口
#     debug_port_arg = DeclareLaunchArgument(
#         'debug_port',
#         default_value=TextSubstitution(text=''), # 默认值为空，表示不启动调试
#         description='If set, launches the node with gdbserver on the specified port (e.g., 3000)'
#     )

#     return LaunchDescription([
#         debug_port_arg,
#         OpaqueFunction(function=launch_setup)
#     ])


# {
#     "version": "0.2.0",
#     "configurations": [
#         {
#             "name": "ROS2 GDB Debug Node via Launch File",
#             "request": "attach",             // 请求类型：附加 (Attach)
#             "type": "cppdbg",                // 调试器类型：C++ Debug

#             // !!! 替换为完整的 Node 可执行文件路径 (例如：/home/user/ros2_ws/install/.../my_node) !!!
#             "program": "/path/to/your/ros2_ws/install/<package_name>/lib/<package_name>/<executable_name>",

#             "MIMode": "gdb",
#             "miDebuggerServerAddress": "localhost:3000", // 必须与 Launch 文件中指定的端口一致
#             "cwd": "${workspaceFolder}",
#             "setupCommands": [
#                 {
#                     "description": "Enable pretty printing for gdb",
#                     "text": "-enable-pretty-printing",
#                     "ignoreFailures": true
#                 }
#             ]
#         }
#     ]
# }
