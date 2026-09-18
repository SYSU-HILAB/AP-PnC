"""Core control stack launch (c_nmpc + px4ctrl) — research code container.

profile:=sim|real selects the px4ctrl parameter set (px4ctrl.yaml vs
px4ctrl_real.yaml, e.g. hover_percentage 0.9 sim / 0.5 real) and the
default odometry source.

Nodes read their configuration from the conventional cwd-relative
bringup/config yaml files; no ROS parameters are passed here except for
px4ctrl, which follows the vendored ROS2 parameter style.

The base services (mavros, nmpc_bridge, LIO) run in the nav_infra
container; px4ctrl consumes /mavros/* and the bridge's world-frame
odometry from there over the shared ROS 2 graph. px4ctrl also owns the
PASS_THROUGH state: /px4ctrl/toggle_pass_through switches it to forward
the NMPC rates stream verbatim (single mavros setpoint publisher).
"""

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    fastdds_xml = os.path.join(bringup_share, "config", "fastdds.xml")

    profile = LaunchConfiguration("profile")
    px4ctrl_params = PythonExpression(
        ["'", bringup_share, "/config/px4ctrl' + ('' if '", profile,
         "'.lower() == 'sim' else '_real') + '.yaml'"])

    odom_topic = LaunchConfiguration("odom_topic")

    c_nmpc_node = launch_ros.actions.Node(
        package="c_nmpc",
        executable="c_nmpc_node",
        name="c_nmpc_node",
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DISABLE_LOANED_MESSAGES": "0",
            "RMW_FASTRTPS_USE_QOS_FROM_XML": "1",
            "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_xml,
        },
        output="screen",
    )

    px4ctrl_node = launch_ros.actions.Node(
        package="px4ctrl",
        executable="px4ctrl_node",
        name="px4ctrl",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("odom", odom_topic),
            ("cmd", "/setpoint_cmd"),
            ("takeoff_land", "/px4ctrl/takeoff_land"),
        ],
        parameters=[px4ctrl_params],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "profile",
            default_value="sim",
            description="sim | real (px4ctrl parameter set + odom default)",
        ),
        launch.actions.DeclareLaunchArgument(
            "odom_topic",
            default_value=PythonExpression([
                "'/ekf_quat/ekf_odom' if '", profile,
                "'.lower() == 'real' else '/nmpc_bridge/odom_world'"]),
            description="Odometry source for px4ctrl: world-frame corrected "
                        "mavros odom (sim) or /ekf_quat/ekf_odom (real)",
        ),
        c_nmpc_node,
        px4ctrl_node,
    ])
