"""Control stack launch (c_nmpc + nmpc_bridge + flight_mux + px4ctrl).

Nodes read their configuration from the conventional cwd-relative
bringup/config yaml files; no ROS parameters are passed here except for
px4ctrl, which follows the vendored ROS2 parameter style.

px4ctrl owns arming/offboard via the mavros services; flight_mux is the
single publisher of /mavros/setpoint_raw/attitude (px4ctrl's stream is
remapped onto an internal topic).
"""

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    fastdds_xml = os.path.join(bringup_share, "config", "fastdds.xml")
    px4ctrl_params = os.path.join(bringup_share, "config", "px4ctrl.yaml")

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

    nmpc_bridge_node = launch_ros.actions.Node(
        package="px4_interface",
        executable="nmpc_bridge",
        name="nmpc_bridge",
        output="screen",
    )

    flight_mux_node = launch_ros.actions.Node(
        package="px4_interface",
        executable="flight_mux",
        name="flight_mux",
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
            # Keep flight_mux the single mavros setpoint publisher.
            ("/mavros/setpoint_raw/attitude", "/px4ctrl/setpoint_raw/attitude"),
        ],
        parameters=[px4ctrl_params],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "odom_topic",
            default_value="/nmpc_bridge/odom_world",
            description="Odometry source for px4ctrl: world-frame corrected "
                        "mavros odom (sim) or /ekf_quat/ekf_odom (real)",
        ),
        c_nmpc_node,
        nmpc_bridge_node,
        flight_mux_node,
        px4ctrl_node,
    ])
