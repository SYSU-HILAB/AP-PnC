"""Control stack launch (c_nmpc + px4_interface).

Nodes read their configuration directly from bringup/config yaml files
($AP_PNC_NMPC_YAML override, cwd-relative fallback); no ROS parameters are
passed here.
"""

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    fastdds_xml = os.path.join(bringup_share, "config", "fastdds.xml")

    c_nmpc_node = launch_ros.actions.Node(
        package="c_nmpc",
        executable="c_nmpc_node",
        name="c_nmpc_node",
        environment={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DISABLE_LOANED_MESSAGES": "0",
            "RMW_FASTRTPS_USE_QOS_FROM_XML": "1",
            "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_xml,
        },
        output="screen",
    )

    px4_interface_node = launch_ros.actions.Node(
        package="px4_interface",
        executable="px4_interface",
        name="px4_interface",
        output="screen",
    )

    return launch.LaunchDescription([
        c_nmpc_node,
        px4_interface_node,
    ])
