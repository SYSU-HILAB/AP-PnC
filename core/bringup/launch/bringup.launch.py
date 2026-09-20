"""Core control stack launch (c_nmpc) — research code container.

c_nmpc reads its configuration from the conventional cwd-relative
bringup/config/c_nmpc.yaml; no ROS parameters are passed here.

The base services (mavros, px4ctrl, LIO) run in the nav_infra container;
c_nmpc consumes /cur_state and feeds /mpc_controls over the shared ROS 2
graph. px4ctrl (nav_infra) owns arming/offboard and the PASS_THROUGH
handover: /px4ctrl/toggle_pass_through switches its remote rates source
(internal NMPC controls or an external stream).
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
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DISABLE_LOANED_MESSAGES": "0",
            "RMW_FASTRTPS_USE_QOS_FROM_XML": "1",
            "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_xml,
        },
        output="screen",
    )

    return launch.LaunchDescription([
        c_nmpc_node,
    ])
