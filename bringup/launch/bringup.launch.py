import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    c_nmpc_config = os.path.join(bringup_share, "config", "c_nmpc.yaml")
    px4_interface_config = os.path.join(bringup_share, "config", "px4_interface.yaml")
    fastdds_xml = os.path.join(bringup_share, "config", "fastdds.xml")

    c_nmpc_node = launch_ros.actions.Node(
        package="c_nmpc",
        executable="c_nmpc_node",
        name="c_nmpc_node",
        parameters=[c_nmpc_config],
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
        parameters=[px4_interface_config],
        output="screen",
    )

    return launch.LaunchDescription([
        c_nmpc_node,
        px4_interface_node,
    ])
