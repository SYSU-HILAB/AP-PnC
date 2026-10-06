"""nav_infra launch: the base-services gateway container stack.

    ros2 launch bringup nav_infra.launch.py profile:=sim
    ros2 launch bringup nav_infra.launch.py profile:=real

Runs inside the nav_infra container (not research code):
  sim  : mavros2 (PX4 SITL over UDP 14540/14580) + px4ctrl; odometry from
         /mavros/local_position/odom (PX4 EKF2).
  real : + livox_ros_driver2 (Mid-360) + fast_lio + ekf_quat;
         px4ctrl odometry from /ekf_quat/ekf_odom.

px4ctrl owns arming/offboard, is the single /mavros/setpoint_raw/attitude
publisher, and hosts both controller modes (tracking / internal NMPC+INDI)
plus the odometry state adapter (/px4ctrl/state, /px4ctrl/state_odom,
/px4ctrl/odom_world). The core container (planner + nmpc) consumes
/px4ctrl/state and feeds /nmpc/control + /setpoint_cmd over the shared ROS 2
graph.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# This mavros build creates plugin topics under a child node
# (/mavros/mavros/*); remap the consumed ones to canonical /mavros/...
# (pattern from the agent_am_demo L0 gateway).
IMU_REMAPPINGS = [
    ("/mavros/mavros/data", "/mavros/imu/data"),
    ("/mavros/mavros/data_raw", "/mavros/imu/data_raw"),
    ("/mavros/mavros/attitude", "/mavros/imu/attitude"),
]


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    nav_config_dir = os.path.join(bringup_share, "config", "nav")

    profile = LaunchConfiguration("profile")
    is_sim = PythonExpression(["'", profile, "'.lower() == 'sim'"])
    is_real = PythonExpression(["'", profile, "'.lower() == 'real'"])

    mavros_sim = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        remappings=IMU_REMAPPINGS,
        parameters=[os.path.join(nav_config_dir, "mavros_sim.yaml")],
        condition=IfCondition(is_sim),
    )

    mavros_real = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        remappings=IMU_REMAPPINGS,
        parameters=[os.path.join(nav_config_dir, "mavros_real.yaml")],
        condition=IfCondition(is_real),
    )

    # px4ctrl: L2 control + FCU interface (arming/offboard, single mavros
    # setpoint publisher). ctrl_mode / throttle_estimator select the
    # controller implementation and the throttle estimation law.
    px4ctrl_params = PythonExpression(
        ["'", bringup_share, "/config/px4ctrl' + ('' if '", profile,
         "'.lower() == 'sim' else '_real') + '.yaml'"])
    px4ctrl_odom = PythonExpression(
        ["'/mavros/local_position/odom' if '", profile,
         "'.lower() == 'sim' else '/ekf_quat/ekf_odom'"])
    px4ctrl = Node(
        package="px4ctrl",
        executable="px4ctrl_node",
        name="px4ctrl",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("odom", px4ctrl_odom),
            ("cmd", "/setpoint_cmd"),
            ("takeoff_land", "/px4ctrl/takeoff_land"),
        ],
        parameters=[px4ctrl_params],
    )

    # mavros 2.15 plugin sub-nodes are created with use_global_arguments(false)
    # (mavros/src/lib/plugin.cpp), so the params file never reaches
    # /mavros/setpoint_raw. Its declared thrust_scaling default is NaN, which
    # makes the plugin ignore every AttitudeTarget with thrust != 0 - PX4 then
    # never sees an offboard signal and refuses OFFBOARD/arming. Set it as
    # soon as the sub-node appears (bounded retry, ~2 min).
    set_thrust_scaling = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "for _ in $(seq 1 240); do "
            "ros2 param set /mavros/setpoint_raw thrust_scaling 1.0 "
            ">/dev/null 2>&1 && exit 0; sleep 0.5; done; "
            "echo 'nav_infra: failed to set setpoint_raw.thrust_scaling' >&2; "
            "exit 1",
        ],
        output="log",
    )

    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            # 1 = livox CustomMsg (FAST-LIO mid360 profile expects it).
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": os.path.join(nav_config_dir,
                                             "mid360_config.json"),
        }],
        condition=IfCondition(is_real),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[os.path.join(nav_config_dir, "fastlio_mid360.yaml")],
        condition=IfCondition(is_real),
    )

    ekf_quat = Node(
        package="ekf_quat_pose",
        executable="ekf_quat",
        name="ekf_quat",
        output="screen",
        parameters=[os.path.join(nav_config_dir, "ekf.yaml")],
        remappings=[
            ("imu", "/mavros/imu/data"),
            ("bodyodometry", "/Odometry"),
            ("ekf_odom", "/ekf_quat/ekf_odom"),
        ],
        condition=IfCondition(is_real),
    )

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="sim",
                              description="sim | real"),
        mavros_sim,
        mavros_real,
        px4ctrl,
        set_thrust_scaling,
        livox,
        fast_lio,
        ekf_quat,
    ])
