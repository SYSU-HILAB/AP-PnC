"""
NMPC Visualization Publisher

This module provides a pure visualization publisher for NMPC simulation data.
Unlike the orchestrator, this class only handles ROS message publishing and
does not manage simulation or controller logic.

Topics Published:
    - /nmpc/odometry: Odometry - Position, velocity, orientation from simulator
    - /nmpc/aero_forces: Vector3 - Aerodynamic force vector (body frame)
    - /nmpc/reference_path: Path - Reference trajectory from CSV (static)
    - /nmpc/tracking_path: Path - Actual path from simulator (rolling buffer)
    - /nmpc/motor_thrusts: Float32MultiArray - Motor thrust commands [N]

Dependencies:
    - feature.utils.ros_visualizer: ROS message conversion utilities

Usage:
    # As part of orchestrator
    visualizer = NmpcVisualizer(node)
    visualizer.publish_odometry(state, stamp)

    # Or standalone with any ROS node
    node = Node("my_node")
    visualizer = NmpcVisualizer(node)
"""

from __future__ import annotations

from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

from ...utils.ros_viz_utils import (
    make_topic,
    compute_reference_sample_times,
    numpy_to_point,
    numpy_to_vector3,
    numpy_to_quaternion_wxyz,
    create_path_message,
    add_pose_to_path,
    truncate_path,
    create_reliable_qos,
    create_transient_local_qos,
)


class NmpcVisualizer:
    """
    Pure visualization publisher for NMPC simulation data.

    This class handles ROS message publishing for NMPC simulation
    without any simulation or controller logic.
    """

    def __init__(self, node: Node):
        """
        Initialize the NMPC visualizer.

        Args:
            node: ROS2 node instance to create publishers from
        """
        self.node = node

        # Create publishers with shared QoS
        qos_profile = create_reliable_qos(depth=10)
        qos_transient_local = create_transient_local_qos(depth=10)

        self.odom_pub = node.create_publisher(
            Odometry, make_topic("odometry"), qos_profile
        )
        self.aero_pub = node.create_publisher(
            Vector3, make_topic("aero_forces"), qos_profile
        )
        self.ref_path_pub = node.create_publisher(
            Path, make_topic("reference_path"), qos_transient_local
        )
        self.track_path_pub = node.create_publisher(
            Path, make_topic("tracking_path"), qos_profile
        )
        self.motor_pub = node.create_publisher(
            Float32MultiArray, make_topic("motor_thrusts"), 10
        )

        # Initialize tracking path
        self.tracking_path = create_path_message(frame_id="world")

        node.get_logger().debug("NmpcVisualizer initialized")

    def publish_odometry(self, state: np.ndarray, stamp) -> None:
        """
        Publish odometry message from simulation state.

        Args:
            state: 18D state vector [pos(3), vel(3), rot(3), quat(4), motor(4), thrust_rate(1)]
            stamp: ROS timestamp for the message header
        """
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position = numpy_to_point(state[0:3])

        # Orientation (WXYZ format from state[9:13])
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(state[9:13])

        # Velocity
        odom.twist.twist.linear = numpy_to_vector3(state[3:6])

        # Angular rates
        odom.twist.twist.angular = numpy_to_vector3(state[6:9])

        self.odom_pub.publish(odom)

    def publish_aero_data(self, aero_data: dict, stamp) -> None:
        """
        Publish aerodynamic forces as Vector3.

        Args:
            aero_data: Dictionary containing 'forces_body' key with force vector
            stamp: ROS timestamp for the message header
        """
        forces = numpy_to_vector3(aero_data['forces_body'])
        self.aero_pub.publish(forces)

    def publish_motor_thrusts(self, state: np.ndarray, stamp) -> None:
        """
        Publish motor thrust commands.

        Args:
            state: 18D state vector containing motor commands at indices 13:17
            stamp: ROS timestamp for the message header
        """
        motor_msg = Float32MultiArray()
        motor_msg.data = state[13:17].tolist()
        self.motor_pub.publish(motor_msg)

    def publish_tracking_path(self, state: np.ndarray, stamp) -> None:
        """
        Publish tracking path with rolling buffer.

        Args:
            state: 18D state vector containing position at indices 0:3 and quaternion at 9:13
            stamp: ROS timestamp for the message header
        """
        add_pose_to_path(
            self.tracking_path,
            state[0:3],
            state[9:13],
            frame_id="world",
            stamp=stamp,
        )

        # Keep path size manageable
        truncate_path(self.tracking_path, max_poses=500)

        self.track_path_pub.publish(self.tracking_path)

    def publish_reference_path(self, trajectory, node: Node) -> None:
        """
        Publish the static reference trajectory as a Path message.

        Args:
            trajectory: TrajectoryReference instance with reference data
            node: ROS2 node instance for logging and timestamp
        """
        ref_path = create_path_message(frame_id="world")
        ref_path.header.stamp = node.get_clock().now().to_msg()

        sample_times = compute_reference_sample_times(
            trajectory, sample_count=400
        )

        for t in sample_times:
            ref = trajectory.get_reference_at_time(t)
            add_pose_to_path(
                ref_path,
                ref['position'],
                ref.get('quaternion'),
                frame_id="world",
            )

        self.ref_path_pub.publish(ref_path)
        node.get_logger().info("Reference trajectory published")
