# -*- coding: future_fstrings -*-
"""
Reference trajectory visualization node.

Publishes the static reference path plus a time-evolving state generated directly
from the CSV reference without running the MPC controller or simulator.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from feature.lu_mpc.controller.trajectory_loader import (
    TrajectoryReference,
    load_trajectory_csv,
)
from feature.lu_mpc.ros.viz_node import compute_reference_sample_times, make_topic


def advance_playback_time(
    current_time: float,
    step: float,
    start_time: float,
    end_time: float,
    loop_playback: bool,
) -> Tuple[float, bool]:
    """
    Advance the playback cursor, optionally looping back to the start.

    Returns:
        next_time: The updated playback time.
        wrapped: True if the cursor wrapped around to the start.
    """
    next_time = current_time + step

    if next_time > end_time:
        if loop_playback:
            overflow = next_time - end_time
            return start_time + overflow, True
        return end_time, False

    return next_time, False


def _numpy_to_point(position: np.ndarray) -> Point:
    point = Point()
    point.x = float(position[0])
    point.y = float(position[1])
    point.z = float(position[2])
    return point


def _numpy_to_vector3(vec: np.ndarray) -> Vector3:
    vector = Vector3()
    vector.x = float(vec[0])
    vector.y = float(vec[1])
    vector.z = float(vec[2])
    return vector


def _identity_quaternion() -> Quaternion:
    quat = Quaternion()
    quat.w = 1.0
    return quat


class ReferencePathNode(Node):
    """
    Lightweight ROS2 node that replays the reference CSV on visualization topics.

    Topics:
        /nmpc/reference_path: Static Path built from the CSV.
        /nmpc/odometry: Reference state replayed as Odometry (no controller).
        /nmpc/tracking_path: Rolling path built from the reference playback.
    """

    def __init__(
        self,
        trajectory_csv: str,
        playback_dt: float = 0.05,
        real_time_factor: float = 1.0,
        loop_playback: bool = True,
    ):
        super().__init__("reference_path_node")

        if real_time_factor <= 0:
            raise ValueError("real_time_factor must be > 0")

        self.playback_dt = playback_dt
        self.loop_playback = loop_playback
        self.paused = False

        trajectory = load_trajectory_csv(trajectory_csv)
        self.trajectory = TrajectoryReference(trajectory)
        self.start_time = float(self.trajectory.time[0])
        self.end_time = float(self.trajectory.time[-1])
        self.current_time = self.start_time

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.path_pub = self.create_publisher(Path, make_topic("reference_path"), qos_profile)
        self.track_path_pub = self.create_publisher(Path, make_topic("tracking_path"), qos_profile)
        self.odom_pub = self.create_publisher(Odometry, make_topic("odometry"), qos_profile)

        self.tracking_path = Path()
        self.tracking_path.header.frame_id = "world"

        self._publish_reference_path()

        self.timer = self.create_timer(self.playback_dt / real_time_factor, self._timer_cb)
        self.get_logger().info("ReferencePathNode ready")

    def toggle_pause(self) -> bool:
        """Toggle playback pause state, returning the new state."""
        self.paused = not self.paused
        state = "paused" if self.paused else "playing"
        self.get_logger().info(f"Reference playback {state}")
        return self.paused

    def _publish_reference_path(self) -> None:
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        sample_times = compute_reference_sample_times(self.trajectory, sample_count=400)
        for t in sample_times:
            ref = self.trajectory.get_reference_at_time(t)
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position = _numpy_to_point(ref["position"])
            pose.pose.orientation = _identity_quaternion()
            msg.poses.append(pose)
        self.reference_path_msg = msg
        self.path_pub.publish(msg)

    def _timer_cb(self) -> None:
        # Continuously publish the reference path so late subscribers (e.g., RViz) can render it.
        if hasattr(self, "reference_path_msg"):
            self.path_pub.publish(self.reference_path_msg)

        if self.paused:
            return

        ref = self.trajectory.get_reference_at_time(self.current_time)
        stamp = self.get_clock().now().to_msg()

        self._publish_odometry(ref, stamp)
        self._publish_tracking_path(ref, stamp)

        next_time, wrapped = advance_playback_time(
            self.current_time, self.playback_dt, self.start_time, self.end_time, self.loop_playback
        )

        if self.loop_playback and wrapped:
            self.tracking_path.poses.clear()
            self.current_time = next_time
            return

        if not self.loop_playback and next_time >= self.end_time:
            self.current_time = self.end_time
            self.get_logger().info("Reference playback finished, stopping timer.")
            self.timer.cancel()
            return

        self.current_time = next_time

    def _publish_odometry(self, ref: dict, stamp) -> None:
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "reference"
        odom.header.stamp = stamp
        odom.pose.pose.position = _numpy_to_point(ref["position"])
        odom.pose.pose.orientation = _identity_quaternion()
        odom.twist.twist.linear = _numpy_to_vector3(ref["velocity"])
        self.odom_pub.publish(odom)

    def _publish_tracking_path(self, ref: dict, stamp) -> None:
        pose = PoseStamped()
        pose.header = self.tracking_path.header
        pose.header.stamp = stamp
        pose.pose.position = _numpy_to_point(ref["position"])
        pose.pose.orientation = _identity_quaternion()
        self.tracking_path.poses.append(pose)

        if len(self.tracking_path.poses) > 400:
            self.tracking_path.poses.pop(0)

        self.track_path_pub.publish(self.tracking_path)
