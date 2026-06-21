"""
Reference Trajectory Visualization Node

This node publishes the reference trajectory from a CSV file for visualization
in RViz2. Unlike the nmpc_orchestrator, this node does NOT run the NMPC
controller or simulator - it directly replays the CSV trajectory.

Data Sources:
    - Trajectory CSV: Loaded via trajectory_loader module
    - No controller or simulator used (lightweight alternative to nmpc_orchestrator)

Topics Published:
    - /ref_traj_df_phi: Path - Full df_phi_quat reference trajectory from CSV (static)
    - /ref_traj_df_lyu: Path - Full df_lyu_quat reference trajectory from CSV (static, offset)
    - /ref_odom_df_phi: Odometry - df_phi_quat reference state replayed at playback_dt
    - /ref_odom_df_lyu: Odometry - df_lyu_quat reference state replayed at playback_dt (offset)

Dependencies:
    - feature.lu_mpc.controller.trajectory_loader: CSV trajectory loading
    - feature.utils.ros_visualizer: ROS message conversion utilities

Parameters:
    - trajectory_csv: str - Path to trajectory CSV file
    - playback_dt: float - Time step for replaying the trajectory (default: 0.05)
    - real_time_factor: float - Speed multiplier (default: 1.0)
    - loop_playback: bool - Whether to loop the trajectory (default: True)
    - df_lyu_offset: np.ndarray - Position offset for df_lyu_quat path [x, y, z]
                            Default: [80.0, 0.0, 0.0]

Key Functionality:
    - Lightweight CSV replay without controller/simulator overhead
    - Supports pause/resume via toggle_pause() method
    - Continuously publishes reference paths for late-joining RViz subscribers
    - Publishes both df_phi_quat and df_lyu_quat reference paths (offset)
    - Publishes both df_phi_quat and df_lyu_quat odometry (for evolution visualization)
    - Differs from nmpc_orchestrator.py: No controller, no simulator, pure CSV replay
"""

from __future__ import annotations

import numpy as np
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ...utils.math_utils import wing_frd_to_quad_flu
from ...utils.ros_viz_utils import (
    add_pose_to_path,
    compute_reference_sample_times,
    create_path_message,
    create_reliable_qos,
    numpy_to_point,
    numpy_to_quaternion_wxyz,
    numpy_to_vector3,
)
from ..controller.trajectory_loader import TrajectoryReference


def advance_playback_time(
    current_time: float,
    step: float,
    start_time: float,
    end_time: float,
    loop_playback: bool,
) -> tuple[float, bool]:
    """
    Advance the playback cursor, optionally looping back to the start.

    Args:
        current_time: Current playback time in seconds
        step: Time step to advance
        start_time: Trajectory start time
        end_time: Trajectory end time
        loop_playback: Whether to loop back to start when reaching end

    Returns:
        (next_time, wrapped) tuple where wrapped is True if cursor looped
    """
    next_time = current_time + step

    if next_time > end_time:
        if loop_playback:
            overflow = next_time - end_time
            return start_time + overflow, True
        return end_time, False

    return next_time, False


class ReferenceVisualizer(Node):
    """
    Lightweight ROS2 node that replays the reference CSV on visualization topics.

    This node provides a simple way to visualize the planned trajectory without
    running the full controller and simulator.
    """

    def __init__(
        self,
        trajectory_csv: str,
        playback_dt: float = 0.05,
        real_time_factor: float = 1.0,
        loop_playback: bool = True,
        df_lyu_offset: np.ndarray | None = None,
    ):
        """
        Initialize the reference visualizer node.

        Args:
            trajectory_csv: Path to trajectory CSV file
            playback_dt: Time step for replaying the trajectory in seconds
            real_time_factor: Speed multiplier (1.0 = real-time)
            loop_playback: Whether to loop the trajectory playback
            df_lyu_offset: Position offset for df_lyu_quat visualization [x, y, z]
                          Default: [80.0, 0.0, 0.0]
        """
        super().__init__("reference_visualizer")

        if real_time_factor <= 0:
            raise ValueError("real_time_factor must be > 0")

        self.playback_dt = playback_dt
        self.loop_playback = loop_playback
        self.paused = False
        self.df_lyu_offset = df_lyu_offset if df_lyu_offset is not None else np.array([80.0, 0.0, 0.0])

        # Load trajectory (validation happens automatically inside TrajectoryReference)
        self.trajectory = TrajectoryReference(trajectory_csv)
        self.start_time = float(self.trajectory.time[0])
        self.end_time = float(self.trajectory.time[-1])
        self.current_time = self.start_time

        # Create publishers with explicit topic names (no /nmpc/ prefix)
        qos_profile = create_reliable_qos(depth=10)

        # Reference path publishers need TRANSIENT_LOCAL for late subscribers (RViz)
        qos_transient_local = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Reference path publishers (static) - df_phi_quat vs df_lyu_quat
        self.df_phi_path_pub = self.create_publisher(
            Path, "/ref_traj_df_phi", qos_transient_local
        )
        self.df_lyu_path_pub = self.create_publisher(
            Path, "/ref_traj_df_lyu", qos_transient_local
        )

        # Odometry publishers (evolution) - df_phi_quat vs df_lyu_quat
        self.df_phi_odom_pub = self.create_publisher(
            Odometry, "/ref_odom_df_phi", qos_profile
        )
        self.df_lyu_odom_pub = self.create_publisher(
            Odometry, "/ref_odom_df_lyu", qos_profile
        )

        # Publish static reference path
        self._publish_reference_path()

        # Setup timer for playback
        self.timer = self.create_timer(
            self.playback_dt / real_time_factor, self._timer_cb
        )

        self.get_logger().info("ReferenceVisualizer ready")
        self.get_logger().info(f"  Trajectory CSV: {trajectory_csv}")
        self.get_logger().info(f"  Playback dt: {playback_dt}s")
        self.get_logger().info(f"  Real-time factor: {real_time_factor}x")
        self.get_logger().info(f"  Loop playback: {loop_playback}")

    def toggle_pause(self) -> bool:
        """
        Toggle playback pause state.

        Returns:
            New pause state (True if paused, False if playing)
        """
        self.paused = not self.paused
        state = "paused" if self.paused else "playing"
        self.get_logger().info(f"Reference playback {state}")
        return self.paused

    def _get_reference_at_time_index(self, t: float) -> int:
        """
        Convert time to trajectory index (no interpolation).

        Args:
            t: Time in seconds

        Returns:
            Index in trajectory arrays
        """
        sim_dt = 0.02  # Trajectory sampling rate
        idx = int(np.round(t / sim_dt))
        return max(0, min(idx, len(self.trajectory.time) - 1))

    def _get_reference_single(self, idx: int) -> dict:
        """
        Get single reference state at index (used for visualization).

        This is a convenience wrapper for visualization that samples
        a single element from trajectory arrays.

        Args:
            idx: Index in trajectory

        Returns:
            Dictionary with single reference state
        """
        ref = {
            "position": self.trajectory.position[idx].copy(),
            "velocity": self.trajectory.velocity[idx].copy(),
            "yb": self.trajectory.yb[idx].copy(),
            "alpha": float(self.trajectory.alpha[idx]),
        }

        if self.trajectory.quat_quad_flu is not None:
            ref["quat_quad_flu"] = self.trajectory.quat_quad_flu[idx].copy()
        if self.trajectory.df_phi_quat is not None:
            ref["df_phi_quat"] = self.trajectory.df_phi_quat[idx].copy()
        if self.trajectory.df_lyu_quat is not None:
            ref["df_lyu_quat"] = self.trajectory.df_lyu_quat[idx].copy()

        return ref

    def _publish_reference_path(self) -> None:
        """Publish both df_phi_quat and df_lyu_quat reference trajectories."""
        # df_phi quaternion path
        msg_df_phi = create_path_message(frame_id="world")
        msg_df_phi.header.stamp = self.get_clock().now().to_msg()

        # df_lyu quaternion path (offset)
        msg_df_lyu = create_path_message(frame_id="world")
        msg_df_lyu.header.stamp = self.get_clock().now().to_msg()

        sample_times = compute_reference_sample_times(
            self.trajectory, sample_count=400
        )

        for t in sample_times:
            idx = self._get_reference_at_time_index(t)
            ref = self._get_reference_single(idx)

            # DF phi quaternion path
            df_phi_quat = self._compose_reference_quat(ref, "df_phi_quat")
            add_pose_to_path(
                msg_df_phi,
                ref["position"],
                df_phi_quat,
                frame_id="world",
            )

            # DF lyu quaternion path (with offset)
            df_lyu_pos = ref["position"] + self.df_lyu_offset
            df_lyu_quat = self._compose_reference_quat(ref, "df_lyu_quat")
            add_pose_to_path(
                msg_df_lyu,
                df_lyu_pos,
                df_lyu_quat,
                frame_id="world",
            )

        self.df_phi_reference_path_msg = msg_df_phi
        self.df_phi_path_pub.publish(msg_df_phi)
        self.df_lyu_reference_path_msg = msg_df_lyu
        self.df_lyu_path_pub.publish(msg_df_lyu)
        self.get_logger().info("Reference trajectories published (df_phi_quat and df_lyu_quat)")

    def _timer_cb(self) -> None:
        """Timer callback for trajectory replay."""
        # Continuously publish reference paths for late subscribers
        if hasattr(self, "df_phi_reference_path_msg"):
            self.df_phi_path_pub.publish(self.df_phi_reference_path_msg)
        if hasattr(self, "df_lyu_reference_path_msg"):
            self.df_lyu_path_pub.publish(self.df_lyu_reference_path_msg)

        if self.paused:
            return

        # Get reference state at current time
        idx = self._get_reference_at_time_index(self.current_time)
        ref = self._get_reference_single(idx)
        stamp = self.get_clock().now().to_msg()

        # Publish odometry (both df_phi_quat and df_lyu_quat)
        self._publish_df_phi_odometry(ref, stamp)
        self._publish_df_lyu_odometry(ref, stamp)

        # Advance playback time
        next_time, wrapped = advance_playback_time(
            self.current_time,
            self.playback_dt,
            self.start_time,
            self.end_time,
            self.loop_playback,
        )

        # Handle looping or termination
        if self.loop_playback and wrapped:
            self.current_time = next_time
            return

        if not self.loop_playback and next_time >= self.end_time:
            self.current_time = self.end_time
            self.get_logger().info("Reference playback finished")
            self.timer.cancel()
            return

        self.current_time = next_time

    def _publish_df_phi_odometry(self, ref: dict, stamp) -> None:
        """Publish odometry from df_phi_quat reference state."""
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "df_phi_reference"
        odom.header.stamp = stamp

        odom.pose.pose.position = numpy_to_point(ref["position"])
        quat = self._compose_reference_quat(ref, "df_phi_quat")
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(quat)
        odom.twist.twist.linear = numpy_to_vector3(ref["velocity"])

        self.df_phi_odom_pub.publish(odom)

    def _publish_df_lyu_odometry(self, ref: dict, stamp) -> None:
        """Publish odometry from df_lyu_quat reference state."""
        df_lyu_quat = self._compose_reference_quat(ref, "df_lyu_quat")
        if df_lyu_quat is None:
            raise RuntimeError("df_lyu_quat is missing from reference state")

        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "df_lyu_reference"
        odom.header.stamp = stamp

        # Apply position offset
        df_lyu_pos = ref["position"] + self.df_lyu_offset
        odom.pose.pose.position = numpy_to_point(df_lyu_pos)
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(df_lyu_quat)
        odom.twist.twist.linear = numpy_to_vector3(ref["velocity"])

        self.df_lyu_odom_pub.publish(odom)

    @staticmethod
    def _compose_reference_quat(ref: dict, key: str):
        """
        Compose CSV quaternion with odometry quaternion for visualization.

        df_phi/df_lyu quats are stored as offsets relative to the odom frame.
        """
        quat_csv = ref.get(key)
        if quat_csv is None:
            raise RuntimeError(f"{key} is missing from reference state")

        return wing_frd_to_quad_flu(quat_csv)
