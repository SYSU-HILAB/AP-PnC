"""
Reference Trajectory Visualization Node

This node publishes the reference trajectory from a CSV file for visualization
in RViz2. Unlike the simulation_visualizer, this node does NOT run the nMPC
controller or simulator - it directly replays the CSV trajectory.

Data Sources:
    - Trajectory CSV: Loaded via trajectory_loader module
    - No controller or simulator used (lightweight alternative to simulation_visualizer)

Topics Published:
    - /nmpc/reference_path: Path - Full reference trajectory from CSV (static)
    - /nmpc/odometry: Odometry - Reference state replayed at playback_dt
    - /nmpc/tracking_path: Path - Rolling buffer of replayed reference states

Dependencies:
    - feature.lu_mpc.controller.trajectory_loader: CSV trajectory loading
    - feature.utils.ros_visualizer: ROS message conversion utilities

Parameters:
    - trajectory_csv: str - Path to trajectory CSV file
    - playback_dt: float - Time step for replaying the trajectory (default: 0.05)
    - real_time_factor: float - Speed multiplier (default: 1.0)
    - loop_playback: bool - Whether to loop the trajectory (default: True)
    - real_offset: np.ndarray - Position offset for real_quat path [x, y, z]
                            Default: [50.0, 0.0, 0.0]

Key Functionality:
    - Lightweight CSV replay without controller/simulator overhead
    - Supports pause/resume via toggle_pause() method
    - Continuously publishes reference paths for late-joining RViz subscribers
    - Publishes both df_quat and real_quat reference paths (offset)
    - Publishes both df_quat and real_quat odometry (for evolution visualization)
    - Differs from simulation_visualizer.py: No controller, no simulator, pure CSV replay
"""

from __future__ import annotations

from typing import Tuple
import rclpy
import numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from feature.lu_mpc.controller.trajectory_loader import (
    TrajectoryReference,
    load_trajectory_csv,
)
from feature.utils.ros_visualizer import (
    make_topic,
    compute_reference_sample_times,
    numpy_to_point,
    numpy_to_vector3,
    numpy_to_quaternion_wxyz,
    create_path_message,
    add_pose_to_path,
    truncate_path,
    create_reliable_qos,
)


def advance_playback_time(
    current_time: float,
    step: float,
    start_time: float,
    end_time: float,
    loop_playback: bool,
) -> Tuple[float, bool]:
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
        real_offset: np.ndarray | None = None,
    ):
        """
        Initialize the reference visualizer node.

        Args:
            trajectory_csv: Path to trajectory CSV file
            playback_dt: Time step for replaying the trajectory in seconds
            real_time_factor: Speed multiplier (1.0 = real-time)
            loop_playback: Whether to loop the trajectory playback
            real_offset: Position offset for real_quat visualization [x, y, z]
                        Default: [50.0, 0.0, 0.0]
        """
        super().__init__("reference_visualizer")

        if real_time_factor <= 0:
            raise ValueError("real_time_factor must be > 0")

        self.playback_dt = playback_dt
        self.loop_playback = loop_playback
        self.paused = False
        self.real_offset = real_offset if real_offset is not None else np.array([50.0, 0.0, 0.0])

        # Load trajectory
        trajectory = load_trajectory_csv(trajectory_csv)
        self.trajectory = TrajectoryReference(trajectory)
        self.start_time = float(self.trajectory.time[0])
        self.end_time = float(self.trajectory.time[-1])
        self.current_time = self.start_time

        # Create publishers
        qos_profile = create_reliable_qos(depth=10)

        # Reference path publishers (static)
        self.path_pub = self.create_publisher(
            Path, make_topic("reference_path"), qos_profile
        )
        self.real_path_pub = self.create_publisher(
            Path, make_topic("real_reference_path"), qos_profile
        )

        # Odometry publishers (evolution)
        self.odom_pub = self.create_publisher(
            Odometry, make_topic("odometry"), qos_profile
        )
        self.real_odom_pub = self.create_publisher(
            Odometry, make_topic("real_odometry"), qos_profile
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

    def _publish_reference_path(self) -> None:
        """Publish both df_quat and real_quat reference trajectories."""
        # DF quaternion path (existing)
        msg_df = create_path_message(frame_id="world")
        msg_df.header.stamp = self.get_clock().now().to_msg()

        # Real quaternion path (new)
        msg_real = create_path_message(frame_id="world")
        msg_real.header.stamp = self.get_clock().now().to_msg()

        sample_times = compute_reference_sample_times(
            self.trajectory, sample_count=400
        )

        has_real_quat = False
        for t in sample_times:
            ref = self.trajectory.get_reference_at_time(t)

            # DF quaternion path
            add_pose_to_path(
                msg_df,
                ref["position"],
                ref.get("quaternion"),
                frame_id="world",
            )

            # Real quaternion path (with offset)
            if ref.get("real_quat") is not None:
                has_real_quat = True
                real_pos = ref["position"] + self.real_offset
                add_pose_to_path(
                    msg_real,
                    real_pos,
                    ref["real_quat"],
                    frame_id="world",
                )

        self.reference_path_msg = msg_df
        self.path_pub.publish(msg_df)

        if has_real_quat:
            self.real_reference_path_msg = msg_real
            self.real_path_pub.publish(msg_real)
            self.get_logger().info("Reference trajectories published (df_quat and real_quat)")
        else:
            self.get_logger().warn("Reference trajectory published (df_quat only, real_quat not available)")

    def _timer_cb(self) -> None:
        """Timer callback for trajectory replay."""
        # Continuously publish reference paths for late subscribers
        if hasattr(self, "reference_path_msg"):
            self.path_pub.publish(self.reference_path_msg)
        if hasattr(self, "real_reference_path_msg"):
            self.real_path_pub.publish(self.real_reference_path_msg)

        if self.paused:
            return

        # Get reference state at current time
        ref = self.trajectory.get_reference_at_time(self.current_time)
        stamp = self.get_clock().now().to_msg()

        # Publish odometry (both df_quat and real_quat)
        self._publish_odometry(ref, stamp)
        self._publish_real_odometry(ref, stamp)

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

    def _publish_odometry(self, ref: dict, stamp) -> None:
        """Publish odometry from reference state."""
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "reference"
        odom.header.stamp = stamp

        odom.pose.pose.position = numpy_to_point(ref["position"])
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(
            ref.get("quaternion")
        )
        odom.twist.twist.linear = numpy_to_vector3(ref["velocity"])

        self.odom_pub.publish(odom)

    def _publish_real_odometry(self, ref: dict, stamp) -> None:
        """Publish odometry from real_quat reference state."""
        if ref.get("real_quat") is None:
            return

        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "real_reference"
        odom.header.stamp = stamp

        # Apply position offset
        real_pos = ref["position"] + self.real_offset
        odom.pose.pose.position = numpy_to_point(real_pos)
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(ref["real_quat"])
        odom.twist.twist.linear = numpy_to_vector3(ref["velocity"])

        self.real_odom_pub.publish(odom)


def main(args=None):
    """Main entry point for reference visualizer."""
    rclpy.init(args=args)

    # Example: use trajectory from command line or default
    import sys
    if len(sys.argv) > 1:
        traj_csv = sys.argv[1]
    else:
        from pathlib import Path
        ros_dir = Path(__file__).parent
        traj_csv = str(ros_dir.parent / 'controller' / 'refs' / 'v14-alpha.csv')

    node = ReferenceVisualizer(
        trajectory_csv=traj_csv,
        playback_dt=0.05,
        real_time_factor=1.0,
        loop_playback=True,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
