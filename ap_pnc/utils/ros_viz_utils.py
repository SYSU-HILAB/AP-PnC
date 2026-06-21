"""
ROS2 visualization utilities shared across feature modules.

Provides:
- ROS message conversion helpers (NumPy → ROS messages)
- Path creation and management utilities
- QoS profile factory functions
- Topic naming utilities
- Launch helpers for ROS executor management

Quaternion Format Standard:
    All quaternion conversions use WXYZ format [w, x, y, z] where w is the
    scalar part. This matches:
    - The CSV trajectory loader format (quat_w, quat_x, quat_y, quat_z)
    - The mathematical convention in ap_pnc/utils/math_utils.py
    - ROS2 standard Quaternion message ordering
"""

from __future__ import annotations

import time

import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# ==============================================================================
# Topic Naming Utilities
# ==============================================================================

TOPIC_PREFIX = "/nmpc"


def make_topic(name: str) -> str:
    """
    Build a fully-qualified ROS2 topic name under the NMPC namespace.

    Args:
        name: Topic suffix, e.g. "odometry", "reference_path"

    Returns:
        Full topic path, e.g. "/nmpc/odometry"
    """
    clean_prefix = TOPIC_PREFIX if TOPIC_PREFIX.startswith("/") else f"/{TOPIC_PREFIX}"
    return f"{clean_prefix}/{name}".replace("//", "/")


def get_topic_prefix() -> str:
    """
    Get the current topic namespace prefix.

    Returns:
        The topic prefix string (e.g., "/nmpc")
    """
    return TOPIC_PREFIX


# ==============================================================================
# ROS Message Conversion Helpers (WXYZ Standard)
# ==============================================================================

def numpy_to_point(position: np.ndarray) -> Point:
    """
    Convert NumPy position array [x, y, z] to ROS Point message.

    Args:
        position: NumPy array of shape (3,) with x, y, z coordinates

    Returns:
        ROS Point message
    """
    point = Point()
    point.x = float(position[0])
    point.y = float(position[1])
    point.z = float(position[2])
    return point


def numpy_to_vector3(vec: np.ndarray) -> Vector3:
    """
    Convert NumPy array [x, y, z] to ROS Vector3 message.

    Args:
        vec: NumPy array of shape (3,)

    Returns:
        ROS Vector3 message
    """
    v_ros = Vector3()
    v_ros.x = float(vec[0])
    v_ros.y = float(vec[1])
    v_ros.z = float(vec[2])
    return v_ros


def numpy_to_quaternion_wxyz(quat: np.ndarray | None) -> Quaternion:
    """
    Convert NumPy quaternion [w, x, y, z] to ROS Quaternion message.

    **STANDARD:** This function expects and uses WXYZ format [w, x, y, z]
    where w is the scalar part. This matches:
    - The CSV trajectory loader format (quat_w, quat_x, quat_y, quat_z)
    - The mathematical convention in ap_pnc/utils/math_utils.py
    - ROS2 standard Quaternion message ordering

    Args:
        quat: NumPy array of shape (4,) with [w, x, y, z] ordering,
              or None to return identity quaternion

    Returns:
        ROS Quaternion message with WXYZ format
    """
    if quat is None or len(quat) != 4:
        return identity_quaternion()

    msg = Quaternion()
    msg.w = float(quat[0])
    msg.x = float(quat[1])
    msg.y = float(quat[2])
    msg.z = float(quat[3])
    return msg


def identity_quaternion() -> Quaternion:
    """
    Return an identity quaternion [w=1, x=0, y=0, z=0] as ROS message.

    Returns:
        ROS Quaternion message representing identity rotation
    """
    quat = Quaternion()
    quat.w = 1.0
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat


# ==============================================================================
# Path Creation and Management
# ==============================================================================

def create_path_message(frame_id: str = "world") -> Path:
    """
    Create an empty ROS Path message with proper header.

    Args:
        frame_id: TF frame ID for the path (default: "world")

    Returns:
        Empty Path message with initialized header
    """
    path = Path()
    path.header.frame_id = frame_id
    return path


def add_pose_to_path(
    path: Path,
    position: np.ndarray,
    quaternion: np.ndarray | None,
    frame_id: str = "world",
    stamp=None,
) -> None:
    """
    Add a pose to a Path message.

    Args:
        path: Path message to modify
        position: NumPy array [x, y, z]
        quaternion: NumPy array [w, x, y, z] or None for identity
        frame_id: TF frame ID
        stamp: Timestamp (uses current time if None)
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position = numpy_to_point(position)
    pose.pose.orientation = numpy_to_quaternion_wxyz(quaternion)
    path.poses.append(pose)


def truncate_path(path: Path, max_poses: int) -> None:
    """
    Keep path size manageable by removing oldest poses.

    Args:
        path: Path message to truncate
        max_poses: Maximum number of poses to keep
    """
    while len(path.poses) > max_poses:
        path.poses.pop(0)


# ==============================================================================
# Colormap Utilities
# ==============================================================================

def get_colormap(name: str = "jet"):
    """
    Get matplotlib colormap for path visualization.

    Args:
        name: Colormap name ("jet", "plasma", "viridis", "hot", "cool", etc.)
              Default is "jet" for traditional visualization.

    Returns:
        matplotlib colormap object

    Examples:
        >>> cmap = get_colormap("plasma")
        >>> rgba = cmap(0.5)  # Get color at midpoint (0.0 to 1.0)
    """
    try:
        import matplotlib.pyplot as plt
        return plt.get_cmap(name)
    except ImportError:
        # Fallback: return a simple color mapping
        def simple_cmap(value):
            """Simple grayscale fallback if matplotlib is not available."""
            value = max(0.0, min(1.0, value))
            return (value, value, value, 1.0)
        return simple_cmap


def get_colormap_color(name: str, value: float):
    """
    Get a single color from a colormap by normalized value.

    Args:
        name: Colormap name ("jet", "plasma", "viridis", etc.)
        value: Normalized value in range [0.0, 1.0]

    Returns:
        RGBA tuple (red, green, blue, alpha) with values in [0, 1]
    """
    cmap = get_colormap(name)
    return cmap(max(0.0, min(1.0, value)))


# ==============================================================================
# Reference Trajectory Sampling
# ==============================================================================

def compute_reference_sample_times(
    trajectory,  # TrajectoryReference object with .time attribute
    sample_count: int,
    max_duration: float | None = None,
) -> np.ndarray:
    """
    Generate sampling timestamps that respect the CSV's full time-span.

    Args:
        trajectory: TrajectoryReference object with .time attribute
        sample_count: Number of time instants to sample (must be >= 2)
        max_duration: Optional cap (seconds) relative to CSV start time

    Returns:
        Monotonically increasing NumPy array of timestamps

    Raises:
        ValueError: If sample_count < 2
    """
    if sample_count < 2:
        raise ValueError("sample_count must be >= 2 for a Path message")

    time_vector = np.asarray(trajectory.time, dtype=float)
    start_time = float(time_vector[0])
    end_time = float(time_vector[-1])

    if max_duration is not None:
        end_time = min(end_time, start_time + max_duration)

    if end_time <= start_time:
        return np.full(sample_count, start_time, dtype=float)

    return np.linspace(start_time, end_time, sample_count)


# ==============================================================================
# QoS Profile Factory
# ==============================================================================

def create_reliable_qos(depth: int = 10) -> QoSProfile:
    """
    Create a reliable QoS profile for important visualization messages.

    Uses:
    - RELIABLE delivery guarantee
    - KEEP_LAST history policy
    - Specified depth

    Args:
        depth: Queue depth (default: 10)

    Returns:
        Configured QoSProfile
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def create_transient_local_qos(depth: int = 10) -> QoSProfile:
    """
    Create a reliable QoS profile with TRANSIENT_LOCAL durability.

    Useful for static Path messages so late subscribers (RViz) receive data.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def create_best_effort_qos(depth: int = 10) -> QoSProfile:
    """
    Create a best-effort QoS profile for high-frequency messages.

    Args:
        depth: Queue depth (default: 10)

    Returns:
        Configured QoSProfile
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ==============================================================================
# Time Utilities
# ==============================================================================

def sleep_period(duration: float) -> None:
    """
    Sleep for the given duration if positive.

    Separated into a helper so unit tests can patch time.sleep.

    Args:
        duration: Sleep duration in seconds (no-op if <= 0)
    """
    if duration <= 0.0:
        return
    time.sleep(duration)
