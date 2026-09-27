"""
Unit tests for feature.utils.ros_viz_utils module.
"""

from __future__ import annotations

import numpy as np
import pytest
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped
from nav_msgs.msg import Path

from ap_pnc.ros_viz_utils import (
    make_topic,
    get_topic_prefix,
    numpy_to_point,
    numpy_to_vector3,
    numpy_to_quaternion_wxyz,
    identity_quaternion,
    create_path_message,
    add_pose_to_path,
    truncate_path,
    create_reliable_qos,
    create_best_effort_qos,
    compute_reference_sample_times,
    sleep_period,
)


class DummyTrajectory:
    """Fixture-friendly trajectory stub."""

    def __init__(self):
        self.time = np.linspace(2.0, 18.0, 5)


@pytest.fixture
def dummy_traj():
    return DummyTrajectory()


# ==============================================================================
# Topic Naming Tests
# ==============================================================================

def test_make_topic_applies_prefix():
    """Test that make_topic applies the NMPC prefix correctly."""
    assert make_topic("odometry") == "/nmpc/odometry"
    assert make_topic("reference_path") == "/nmpc/reference_path"


def test_make_topic_handles_leading_slash():
    """Test that make_topic normalizes leading slashes."""
    assert make_topic("/test") == "/nmpc/test"


def test_get_topic_prefix():
    """Test that get_topic_prefix returns the correct prefix."""
    assert get_topic_prefix() == "/nmpc"


# ==============================================================================
# ROS Message Conversion Tests
# ==============================================================================

def test_numpy_to_point():
    """Test conversion from NumPy array to ROS Point."""
    arr = np.array([1.0, 2.0, 3.0])
    point = numpy_to_point(arr)

    assert isinstance(point, Point)
    assert point.x == 1.0
    assert point.y == 2.0
    assert point.z == 3.0


def test_numpy_to_vector3():
    """Test conversion from NumPy array to ROS Vector3."""
    arr = np.array([0.5, -1.5, 2.5])
    vec = numpy_to_vector3(arr)

    assert isinstance(vec, Vector3)
    assert vec.x == 0.5
    assert vec.y == -1.5
    assert vec.z == 2.5


def test_numpy_to_quaternion_wxyz():
    """Test WXYZ quaternion conversion."""
    # Standard WXYZ format: [w, x, y, z]
    quat_np = np.array([1.0, 0.0, 0.0, 0.0])  # Identity
    quat_ros = numpy_to_quaternion_wxyz(quat_np)

    assert isinstance(quat_ros, Quaternion)
    assert quat_ros.w == 1.0
    assert quat_ros.x == 0.0
    assert quat_ros.y == 0.0
    assert quat_ros.z == 0.0


def test_numpy_to_quaternion_wxyz_rotation():
    """Test WXYZ conversion with actual rotation."""
    # 90-degree rotation around Z-axis
    quat_np = np.array([np.sqrt(2) / 2, 0.0, 0.0, np.sqrt(2) / 2])
    quat_ros = numpy_to_quaternion_wxyz(quat_np)

    assert quat_ros.w == pytest.approx(np.sqrt(2) / 2)
    assert quat_ros.x == pytest.approx(0.0)
    assert quat_ros.y == pytest.approx(0.0)
    assert quat_ros.z == pytest.approx(np.sqrt(2) / 2)


def test_numpy_to_quaternion_none_returns_identity():
    """Test that None input returns identity quaternion."""
    quat_ros = numpy_to_quaternion_wxyz(None)

    assert quat_ros.w == 1.0
    assert quat_ros.x == 0.0
    assert quat_ros.y == 0.0
    assert quat_ros.z == 0.0


def test_numpy_to_quaternion_invalid_length_returns_identity():
    """Test that invalid length returns identity quaternion."""
    quat_ros = numpy_to_quaternion_wxyz(np.array([1.0, 0.0]))

    assert quat_ros.w == 1.0
    assert quat_ros.x == 0.0
    assert quat_ros.y == 0.0
    assert quat_ros.z == 0.0


def test_identity_quaternion():
    """Test identity quaternion factory."""
    quat = identity_quaternion()

    assert quat.w == 1.0
    assert quat.x == 0.0
    assert quat.y == 0.0
    assert quat.z == 0.0


# ==============================================================================
# Path Management Tests
# ==============================================================================

def test_create_path_message():
    """Test Path message creation."""
    path = create_path_message(frame_id="world")

    assert isinstance(path, Path)
    assert path.header.frame_id == "world"
    assert len(path.poses) == 0


def test_create_path_message_custom_frame():
    """Test Path message creation with custom frame ID."""
    path = create_path_message(frame_id="map")

    assert path.header.frame_id == "map"


def test_add_pose_to_path():
    """Test adding a pose to a path."""
    path = create_path_message()

    position = np.array([1.0, 2.0, 3.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    add_pose_to_path(path, position, quaternion, frame_id="world")

    assert len(path.poses) == 1
    assert path.poses[0].pose.position.x == 1.0
    assert path.poses[0].pose.orientation.w == 1.0


def test_add_pose_to_path_without_quaternion():
    """Test adding a pose without quaternion (uses identity)."""
    path = create_path_message()

    position = np.array([1.0, 2.0, 3.0])

    add_pose_to_path(path, position, None, frame_id="world")

    assert len(path.poses) == 1
    assert path.poses[0].pose.orientation.w == 1.0  # Identity


def test_truncate_path():
    """Test path truncation."""
    path = create_path_message()

    # Add 10 poses
    for i in range(10):
        pos = np.array([float(i), 0.0, 0.0])
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        add_pose_to_path(path, pos, quat)

    assert len(path.poses) == 10

    # Truncate to 5
    truncate_path(path, max_poses=5)

    assert len(path.poses) == 5
    # Should remove oldest (first 5)
    assert path.poses[0].pose.position.x == 5.0


def test_truncate_path_at_limit():
    """Test that truncation does nothing when at or below limit."""
    path = create_path_message()

    for i in range(5):
        pos = np.array([float(i), 0.0, 0.0])
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        add_pose_to_path(path, pos, quat)

    original_length = len(path.poses)
    truncate_path(path, max_poses=10)

    assert len(path.poses) == original_length


# ==============================================================================
# Reference Sampling Tests
# ==============================================================================

def test_reference_sample_times_cover_full_range(dummy_traj):
    """Test that sampling covers full trajectory range."""
    times = compute_reference_sample_times(dummy_traj, sample_count=4)

    assert times[0] == pytest.approx(dummy_traj.time[0])
    assert times[-1] == pytest.approx(dummy_traj.time[-1])
    assert len(times) == 4


def test_reference_sample_times_duration_cap(dummy_traj):
    """Test that max_duration caps the sampling range."""
    times = compute_reference_sample_times(
        dummy_traj,
        sample_count=4,
        max_duration=4.0,
    )

    assert times[0] == pytest.approx(dummy_traj.time[0])
    assert times[-1] == pytest.approx(dummy_traj.time[0] + 4.0)


def test_reference_sample_times_raises_on_invalid_count(dummy_traj):
    """Test that sample_count < 2 raises ValueError."""
    with pytest.raises(ValueError, match="sample_count must be >= 2"):
        compute_reference_sample_times(dummy_traj, sample_count=1)


def test_reference_sample_times_single_value_when_start_equals_end():
    """Test that sampling returns single value when start equals end."""
    # Create a trajectory with start == end after max_duration
    times = compute_reference_sample_times(
        dummy_traj,
        sample_count=4,
        max_duration=0.0,
    )

    assert all(t == pytest.approx(dummy_traj.time[0]) for t in times)


# ==============================================================================
# QoS Profile Tests
# ==============================================================================

def test_create_reliable_qos():
    """Test reliable QoS profile creation."""
    qos = create_reliable_qos(depth=10)

    assert qos.reliability.name == "RELIABLE"
    assert qos.history.name == "KEEP_LAST"
    assert qos.depth == 10


def test_create_reliable_qos_custom_depth():
    """Test reliable QoS with custom depth."""
    qos = create_reliable_qos(depth=20)

    assert qos.depth == 20


def test_create_best_effort_qos():
    """Test best-effort QoS profile creation."""
    qos = create_best_effort_qos(depth=10)

    assert qos.reliability.name == "BEST_EFFORT"
    assert qos.history.name == "KEEP_LAST"
    assert qos.depth == 10


# ==============================================================================
# Time Utility Tests
# ==============================================================================

def test_sleep_period_skips_non_positive(monkeypatch):
    """Test that sleep_period skips non-positive durations."""
    called = []

    def fake_sleep(value):
        called.append(value)

    import ap_pnc.ros_viz_utils as rv
    monkeypatch.setattr(rv.time, "sleep", fake_sleep)

    rv.sleep_period(0.0)
    rv.sleep_period(-1.0)

    assert called == []


def test_sleep_period_invokes_time_sleep(monkeypatch):
    """Test that sleep_period calls time.sleep for positive durations."""
    recorded = []

    def fake_sleep(value):
        recorded.append(value)

    import ap_pnc.ros_viz_utils as rv
    monkeypatch.setattr(rv.time, "sleep", fake_sleep)

    rv.sleep_period(0.5)

    assert recorded == [0.5]
