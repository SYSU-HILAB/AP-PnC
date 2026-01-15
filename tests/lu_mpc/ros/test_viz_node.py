import numpy as np
import pytest

from feature.utils import ros_visualizer


class _DummyTrajectory:
    """Fixture-friendly trajectory stub exposing the attributes StatePublisherNode expects."""

    def __init__(self):
        self.time = np.linspace(2.0, 18.0, 5)


@pytest.fixture
def dummy_traj():
    return _DummyTrajectory()


def test_reference_sample_times_cover_full_csv(dummy_traj):
    times = ros_visualizer.compute_reference_sample_times(dummy_traj, sample_count=4)
    assert times[0] == pytest.approx(dummy_traj.time[0])
    assert times[-1] == pytest.approx(dummy_traj.time[-1])


def test_reference_sample_times_can_be_duration_capped(dummy_traj):
    times = ros_visualizer.compute_reference_sample_times(
        dummy_traj,
        sample_count=4,
        max_duration=4.0,
    )
    assert times[-1] == pytest.approx(dummy_traj.time[0] + 4.0)


def test_topic_helper_applies_nmpc_prefix():
    assert ros_visualizer.make_topic("reference_path") == "/nmpc/reference_path"


def test_sleep_period_skips_non_positive(monkeypatch):
    called = []

    def fake_sleep(value):
        called.append(value)

    class DummyTime:
        def sleep(self, value):
            fake_sleep(value)

    monkeypatch.setattr(ros_visualizer, "time", DummyTime())
    ros_visualizer.sleep_period(0.0)
    ros_visualizer.sleep_period(-1.0)
    assert called == []


def test_sleep_period_invokes_time(monkeypatch):
    recorded = []

    def fake_sleep(value):
        recorded.append(value)

    class DummyTime:
        def sleep(self, value):
            fake_sleep(value)

    monkeypatch.setattr(ros_visualizer, "time", DummyTime())
    ros_visualizer.sleep_period(0.5)
    assert recorded == [0.5]
