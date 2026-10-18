"""Synthetic calibration import; no private values or runtime frame adapters."""

import copy

import pytest

from tooling.actuator_profile import import_matlab_actuator


@pytest.fixture
def snapshot():
    return {
        "model": "practical",
        "kp": [1, 2, 3],
        "ki": [4, 5, 6],
        "kd": [7, 8, 9],
        "damping_wing": [-1, -2, -3],
        "tilts_rad": [0.1, -0.2, 0.3, -0.4],
        "mixer": [[1, 2, 3, 4], [1, 5, 6, 7], [1, 8, 9, 10], [1, 11, 12, 13]],
        "arms_m": [0.2, 0.3, 0.4, 0.5],
        "motor_tau_s": 0.076,
    }


def test_import_preserves_snapshot_and_physics(snapshot):
    original = copy.deepcopy(snapshot)
    native = import_matlab_actuator(snapshot)
    assert snapshot == original
    assert native["body_frame"] == "FLU"
    assert native["kp"] == [3, 2, 1]
    assert native["ki"] == [6, 5, 4]
    assert native["kd"] == [9, 8, 7]
    assert native["damping_body"] == [-3, -2, -1]
    assert native["tilts_rad"] == [-0.1, 0.2, -0.3, 0.4]
    assert native["arms_m"] == original["arms_m"]
    assert native["motor_tau_s"] == original["motor_tau_s"]
    assert native["mixer"] == [[1, 4, -3, 2], [1, 7, -6, 5], [1, 10, -9, 8], [1, 13, -12, 11]]
    # A second preparation of a stamped snapshot must not rotate anything again.
    assert import_matlab_actuator(native) == native
    native["kp"][0] *= 3
    assert snapshot == original


@pytest.mark.parametrize(
    "change",
    [
        {"body_frame": "unknown"},
        {"body_frame": "FLU"},
        {"kp": [1, 2]},
        {"tilts_rad": [0, 0, 0, float("nan")]},
        {"mixer": [[1, 2, 3]]},
    ],
)
def test_ambiguous_or_invalid_parameters_rejected(snapshot, change):
    snapshot.update(change)
    with pytest.raises(ValueError):
        import_matlab_actuator(snapshot)


def test_unstamped_unknown_profile_rejected():
    with pytest.raises(ValueError, match="stamped FLU"):
        import_matlab_actuator({"kp": [1, 2, 3]})
