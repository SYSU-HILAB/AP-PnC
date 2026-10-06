"""Offline diagnostics retain the last committed state, even on failed runs."""

import csv
import importlib.util

import numpy as np
import pytest

from tooling.env import get_git_root


def module():
    path = get_git_root() / "infra/sim_infra/simple_sim/stack/plot_aero_history.py"
    spec = importlib.util.spec_from_file_location("aero_history", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def fixture(folder, valid=1):
    row = {"time_ns": 0, "next_time_ns": 20_000_000}
    for prefix, angle, force in (("", 0.2, 1.0), ("next_", 0.3, 9.0)):
        row[prefix + "aero_alpha_rad"] = angle
        row[prefix + "aero_beta_rad"] = angle / 2
        row[prefix + "aero_observation_valid"] = valid
        for i, axis in enumerate("xyz"):
            row[prefix + "air_wing_v_" + axis] = i + 1.0
            row[prefix + "aero_f" + axis] = force + i
            row[prefix + "aero_m" + axis] = 0.1 * i
    with (folder / "steps.csv").open("w") as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)


def test_final_committed_force_and_angle_included(tmp_path):
    fixture(tmp_path)
    time, angle, velocity, force, moment = module().committed_series(tmp_path)
    np.testing.assert_allclose(time, [0, 0.02])
    np.testing.assert_allclose(angle[:, 0], [0.2, 0.3])
    np.testing.assert_allclose(force[-1], [9, 10, 11])
    assert velocity.shape == moment.shape == (2, 3)


def test_missing_model_observation_is_not_fabricated(tmp_path):
    fixture(tmp_path, valid=0)
    with pytest.raises(ValueError, match="unavailable"):
        module().committed_series(tmp_path)
