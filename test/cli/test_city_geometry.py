"""Synthetic-only regression checks for scene reflection and offline map checks."""

import csv
import hashlib
import itertools
import json

import numpy as np
import pytest
import yaml
from infra.sim_infra.lidar_sim.scripts.prepare_city_ring import reflect_gaussians
from infra.sim_infra.lidar_sim.scripts.verify_city_run import check_map, verify
from scipy.spatial.transform import Rotation


@pytest.mark.parametrize("signs", list(itertools.product((-1, 1), repeat=3)))
def test_reflection_preserves_covariance_and_sh(signs):
    names = list("xyz") + [f"n{k}" for k in "xyz"] + [f"rot_{k}" for k in range(4)]
    names += [f"scale_{k}" for k in range(3)] + [f"f_rest_{k}" for k in range(24)]
    data = np.zeros(12, dtype=[(name, "f8") for name in names])
    rng = np.random.default_rng(213)
    for name in names:
        data[name] = rng.normal(size=len(data))
    q = Rotation.random(len(data), random_state=rng).as_quat()
    for i, name in enumerate(["rot_1", "rot_2", "rot_3", "rot_0"]):
        data[name] = q[:, i]
    before = data.copy()
    reflected = reflect_gaussians(data, signs)
    assert np.array_equal(data, before)
    assert np.array_equal(reflect_gaussians(reflected, signs), data)
    mirror = np.diag(signs)
    for i in range(len(data)):
        r = Rotation.from_quat(q[i]).as_matrix()
        rr = Rotation.from_quat([reflected[f"rot_{k}"][i] for k in (1, 2, 3, 0)]).as_matrix()
        diagonal = np.diag([np.exp(2 * data[f"scale_{k}"][i]) for k in range(3)])
        np.testing.assert_allclose(rr @ diagonal @ rr.T, mirror @ r @ diagonal @ r.T @ mirror, atol=1e-10)
        assert np.linalg.det(rr) == pytest.approx(1)
    for axis, sign in zip("xyz", signs, strict=True):
        np.testing.assert_array_equal(reflected[axis], data[axis] * sign)
        np.testing.assert_array_equal(reflected[f"n{axis}"], data[f"n{axis}"] * sign)
    direction = rng.normal(size=(20, 3))
    direction /= np.linalg.norm(direction, axis=1)[:, None]

    def basis(v):
        x, y, z = v.T
        return np.column_stack([y, z, x, x * y, y * z, 3 * z * z - 1, x * z, x * x - y * y])

    for channel in range(3):
        fields = [f"f_rest_{8 * channel + k}" for k in range(8)]
        coefficients = np.column_stack([data[k] for k in fields])
        reflected_coefficients = np.column_stack([reflected[k] for k in fields])
        np.testing.assert_allclose(basis(direction) @ reflected_coefficients.T,
                                   basis(direction * signs) @ coefficients.T, atol=1e-12)


@pytest.fixture
def synthetic_map(tmp_path):
    cloud = tmp_path / "scene/occupancy.ply"
    cloud.parent.mkdir()
    points = np.array([[4.25, 4.25, 4.25]], dtype="<f4")
    cloud.write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex 1\n"
                      b"property float x\nproperty float y\nproperty float z\nend_header\n" + points.tobytes())
    (tmp_path / "mission-metadata.json").write_text(json.dumps({"asset_sha256": {
        "occupancy.ply": hashlib.sha256(cloud.read_bytes()).hexdigest()}}))
    mission = {"scene_scale": 1., "map_origin_m": [0, 0, 0], "map_corner_m": [10, 10, 10],
               "voxel_m": 1., "dilation_cells": 1}
    return tmp_path, mission


def test_independent_dilation_includes_diagonal_neighbours(synthetic_map):
    job, mission = synthetic_map
    with pytest.raises(ValueError, match="inflated map"):
        check_map(job, mission, np.array([[5.1, 5.1, 5.1]]))
    result = check_map(job, mission, np.array([[6.1, 6.1, 6.1]]))
    assert result["min_extra_voxel_cells"] == 1


@pytest.mark.parametrize("position", [[-0.1, 1, 1], [10, 1, 1], [float("nan"), 1, 1]])
def test_map_rejects_outside_and_nonfinite_positions(synthetic_map, position):
    job, mission = synthetic_map
    with pytest.raises(ValueError, match="map bounds"):
        check_map(job, mission, np.array([position]))


def test_map_rejects_no_coverage(synthetic_map):
    job, mission = synthetic_map
    mission["map_origin_m"] = [6, 6, 6]
    with pytest.raises(ValueError, match="no occupancy points"):
        check_map(job, mission, np.array([[7, 7, 7]]))


@pytest.mark.parametrize("field,value,message", [("p_x", "nan", "nonfinite"),
                                               ("tick", "1", "clock"),
                                               ("time_ns", "1", "clock"),
                                               ("next_time_ns", "0", "clock")])
def test_verifier_rejects_corrupt_csv_before_reference_checks(tmp_path, monkeypatch, field, value, message):
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    job = tmp_path / ".artifacts/job"
    (job / "stream").mkdir(parents=True)
    run = job / "recordings/synthetic"
    run.mkdir(parents=True)
    (job / "stream/live.json").write_text(json.dumps({"phase": "complete", "t": .02,
                                                      "recording": "/synthetic"}))
    (job / "mission.yaml").write_text(yaml.safe_dump({"waypoints_m": [[0, 0, 0]] * 4}))
    (run / "manifest.json").write_text(json.dumps({"status": "finished", "committed_steps": 1}))
    (run / "metrics.json").write_text("{}")
    (run / "replans.jsonl").write_text("{}\n" * 3)
    row = {"tick": "0", "time_ns": "0", "next_time_ns": "20000000", "solver_status": "0", "p_x": "0"}
    row[field] = value
    with (run / "steps.csv").open("w") as target:
        writer = csv.DictWriter(target, fieldnames=row.keys())
        writer.writeheader()
        writer.writerow(row)
    with pytest.raises(ValueError, match=message):
        verify(job)


def test_map_rejects_changed_frozen_asset(synthetic_map):
    job, mission = synthetic_map
    with (job / "scene/occupancy.ply").open("ab") as stream:
        stream.write(b"changed")
    with pytest.raises(ValueError, match="hash changed"):
        check_map(job, mission, np.array([[6, 6, 6]]))
