from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from ap_pnc.lu_mpc.controller.trajectory_loader import (
    TrajectoryReference,
    load_trajectory_csv,
)

REQUIRED_HEADERS = [
    "time",
    "pos_x",
    "pos_y",
    "pos_z",
    "vel_x",
    "vel_y",
    "vel_z",
    "yb_x",
    "yb_y",
    "yb_z",
    "df_phi_alpha",
    "df_lyu_alpha",
    "df_phi_quat_w",
    "df_phi_quat_x",
    "df_phi_quat_y",
    "df_phi_quat_z",
    "df_lyu_quat_w",
    "df_lyu_quat_x",
    "df_lyu_quat_y",
    "df_lyu_quat_z",
]

FULL_HEADERS = REQUIRED_HEADERS + ["thrust"]


def _write_csv(path: Path, headers: list[str], rows: list[list[float]]) -> Path:
    with path.open("w", encoding="utf-8") as handle:
        handle.write(",".join(headers) + "\n")
        for row in rows:
            handle.write(",".join(str(value) for value in row) + "\n")
    return path


def _write_valid_csv(path: Path) -> Path:
    rows = [
        [
            0.0,
            0.0,
            0.0,
            -1.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.1,
            0.15,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.2,
        ],
        [
            1.0,
            1.0,
            0.5,
            -0.5,
            1.0,
            0.2,
            -0.1,
            0.0,
            1.0,
            0.0,
            0.2,
            0.25,
            0.9239,
            0.3827,
            0.0,
            0.0,
            0.9239,
            0.3827,
            0.0,
            0.0,
            0.25,
        ],
    ]
    return _write_csv(path, FULL_HEADERS, rows)


def test_load_trajectory_csv_missing_required_fields_raises(tmp_path: Path):
    csv_path = _write_csv(tmp_path / "invalid.csv", ["time"], [[0.0], [1.0]])
    with pytest.raises(ValueError):
        load_trajectory_csv(str(csv_path))


def test_load_trajectory_csv_produces_expected_arrays(tmp_path: Path):
    csv_path = _write_valid_csv(tmp_path / "valid.csv")
    trajectory = load_trajectory_csv(str(csv_path))

    assert trajectory["position"].shape == (2, 3)
    assert np.allclose(trajectory["alpha"], trajectory["df_phi_alpha"])
    assert "df_phi_quat" in trajectory and trajectory["df_phi_quat"].shape == (2, 4)

    reference = TrajectoryReference(trajectory)
    sample_window = reference.get_reference_window(cur_idx_ref=0, ctrl_dt=0.02, N=0)

    # Extract first (only) element from window
    sample = {"position": sample_window["position"][0], "yb": sample_window["yb"][0]}

    assert sample["position"].shape == (3,)
    assert sample["yb"].shape == (3,)
