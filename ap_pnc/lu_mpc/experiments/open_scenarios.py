"""
Open Collision-Free Scenarios for Tracker Performance Testing.

This module provides predefined, obstacle-free reference trajectories
(open circle, open line, open lemniscate) specifically crafted for evaluating
the tail-sitter UAV tracker without collision obstacles.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ap_pnc.env import get_git_root
from ap_pnc.lu_mpc.experiments.trajectory_gen import (
    TrajectoryGenConfig,
    generate_trajectory,
    make_circle_waypoints,
    trajectory_to_csv,
)

SCENARIOS = {
    "open_circle": {
        "name": "Open Circle Course",
        "description": "Obstacle-free horizontal circle trajectory (radius 15m, speed ~10m/s)",
        "radius": 15.0,
        "segments": 8,
        "segment_duration": 1.5,
        "altitude": 2.0,
        "speed": 10.0,
    },
    "open_line": {
        "name": "Open Straight Run",
        "description": "Obstacle-free straight line dash (60m, speed ~12m/s)",
        "length": 60.0,
        "segments": 4,
        "segment_duration": 1.25,
        "altitude": 2.0,
        "speed": 12.0,
    },
    "open_lemniscate": {
        "name": "Open Lemniscate Course",
        "description": "Obstacle-free figure-8 trajectory (radius 10m, speed ~8m/s)",
        "radius": 10.0,
        "segments": 12,
        "segment_duration": 1.2,
        "altitude": 2.0,
        "speed": 8.0,
    },
}


def make_lemniscate_waypoints(
    num_segments: int = 12, radius: float = 10.0, z: float = 2.0
) -> list[list[float]]:
    """Generate waypoints forming a smooth figure-8 (Bernoulli Lemniscate)."""
    t = np.linspace(0, 2 * np.pi, num_segments + 1)
    denom = 1 + np.sin(t) ** 2
    x = radius * np.cos(t) / denom
    y = radius * np.sin(t) * np.cos(t) / denom
    z_arr = np.full(len(t), z)
    return np.column_stack([x, y, z_arr]).tolist()


def make_line_waypoints(
    num_segments: int = 4, length: float = 60.0, z: float = 2.0
) -> list[list[float]]:
    """Generate waypoints along a straight flight line."""
    x = np.linspace(-length / 2.0, length / 2.0, num_segments + 1)
    y = np.zeros(num_segments + 1)
    z_arr = np.full(num_segments + 1, z)
    return np.column_stack([x, y, z_arr]).tolist()


def ensure_open_scenario(scenario_key: str = "open_circle", speed: float | None = None) -> Path:
    """
    Ensure the trajectory CSV for an open collision-free scenario exists.
    Generates and saves it to artifacts/simulations/temp_data/ if missing.
    """
    root = get_git_root()
    temp_dir = root / "artifacts" / "simulations" / "temp_data"
    temp_dir.mkdir(parents=True, exist_ok=True)

    if scenario_key not in SCENARIOS:
        scenario_key = "open_circle"

    meta = SCENARIOS[scenario_key]
    filename = f"{scenario_key}.csv"
    csv_path = temp_dir / filename

    if csv_path.exists():
        return csv_path

    seg_dur = meta["segment_duration"]
    if speed is not None and speed > 0.1:
        # Scale segment duration inversely with speed
        scale = meta["speed"] / speed
        seg_dur *= scale

    num_seg = meta["segments"]
    if scenario_key == "open_circle":
        wp = make_circle_waypoints(num_seg, radius=meta["radius"], z=meta["altitude"])
    elif scenario_key == "open_line":
        wp = make_line_waypoints(num_seg, length=meta["length"], z=meta["altitude"])
    elif scenario_key == "open_lemniscate":
        wp = make_lemniscate_waypoints(num_seg, radius=meta["radius"], z=meta["altitude"])
    else:
        wp = make_circle_waypoints(num_seg, radius=15.0, z=2.0)

    cfg = TrajectoryGenConfig(
        dt=0.02,
        waypoints=wp,
        segment_durations=[seg_dur] * num_seg,
        a_max=15.0,
        output_dir="artifacts/simulations/temp_data",
        filename=filename,
    )

    traj = generate_trajectory(cfg)
    trajectory_to_csv(traj, csv_path)
    return csv_path
