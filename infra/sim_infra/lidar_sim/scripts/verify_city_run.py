"""Verify a completed local city run from saved data, not from browser pixels."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree

from tooling.env import artifact_path


def check_map(job, mission, positions):
    """Independent equivalent of the native 26-neighbour voxel dilation."""
    cloud = job / "scene/occupancy.ply"
    expected = json.loads((job / "mission-metadata.json").read_text())["asset_sha256"]["occupancy.ply"]
    if hashlib.sha256(cloud.read_bytes()).hexdigest() != expected:
        raise ValueError("frozen occupancy hash changed")
    with cloud.open("rb") as source:
        header = []
        while True:
            line = source.readline()
            if not line or len(header) > 100:
                raise ValueError("invalid occupancy header")
            header.append(line.strip())
            if line.strip() == b"end_header":
                break
        if b"format binary_little_endian 1.0" not in header:
            raise ValueError("binary xyz occupancy required")
        count = int(next(line.split()[-1] for line in header if line.startswith(b"element vertex ")))
        points = np.fromfile(source, dtype="<f4").reshape(-1, 3) * mission["scene_scale"]
        if len(points) != count or not np.isfinite(points).all():
            raise ValueError("invalid occupancy payload")
    origin, corner = np.array(mission["map_origin_m"]), np.array(mission["map_corner_m"])
    voxel = mission["voxel_m"]
    points = points[((points >= origin) & (points < corner)).all(axis=1)]
    if not len(points):
        raise ValueError("no occupancy points in mission bounds")
    if not np.isfinite(positions).all() or not ((positions >= origin) & (positions < corner)).all():
        raise ValueError("reference or plant left map bounds")
    cells = np.floor((points - origin) / voxel)
    distances, _ = cKDTree(np.unique(cells, axis=0)).query(np.floor((positions - origin) / voxel), p=np.inf)
    if np.any(distances <= mission["dilation_cells"]):
        raise ValueError("reference or plant intersects independently reconstructed inflated map")
    return {"checked_positions": len(positions),
            "min_extra_voxel_cells": float(distances.min() - mission["dilation_cells"]),
            "scope": "committed plant states and reference sampled every <=0.01s; not contact physics"}


def verify(job):
    if not job.is_absolute():
        raise ValueError("absolute prepared job required")
    job = artifact_path(job)
    if (job / "execution.json").exists():
        execution = json.loads((job / "execution.json").read_text())
        for name, expected in execution["frozen_files_sha256"].items():
            if hashlib.sha256((job / name).read_bytes()).hexdigest() != expected:
                raise ValueError("execution inputs changed after launch")
        if hashlib.sha256((job / "executed-city_mission_node").read_bytes()).hexdigest() != execution["executable_sha256"]:
            raise ValueError("executed binary snapshot changed")
    live = json.loads((job / "stream/live.json").read_text())
    mission = yaml.safe_load((job / "mission.yaml").read_text())
    run = job / "recordings" / Path(live["recording"]).name
    manifest = json.loads((run / "manifest.json").read_text())
    metrics = json.loads((run / "metrics.json").read_text())
    events = [json.loads(line) for line in (run / "replans.jsonl").read_text().splitlines()]
    with (run / "steps.csv").open() as source:
        records = list(csv.DictReader(source))
    if live["phase"] != "complete" or manifest["status"] != "finished":
        raise ValueError("mission did not complete")
    if len(records) != manifest["committed_steps"] or len(events) != len(mission["waypoints_m"]) - 1:
        raise ValueError("incomplete simulation/planning record")
    if not records or not np.isfinite([[float(value) for value in row.values()] for row in records]).all():
        raise ValueError("empty or nonfinite simulation record")
    times_ns = np.array([int(row["time_ns"]) for row in records])
    next_ns = np.array([int(row["next_time_ns"]) for row in records])
    if (times_ns[0] != 0 or not (next_ns > times_ns).all()
            or not np.array_equal(next_ns[:-1], times_ns[1:])
            or any(int(row["tick"]) != i for i, row in enumerate(records))
            or abs(next_ns[-1] * 1e-9 - live["t"]) > 1e-9):
        raise ValueError("noncontiguous simulation clock or tick sequence")
    if any(int(row["solver_status"]) != 0 for row in records):
        raise ValueError("nonzero NMPC status")
    pieces = yaml.safe_load((run / "reference.yaml").read_text())["pieces"]
    duration = np.array([piece["duration_s"] for piece in pieces])
    coefficients = np.array([piece["coefficients_xyz_descending"] for piece in pieces])
    if not np.isfinite(coefficients).all() or not (duration > 0).all():
        raise ValueError("invalid saved polynomial")
    boundaries = np.cumsum(duration)
    max_reference_error = 0.0
    max_join_error = 0.0
    for index in range(len(pieces) - 1):
        for order in range(3):
            left = [np.polyval(np.polyder(c, order), duration[index]) for c in coefficients[index]]
            right = [np.polyval(np.polyder(c, order), 0) for c in coefficients[index + 1]]
            max_join_error = max(max_join_error, float(np.linalg.norm(np.array(left) - right)))
    for row in records:
        t = int(row["time_ns"]) * 1e-9
        index = min(int(np.searchsorted(boundaries, t)), len(pieces) - 1)
        local = t - (boundaries[index - 1] if index else 0)
        reference = np.array([np.polyval(c, local) for c in coefficients[index]])
        logged = np.array([float(row[f"ref_p_{axis}"]) for axis in "xyz"])
        max_reference_error = max(max_reference_error, float(np.linalg.norm(reference - logged)))
    if max_reference_error > 1e-7 or max_join_error > 1e-5:
        raise ValueError("saved reference differs from executed reference or breaks C2 continuity")
    final = np.array([float(records[-1][f"next_p_{axis}"]) for axis in "xyz"])
    speed = np.linalg.norm([float(records[-1][f"next_v_{axis}"]) for axis in "xyz"])
    error = np.linalg.norm(final - mission["waypoints_m"][0])
    if error > 1 or speed > 0.5:
        raise ValueError("terminal hover tolerance failed")
    for index, event in enumerate(events):
        if event["id"] != index + 1 or event["target_m"] != mission["waypoints_m"][index + 1]:
            raise ValueError("waypoint order changed")
        if event["starts_at_s"] - event["planned_at_s"] < 1:
            raise ValueError("planning overwrote active NMPC horizon")
    plant_positions = np.array([[float(row[f"{prefix}p_{axis}"]) for axis in "xyz"]
                                for row in records for prefix in ("", "next_")])
    sampled_reference = np.concatenate([
        np.array([np.polyval(c, np.linspace(0, d, int(np.ceil(d / .01)) + 1)) for c in coeff]).T
        for d, coeff in zip(duration, coefficients, strict=True)])
    map_result = check_map(job, mission, np.vstack([plant_positions, sampled_reference]))
    errors = np.linalg.norm([[float(row[f"p_{axis}"]) - float(row[f"ref_p_{axis}"])
                              for axis in "xyz"] for row in records], axis=1)
    position_rmse, position_max = float(np.sqrt(np.mean(errors ** 2))), float(errors.max())
    if not np.allclose([position_rmse, position_max],
                       [metrics["position_rmse_m"], metrics["position_max_m"]], rtol=1e-9, atol=1e-9):
        raise ValueError("saved tracking metrics differ from CSV")
    if position_max > 2.0:
        raise ValueError("maximum tracking error exceeds 2m")
    ring_result = {}
    if "ring_radius_m" in mission:
        center = np.array(mission["ring_center_m"])
        radius = mission["ring_radius_m"]
        radial_error = np.abs(np.linalg.norm(plant_positions[:, :2] - center[:2], axis=1) - radius)
        reference_radial_error = np.abs(np.linalg.norm(sampled_reference[:, :2] - center[:2], axis=1) - radius)
        if radial_error.max() > 2 or reference_radial_error.max() > 1:
            raise ValueError("ring path exceeds road-centreline tolerance")
        ring_result = {"actual_max_radial_error_m": float(radial_error.max()),
                       "reference_max_radial_error_m": float(reference_radial_error.max())}
    cruise_result = {}
    if "velocities_mps" in mission:
        gaps = [b["starts_at_s"] - a["starts_at_s"] - a["duration_s"]
                for a, b in zip(events[:-1], events[1:], strict=True)]
        if max(abs(gap) for gap in gaps) > 1e-3:
            raise ValueError("intermediate waypoint dwell or reference gap detected")
        times = np.array([int(row["time_ns"]) * 1e-9 for row in records])
        speeds = np.linalg.norm([[float(row[f"v_{axis}"]) for axis in "xyz"] for row in records], axis=1)
        ref_speeds = np.linalg.norm([[float(row[f"ref_v_{axis}"]) for axis in "xyz"] for row in records], axis=1)
        start = events[0]["starts_at_s"] + events[0]["duration_s"]
        end = events[-1]["starts_at_s"]
        cruise = (times >= start) & (times < end)
        waypoint_speeds = [float(speeds[np.argmin(abs(times - (e["starts_at_s"] + e["duration_s"])))])
                           for e in events[:-1]]
        cruise_result = {
            "cruise_window_s": [start, end],
            "cruise_actual_speed_mps": {"min": float(speeds[cruise].min()),
                                        "mean": float(speeds[cruise].mean()), "max": float(speeds[cruise].max())},
            "cruise_reference_speed_mps": [float(ref_speeds[cruise].min()), float(ref_speeds[cruise].max())],
            "intermediate_waypoint_actual_speeds_mps": waypoint_speeds,
            "max_intermediate_dwell_s": max(abs(gap) for gap in gaps),
        }
        if (np.abs(ref_speeds[cruise] - 10).max() > 0.03 or speeds[cruise].min() < 9.0
                or speeds[cruise].mean() < 9.5 or min(waypoint_speeds) < 9.0):
            print(json.dumps(cruise_result, indent=2))
            raise ValueError("10 m/s continuous cruise acceptance failed")
    result = {
        **cruise_result, **ring_result, "independent_map_check": map_result,
        "status": "passed", "simulation_s": live["t"], "committed_steps": len(records),
        "online_plans": len(events), "planning_wall_ms": [e["wall_ms"] for e in events],
        "return_error_m": float(error), "return_speed_mps": float(speed),
        "saved_reference_max_error_m": max_reference_error,
        "polynomial_c2_max_join_error": max_join_error,
        "position_rmse_m": position_rmse, "position_max_m": position_max,
        "solver_nonzero_statuses": 0, "recording": str(run),
        "recording_name": run.name, "stream": live["stream"],
        "boundary": "static Gaussian-derived map, no contact physics or dynamic obstacles",
    }
    (run / "verification.json").write_text(json.dumps(result, indent=2))
    temporary = job / "verification.json.tmp"
    temporary.write_text(json.dumps(result, indent=2))
    temporary.replace(job / "verification.json")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("job", type=Path)
    verify(parser.parse_args().job)
