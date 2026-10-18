"""Stage the local-only MatrixCity experiment; numerical execution is Docker-only."""

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import time
from pathlib import Path

import yaml

from tooling.env import artifact_path, get_git_root
from tooling.simple_stack import JobRequest, native_arch, write_configuration


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare(scene="ring", rate_tuning="auto"):
    if scene not in ("ring", "original"):
        raise ValueError("unknown city scene")
    if rate_tuning not in ("auto", "baseline", "body-z-pd3"):
        raise ValueError("unknown rate tuning")
    if rate_tuning == "auto":
        rate_tuning = "body-z-pd3" if scene == "ring" else "baseline"
    root = get_git_root()
    scene_scale = 1.0 if scene == "ring" else 6.0
    route_gain = scene_scale / 3.0
    viewer_assets = artifact_path(".artifacts/dev-sim/city-loop/assets")
    assets = artifact_path(
        f".artifacts/dev-sim/{'city-ring' if scene == 'ring' else 'city-loop'}/assets"
    )
    profile = root / ".artifacts/vtol-matlab/profile.yaml"
    provenance = profile.with_name("provenance.json")
    for required in (
        profile,
        provenance,
        assets / "city.ply",
        assets / "occupancy.ply",
        assets / "scene.json",
    ):
        if not required.is_file():
            raise FileNotFoundError(required)
    job = artifact_path(f".artifacts/dev-sim/city-loop/jobs/{time.time_ns()}")
    (job / "config").mkdir(parents=True)
    (job / "recordings").mkdir()
    (job / "stream").mkdir()
    (job / "scene").mkdir()
    shutil.copy2(assets / "occupancy.ply", job / "scene/occupancy.ply")
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml"):
        shutil.copy2(root / "core/bringup/config" / name, job / "config" / name)
    shutil.copy2(profile, job / "profile.yaml")
    shutil.copy2(provenance, job / "upstream-provenance.json")
    request = JobRequest(
        profile="practical",
        aero="lyu",
        v_max=10.0,
        duration_s=240,
        overrides={
            "simple_sim.controller": "nmpc",
            "problem_formulation.corridor_source": "map_sfc",
            "problem_formulation.cost.time_weight": 10000.0,
            "problem_formulation.cost.vel_weight": 100000.0,
            "problem_formulation.minco.rel_cost_tol": 1e-4,
            "problem_formulation.map.sfc_progress_m": 12.0
            if scene == "ring"
            else 6.0 * scene_scale,
            "problem_formulation.map.sfc_range_m": 8.0 if scene == "ring" else 3.0 * scene_scale,
            "problem_formulation.map.speed_controlled": True,
        },
    )
    request.validate()
    write_configuration(job, request)
    if rate_tuning == "body-z-pd3":
        # Controller tuning, NOT new motor calibration. Keep the source profile,
        # motor lag/polars/mixer/inertia and the other two feedback axes unchanged.
        path = job / "config/simple_sim.yaml"
        configuration = yaml.safe_load(path.read_text())
        actuator = configuration["simple_sim"]["actuator"]
        for gain in ("kp", "kd"):
            actuator[gain][2] *= 3.0  # Quadrotor body-Z angular-rate channel.
        path.write_text(yaml.safe_dump(configuration, sort_keys=False))
    container_job = Path("/workspace") / job.relative_to(root)
    mission = {
        "output": str(container_job / "stream"),
        "planning": "/workspace/core/bringup/config/planning.yaml",
        "simulation": "/workspace/core/bringup/config/simple_sim.yaml",
        "occupancy": str(container_job / "scene/occupancy.ply"),
        "scene_scale": scene_scale,
        "map_origin_m": [route_gain * v for v in [-43.5, -49.5, -7.5]],
        "map_corner_m": [route_gain * v for v in [196.5, 121.5, 127.5]],
        "voxel_m": route_gain,
        "dilation_cells": 2,
        "cruise_mps": 10.0,
        "waypoints_m": [
            [74, -12, 18],
            [130, -12, 18],
            [150, 8, 18],
            [150, 52, 18],
            [130, 72, 18],
            [17, 72, 18],
            [-3, 52, 18],
            [-3, 8, 18],
            [17, -12, 18],
            [74, -12, 18],
        ],
        "velocities_mps": [
            [0, 0, 0],
            [10, 0, 0],
            [0, 10, 0],
            [0, 10, 0],
            [-10, 0, 0],
            [-10, 0, 0],
            [0, -10, 0],
            [0, -10, 0],
            [10, 0, 0],
            [0, 0, 0],
        ],
    }
    mission["waypoints_m"] = [
        [route_gain * value for value in point] for point in mission["waypoints_m"]
    ]
    if scene == "ring":
        radius, height, sectors = 88.8, 12.0, 16
        # Reserve the last 45 degrees for a gentler return deceleration.
        indices = [*range(sectors - 1), sectors]
        angles = [-math.pi / 2 + i * 2 * math.pi / sectors for i in indices]
        mission.update(
            {
                "map_origin_m": [-120.25, -120.25, -4.25],
                "map_corner_m": [120.25, 120.25, 32.25],
                "voxel_m": 0.5,
                "dilation_cells": 4,
                "ring_radius_m": radius,
                "ring_center_m": [0.0, 0.0, height],
                "waypoints_m": [
                    [radius * math.cos(a), radius * math.sin(a), height] for a in angles
                ],
                "velocities_mps": [[-10 * math.sin(a), 10 * math.cos(a), 0.0] for a in angles],
                "accelerations_mps2": [
                    [-100 / radius * math.cos(a), -100 / radius * math.sin(a), 0.0] for a in angles
                ],
            }
        )
        mission["waypoints_m"][-1] = mission["waypoints_m"][0].copy()
        for key in ("velocities_mps", "accelerations_mps2"):
            mission[key][0] = [0.0, 0.0, 0.0]
            mission[key][-1] = [0.0, 0.0, 0.0]
    (job / "mission.yaml").write_text(yaml.safe_dump(mission, sort_keys=False))
    (job / "scene.json").write_text((assets / "scene.json").read_text())
    hashes = {name: file_sha256(assets / name) for name in ("city.ply", "occupancy.ply")}
    (job / "mission-metadata.json").write_text(
        json.dumps(
            {
                "schema": "ap-pnc/city-demo/v1",
                "asset_sha256": hashes,
                "solver_variant": "baseline",
                "inner_rate_tuning": rate_tuning,
                "acceptance": "prepared: this run awaits independent 10 m/s practical acceptance",
                "scene": scene,
                "synthetic_mirrored": scene == "ring",
                "voxel_m": mission["voxel_m"],
                "dilation_m": mission["voxel_m"] * mission["dilation_cells"],
                "scene_scale": scene_scale,
                "effective_metres_per_source_unit": 40.0 * scene_scale,
                "cruise_mps": 10.0,
                "planner": "shared planner_core: A* / FIRI checked speed-controlled PVA quintic, online asynchronous legs; GCOPTER mode not used",
                "map": "static full-scene occupancy, not online LiDAR reconstruction",
                "plant": "practical local profile + Lyu + INDI + real NMPC",
                "replanning": "future C2 reference append, nonzero fly-through PVA, terminal hover only",
                "collision_dynamics": False,
                "hardware_connected": False,
                "scale": "visual metric calibration, not surveyed",
            },
            indent=2,
        )
    )
    rig = json.loads((root / "infra/sim_infra/lidar_sim/web/camera_rig.json").read_text())
    view = {
        "scene_scale": scene_scale,
        "overview_eye": [route_gain * v for v in [192, -135, 240]],
        "overview_target": [route_gain * v for v in [74, 30, 9]],
        "initial_position": mission["waypoints_m"][0],
        "camera_rig": rig,
    }
    if scene == "ring":
        view.update(
            {
                "scene_asset": "/assets/ring-city.ply",
                "synthetic_mirrored": True,
                "overview_eye": [180, -220, 250],
                "overview_target": [0, 0, 5],
            }
        )
        viewer_assets.mkdir(parents=True, exist_ok=True)
        link = viewer_assets / "ring-city.ply"
        if link.exists() and not link.is_symlink():
            raise FileExistsError(f"refusing to replace regular file {link}")
        link.unlink(missing_ok=True)
        link.symlink_to(os.path.relpath(assets / "city.ply", start=viewer_assets))
    (job / "view.json").write_text(json.dumps(view, indent=2))
    (viewer_assets / "view.json").write_text(json.dumps(view, indent=2))
    live = viewer_assets / "live.json"
    if live.exists() and not live.is_symlink():
        raise FileExistsError(f"refusing to replace regular file {live}")
    # Both locations share the same project-root-relative layout in Docker.
    for name in ("live.json", "poses.json", "verification.json"):
        link = viewer_assets / name
        if link.exists() and not link.is_symlink():
            raise FileExistsError(f"refusing to replace regular file {link}")
        link.unlink(missing_ok=True)
        target = job / name if name == "verification.json" else job / "stream" / name
        link.symlink_to(os.path.relpath(target, start=viewer_assets))
    print(job)
    return job


def launch(job):
    if not job.is_absolute():
        raise ValueError("absolute prepared job path required")
    job = artifact_path(job)
    root = get_git_root()
    metadata = json.loads((job / "mission-metadata.json").read_text())
    if (
        metadata.get("schema") != "ap-pnc/city-demo/v1"
        or metadata.get("solver_variant") != "baseline"
    ):
        raise ValueError("invalid or obsolete prepared city mission")
    executable = root / ".artifacts/dev-sim/city-loop/build/city_mission_node"
    if not executable.is_file():
        raise FileNotFoundError(f"build the native city consumer first: {executable}")
    if not (job / "mission.yaml").is_file():
        raise FileNotFoundError(job / "mission.yaml")
    snapshot = job / "executed-city_mission_node"
    if snapshot.exists() and file_sha256(snapshot) != file_sha256(executable):
        raise ValueError("prepared job is already bound to another executable")
    if not snapshot.exists():
        shutil.copy2(executable, snapshot)
    execution = {
        "platform": f"linux/{native_arch()}",
        "executable_sha256": file_sha256(snapshot),
        "frozen_files_sha256": {
            name: file_sha256(job / name)
            for name in (
                "mission.yaml",
                "config/nmpc.yaml",
                "config/simple_sim.yaml",
                "config/planning.yaml",
                "profile.yaml",
                "upstream-provenance.json",
                "scene/occupancy.ply",
            )
        },
    }
    receipt = job / "execution.json"
    if receipt.exists() and json.loads(receipt.read_text()) != execution:
        raise ValueError("frozen execution inputs changed; prepare a new job")
    receipt.write_text(json.dumps(execution, indent=2))
    container_job = Path("/workspace") / job.relative_to(root)
    arguments = [
        "docker",
        "run",
        "-d",
        "--name",
        f"ap-pnc-city-{job.name}",
        "--platform",
        f"linux/{native_arch()}",
        "--ipc=host",
        "--network",
        "none",
        "--entrypoint",
        "/bin/bash",
        "-e",
        "AP_PNC_DIR=/workspace",
        "-e",
        "ROS_DOMAIN_ID=213",
        "-e",
        "ROS_LOCALHOST_ONLY=1",
        "-e",
        f"ROS_LOG_DIR={container_job}/ros-log",
        "-v",
        f"{root}:/workspace:ro",
        "-v",
        f"{root}/.artifacts/dev-sim/city-loop:/workspace/.artifacts/dev-sim/city-loop:rw",
        "-v",
        f"{job}/recordings:/workspace/.artifacts/benchmark:rw",
    ]
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml"):
        arguments += ["-v", f"{job}/config/{name}:/workspace/core/bringup/config/{name}:ro"]
    arguments += [
        f"ap-pnc-tools:{native_arch()}",
        "-c",
        'source /opt/ros/humble/setup.bash; exec "$@"',
        "city-mission",
        str(container_job / snapshot.name),
        "--ros-args",
        "-p",
        f"mission:={container_job}/mission.yaml",
    ]
    subprocess.run(arguments, check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--launch", type=Path, help="launch an already prepared absolute job directory"
    )
    parser.add_argument("--scene", choices=("ring", "original"), default="ring")
    parser.add_argument("--rate-tuning", choices=("auto", "baseline", "body-z-pd3"), default="auto")
    args = parser.parse_args()
    if args.launch:
        launch(args.launch)
    else:
        prepare(args.scene, args.rate_tuning)
