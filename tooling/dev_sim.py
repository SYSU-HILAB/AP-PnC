"""ROS2 development simulation, deliberately separate from benchmark jobs."""

import json
import math
import shutil
import subprocess
import time
import uuid
from pathlib import Path

from tooling.env import artifact_path, get_git_root, setup_env
from tooling.simple_stack import JobRequest, native_arch, write_configuration


def image():
    return f"ap-pnc-dev-sim:{native_arch()}"


def build():
    root = get_git_root()
    base = f"simple-sim:{native_arch()}"
    if subprocess.run(["docker", "image", "inspect", base], capture_output=True).returncode:
        raise RuntimeError("build the numerical engine first: ap-pnc build simple-sim")
    subprocess.run(["docker", "build", "--platform", f"linux/{native_arch()}", "-t", image(),
                    "-f", str(root / "infra/sim_infra/lidar_sim/Dockerfile"), str(root)],
                   env=setup_env(), check=True)


def prepare(*, controller="nmpc", v_max=4.0, output=None):
    if controller not in {"se3", "nmpc"}:
        raise ValueError("controller must be se3 or nmpc")
    if not math.isfinite(v_max) or v_max <= 0:
        raise ValueError("v_max must be finite and positive")
    root = get_git_root()
    profile = root / ".artifacts/vtol-matlab/profile.yaml"
    provenance = profile.with_name("provenance.json")
    if not profile.is_file() or not provenance.is_file():
        raise FileNotFoundError("practical profile unavailable; run stack.sh import-matlab first")
    job = artifact_path(output) if output is not None else artifact_path(
        f".artifacts/dev-sim/linux-{native_arch()}/{time.strftime('%Y%m%dT%H%M%S')}-{uuid.uuid4().hex[:8]}")
    if job.exists():
        raise FileExistsError(f"refusing to overwrite {job}")
    (job / "config").mkdir(parents=True)
    (job / "recordings").mkdir()
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml", "lidar_sim.yaml"):
        shutil.copy2(root / "core/bringup/config" / name, job / "config" / name)
    shutil.copy2(profile, job / "profile.yaml")
    shutil.copy2(provenance, job / "upstream-provenance.json")
    request = JobRequest(profile="practical", aero="lyu", v_max=v_max, hold_s=2.0,
                         overrides={"simple_sim.controller": controller,
                                    "simple_sim.ros.publish_stride": 1,
                                    "problem_formulation.basic_traj.radius": 8.0,
                                    "problem_formulation.basic_traj.height": 3.0})
    request.validate()
    write_configuration(job, request)
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True).strip())
    (job / "dev-sim.json").write_text(json.dumps({
        "schema": "ap-pnc/ros2-dev-sim/v1", "source_revision": revision, "source_dirty": dirty,
        "profile": "practical", "aero": "lyu", "controller": controller,
        "scene": "pillars", "ros_domain_id": 213, "hardware_connected": False,
        "collision_dynamics": False, "lidar_model": "analytic snapshot first returns",
        "container_name": f"ap-pnc-dev-{job.name}",
    }, indent=2))
    return job


def command(job: Path, *, rviz=False):
    if not job.is_absolute():
        raise ValueError("absolute prepared job directory required")
    job = artifact_path(job)
    metadata = json.loads((job / "dev-sim.json").read_text())
    if metadata.get("schema") != "ap-pnc/ros2-dev-sim/v1" or metadata.get("container_name") != f"ap-pnc-dev-{job.name}":
        raise ValueError("invalid development job metadata")
    # No host networking: this experiment cannot join an aircraft ROS domain.
    return ["docker", "run", "--rm", "--platform", f"linux/{native_arch()}",
            "--ipc=host", "--network", "none", "--name", metadata["container_name"],
            "-e", "AP_PNC_DIR=/workspace", "-e", "ROS_DOMAIN_ID=213",
            "-e", "ROS_LOCALHOST_ONLY=1",
            "-v", f"{job}/config:/workspace/core/bringup/config:ro",
            "-v", f"{job}/recordings:/workspace/.artifacts/benchmark:rw",
            image(), "ros2", "launch", "bringup", "dev_sim.launch.py",
            "config:=/workspace/core/bringup/config/simple_sim.yaml",
            "lidar_config:=/workspace/core/bringup/config/lidar_sim.yaml",
            f"rviz:={str(rviz).lower()}"]


def run(job):
    arguments = command(job)
    job = artifact_path(job)
    print(f"JOB={job}", flush=True)
    print("Starts paused; use docker exec in this container for /simple_sim/resume.", flush=True)
    try:
        subprocess.run(arguments, env=setup_env(), check=True)
    except KeyboardInterrupt:
        # Stop only this job's isolated container; never a host-global stack.
        name = json.loads((job / "dev-sim.json").read_text())["container_name"]
        subprocess.run(["docker", "stop", "--time", "5", name], capture_output=True)
