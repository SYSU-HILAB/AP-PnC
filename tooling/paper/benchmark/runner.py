"""Benchmark orchestrator: config -> plan -> simulate -> data -> metrics.

This is the single entry point the CLI calls. It composes the whole offline
pipeline; nothing here depends on ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from tooling.env import artifact_path, get_git_root

from . import data, metrics as metrics_mod
from .controllers import SE3Controller, SE3Gains
from .plant import PlantConfig, QuadPlant
from .simulate import simulate
from .sources import planner as planner_source

TRAJ_TYPES = {1: "lemniscate", 2: "circle", 3: "sin", 4: "line"}


@dataclass
class BenchmarkConfig:
    """Inputs of one benchmark run."""

    v_max: float
    traj_type: int = 1
    radius: float = 4.0
    output_dir: Path | None = None
    dt: float = 1e-3
    control_dt: float = 0.01
    settle_time: float = 2.0
    mass: float = 1.0
    gains: SE3Gains | None = None


def _base_planning_yaml() -> Path:
    """Locate the repo's canonical planning yaml (repo layout or container)."""
    root = get_git_root()
    for candidate in (
        root / "core" / "bringup" / "config" / "planning.yaml",
        root / "bringup" / "config" / "planning.yaml",
    ):
        if candidate.exists():
            return candidate
    raise FileNotFoundError("planning.yaml not found under core/bringup/config")


def write_planning_yaml(cfg: BenchmarkConfig, out_path: Path) -> Path:
    """Derive a run-local planning yaml from the repo defaults + CLI overrides."""
    out_path = artifact_path(out_path)
    base = yaml.safe_load(_base_planning_yaml().read_text()) or {}
    problem = base.setdefault("problem_formulation", {})
    problem.setdefault("basic_traj", {})["type"] = cfg.traj_type
    problem["basic_traj"]["radius"] = cfg.radius
    problem.setdefault("cost", {})["v_max"] = cfg.v_max
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(base, sort_keys=False))
    return out_path


def _default_output_dir(cfg: BenchmarkConfig) -> Path:
    name = f"{TRAJ_TYPES.get(cfg.traj_type, cfg.traj_type)}_vel{cfg.v_max:g}_r{cfg.radius:g}"
    return get_git_root() / ".artifacts" / "benchmark" / name


def run(cfg: BenchmarkConfig) -> dict:
    """Run the full offline pipeline and return paths + metrics."""
    out_dir = artifact_path(cfg.output_dir if cfg.output_dir else _default_output_dir(cfg))
    out_dir.mkdir(parents=True, exist_ok=True)

    yaml_path = write_planning_yaml(cfg, out_dir / "planning.yaml")

    # 1) real planned trajectory, in-process through the pure C++ core
    ref = planner_source.plan(yaml_path)

    # 2) controller + plant
    plant = QuadPlant(PlantConfig(mass=cfg.mass))
    controller = SE3Controller(
        mass=cfg.mass,
        inertia=plant.cfg.inertia,
        gravity=plant.cfg.gravity,
        gains=cfg.gains,
    )

    # 3) closed loop
    records = simulate(
        ref,
        controller,
        plant,
        dt=cfg.dt,
        control_dt=cfg.control_dt,
    )

    # 4) metrics + data
    metrics = metrics_mod.tracking_metrics(records, settle_time=cfg.settle_time)
    meta = {
        "traj_type": cfg.traj_type,
        "traj_type_name": TRAJ_TYPES.get(cfg.traj_type, str(cfg.traj_type)),
        "v_max": cfg.v_max,
        "radius": cfg.radius,
        "dt": cfg.dt,
        "control_dt": cfg.control_dt,
        "settle_time": cfg.settle_time,
        "mass": cfg.mass,
        "controller": "se3",
        "reference_points": int(ref.t.size),
        "reference_duration": ref.duration,
    }

    data_path = data.save_npz(out_dir / "data.npz", records, meta=meta)
    data.save_json(out_dir / "metrics.json", {**meta, **metrics})

    return {
        "output_dir": out_dir,
        "planning_yaml": yaml_path,
        "data": data_path,
        "metrics": metrics,
        "meta": meta,
    }
