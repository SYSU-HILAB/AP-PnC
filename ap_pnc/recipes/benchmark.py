from pathlib import Path

import typer
import yaml

from ap_pnc.env import get_git_root, setup_env
from ap_pnc.recipes import stack

app = typer.Typer(help="Differential-flatness planning benchmark experiments.")

TRAJ_TYPES = {1: "Lemniscate", 2: "Circle", 3: "Sinusoidal", 4: "Line"}


def _sync_planning_config(v_max: float, traj_type: int, radius: float) -> Path:
    """Write experiment parameters into bringup/config/planning.yaml."""
    git_root = get_git_root()
    yaml_path = git_root / "bringup" / "config" / "planning.yaml"
    if not yaml_path.exists():
        typer.echo(f"Planning config not found: {yaml_path}", err=True)
        raise typer.Exit(1)

    data = yaml.safe_load(yaml_path.read_text()) or {}
    problem = data.setdefault("problem_formulation", {})
    problem.setdefault("basic_traj", {})["type"] = traj_type
    problem["basic_traj"]["radius"] = radius
    problem.setdefault("cost", {})["v_max"] = v_max
    yaml_path.write_text(yaml.safe_dump(data, sort_keys=False))

    typer.echo(
        f"Synced {yaml_path.relative_to(git_root)}: "
        f"type={TRAJ_TYPES.get(traj_type, traj_type)}, radius={radius}, v_max={v_max}"
    )
    return yaml_path


@app.command()
def run(
    v_max: float = typer.Option(..., "--vmax", help="Max velocity [m/s]"),
    traj_type: int = typer.Option(1, "--type", help=f"Trajectory type: {TRAJ_TYPES}"),
    radius: float = typer.Option(4.0, "--radius", help="Trajectory radius [m]"),
    launch: bool = typer.Option(True, "--launch/--no-launch", help="Start the stack afterwards"),
) -> None:
    """Configure the planning experiment, then start the full local stack."""
    setup_env()
    if traj_type not in TRAJ_TYPES:
        typer.echo(f"Unknown trajectory type {traj_type}. Choose from: {TRAJ_TYPES}", err=True)
        raise typer.Exit(1)
    _sync_planning_config(v_max, traj_type, radius)
    if launch:
        stack.start()
