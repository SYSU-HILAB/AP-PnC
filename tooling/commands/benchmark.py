from pathlib import Path

import typer
import yaml

from tooling.commands import stack
from tooling.env import project_path
from tooling.paper.benchmark import runner
from tooling.paper.benchmark.runner import TRAJ_TYPES

app = typer.Typer(help="Trajectory-tracking benchmark experiments (planner -> controller -> data).")


def _sync_planning_config(v_max: float, traj_type: int, radius: float) -> Path:
    """Write experiment parameters into the flight planning config (tracked source).

    The gazebo stack bind-mounts ``core/bringup``; this intentionally edits that
    configuration in place. Run-local overrides (simple_sim) never touch it.
    """
    yaml_path = project_path("core/bringup/config/planning.yaml")
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
        f"Synced {yaml_path.name}: type={TRAJ_TYPES.get(traj_type, traj_type)}, "
        f"radius={radius}, v_max={v_max}"
    )
    return yaml_path


@app.command()
def run(
    v_max: float = typer.Option(..., "--vmax", help="Max velocity [m/s]"),
    traj_type: int = typer.Option(1, "--type", help=f"Trajectory type: {TRAJ_TYPES}"),
    radius: float = typer.Option(4.0, "--radius", help="Trajectory radius [m]"),
    dt: float = typer.Option(1e-3, "--dt", help="Simulation step [s]"),
    control_dt: float = typer.Option(0.01, "--control-dt", help="Controller period [s]"),
    settle: float = typer.Option(2.0, "--settle", help="Log from this time onward [s]"),
    output: str = typer.Option(
        "", "--output", "-o", help="Output directory (default: .artifacts/benchmark/<name>)"
    ),
) -> None:
    """Offline benchmark: plan a trajectory, run the SE3 controller, write data.

    Fully in-process (no ROS): the pure planning core is called through
    planner_bindings, then the SE3 controller tracks it in simulation.
    """
    if traj_type not in TRAJ_TYPES:
        typer.echo(f"Unknown trajectory type {traj_type}. Choose from: {TRAJ_TYPES}", err=True)
        raise typer.Exit(1)

    cfg = runner.BenchmarkConfig(
        v_max=v_max,
        traj_type=traj_type,
        radius=radius,
        dt=dt,
        control_dt=control_dt,
        settle_time=settle,
        output_dir=Path(output) if output else None,
    )
    result = runner.run(cfg)

    typer.echo(f"Run dir : {result['output_dir']}")
    typer.echo(f"Data    : {result['data']}")
    typer.echo("Metrics :")
    for key, value in result["metrics"].items():
        typer.echo(f"  {key:12} {value:.6g}")


@app.command()
def flight(
    v_max: float = typer.Option(..., "--vmax", help="Max velocity [m/s]"),
    traj_type: int = typer.Option(1, "--type", help=f"Trajectory type: {TRAJ_TYPES}"),
    radius: float = typer.Option(4.0, "--radius", help="Trajectory radius [m]"),
    launch: bool = typer.Option(True, "--launch/--no-launch", help="Start the stack afterwards"),
) -> None:
    """Configure the planning experiment, then start the full local ROS stack."""
    if traj_type not in TRAJ_TYPES:
        typer.echo(f"Unknown trajectory type {traj_type}. Choose from: {TRAJ_TYPES}", err=True)
        raise typer.Exit(1)
    _sync_planning_config(v_max, traj_type, radius)
    if launch:
        stack.start()
