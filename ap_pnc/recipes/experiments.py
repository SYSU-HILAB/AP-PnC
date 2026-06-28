import subprocess
import sys
from pathlib import Path

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer(help="Run wind and other performance experiments.")


@app.command()
def wind(
    ctx: typer.Context,
    velocities: str = typer.Option(
        "12.0",
        "--velocities",
        help="Space-separated target velocities in m/s",
    ),
    side_min: float = typer.Option(-4.0, "--side-min", help="Minimum side wind in m/s"),
    side_max: float = typer.Option(4.0, "--side-max", help="Maximum side wind in m/s"),
    side_steps: int = typer.Option(17, "--side-steps", help="Number of side wind steps"),
    long_min: float = typer.Option(-6.0, "--long-min", help="Minimum longitudinal wind in m/s"),
    long_max: float = typer.Option(6.0, "--long-max", help="Maximum longitudinal wind in m/s"),
    long_steps: int = typer.Option(25, "--long-steps", help="Number of longitudinal wind steps"),
    output_dir: str = typer.Option(
        "artifacts/simulations/wind_performance",
        "--output-dir",
        help="Output directory for results and plots",
    ),
    trajectory_dir: str = typer.Option(
        "artifacts/simulations/temp_data",
        "--trajectory-dir",
        help="Directory containing trajectory CSV files",
    ),
    direct: bool = typer.Option(
        False,
        "--direct",
        help="Skip tests, load existing results and generate plots",
    ),
    verbose: bool = typer.Option(False, "--verbose", help="Enable verbose output"),
) -> None:
    """
    Run wind disturbance performance tests for the MPC controller.

    Tests tracking error and success rates under various wind conditions.
    Converts body-frame wind specification to world-frame constant wind.
    """
    setup_env()
    root = get_git_root()

    script = root / "test" / "experiments" / "run_wind_performance_test.py"
    if not script.exists():
        typer.echo(f"[ERROR] Test script not found: {script}", err=True)
        raise typer.Exit(1)

    cmd = [
        sys.executable,
        str(script),
        "--output-dir",
        output_dir,
        "--trajectory-dir",
        trajectory_dir,
    ]

    if direct:
        cmd.append("--direct")
    if verbose:
        cmd.append("--verbose")

    # Velocity list
    for v in velocities.split():
        cmd.extend(["--velocities", v])

    cmd.extend(["--side-min", str(side_min)])
    cmd.extend(["--side-max", str(side_max)])
    cmd.extend(["--side-steps", str(side_steps)])
    cmd.extend(["--long-min", str(long_min)])
    cmd.extend(["--long-max", str(long_max)])
    cmd.extend(["--long-steps", str(long_steps)])

    typer.echo(f"Running wind experiment: {script}")
    result = subprocess.run(cmd, cwd=str(root))
    raise typer.Exit(result.returncode)


@app.command()
def gen_trajectory(
    config: str = typer.Option("", "--config", help="Path to TOML config file"),
    circle: bool = typer.Option(False, "--circle", help="Generate a circle trajectory"),
    num_segments: int = typer.Option(8, "--num-segments", help="Number of segments (for circle)"),
    segment_duration: float = typer.Option(2.0, "--segment-duration", help="Duration per segment [s]"),
    radius: float = typer.Option(30.0, "--radius", help="Circle radius [m]"),
    altitude: float = typer.Option(2.0, "--altitude", help="Flight altitude [m]"),
    output_dir: str = typer.Option("", "--output-dir", help="Override output directory"),
    filename: str = typer.Option("", "--filename", help="Override output filename"),
    dt: float = typer.Option(0.02, "--dt", help="Time step [s]"),
    a_max: float = typer.Option(15.0, "--a-max", help="Max tangential acceleration [m/s²]"),
) -> None:
    """
    Generate a minimal-jerk multi-segment trajectory CSV.

    Uses a piecewise quintic spline (minimum jerk) through waypoints.
    Output is compatible with TrajectoryReference and can be consumed
    by ``experiments wind``.
    """
    setup_env()
    root = get_git_root()

    from ap_pnc.lu_mpc.experiments.trajectory_gen import (
        TrajectoryGenConfig,
        generate_trajectory,
        make_circle_waypoints,
        make_default_config,
        trajectory_to_csv,
    )

    if config:
        cfg_path = Path(config)
        if not cfg_path.is_absolute():
            cfg_path = root / cfg_path
        cfg = TrajectoryGenConfig.from_toml(cfg_path)
    elif circle:
        wp = make_circle_waypoints(num_segments, radius, altitude)
        cfg = TrajectoryGenConfig(
            dt=dt,
            waypoints=wp,
            segment_durations=[segment_duration] * num_segments,
            a_max=a_max,
            output_dir=output_dir or "artifacts/simulations/temp_data",
            filename=filename or "minimal_jerk_trajectory.csv",
        )
    else:
        cfg = make_default_config()

    # Override output params if provided
    if output_dir:
        cfg.output_dir = output_dir
    if filename:
        cfg.filename = filename

    typer.echo(f"Generating trajectory: {len(cfg.waypoints)} waypoints, {len(cfg.segment_durations)} segments")
    traj = generate_trajectory(cfg)

    csv_path = root / cfg.output_dir / cfg.filename
    trajectory_to_csv(traj, csv_path)
    typer.echo(f"Done — trajectory saved to {csv_path}")


@app.command()
def list_trajectories(
    trajectory_dir: str = typer.Option(
        "artifacts/simulations/temp_data",
        "--trajectory-dir",
        help="Directory containing trajectory CSV files",
    ),
) -> None:
    """
    List available trajectory CSV files for experiments.
    """
    setup_env()
    root = get_git_root()
    traj_path = root / trajectory_dir

    if not traj_path.exists():
        typer.echo(f"[INFO] Trajectory directory not found: {traj_path}")
        raise typer.Exit(0)

    csv_files = sorted(traj_path.glob("*.csv"))
    if not csv_files:
        typer.echo(f"[INFO] No CSV files found in {traj_path}")
        raise typer.Exit(0)

    typer.echo(f"Available trajectories in {traj_path}:")
    for f in csv_files:
        typer.echo(f"  {f.name}")
