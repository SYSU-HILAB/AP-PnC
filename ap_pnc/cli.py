import typer

from ap_pnc.recipes import (
    aerodyn,
    benchmark,
    build,
    debug,
    docker,
    experiments,
    gen_nmpc_lib,
    install,
    lint,
    nn_trim,
    sim,
    stack,
    test,
    tracker,
    traj,
)

app = typer.Typer(no_args_is_help=True, help="AP-PnC tailsitter project CLI.")

app.add_typer(install.app, name="install", help="Install everything (venv, acados, deps)")
app.add_typer(
    gen_nmpc_lib.app, name="gen-nmpc-lib", help="Generate acados NMPC code and bundle lib"
)
app.add_typer(traj.app, name="traj", help="Trajectory visualization and tools")
app.add_typer(aerodyn.app, name="aerodyn", help="Aerodynamics data generation, viz, ML")
app.add_typer(benchmark.app, name="benchmark", help="Planning benchmark experiments")
app.add_typer(test.app, name="test", help="Test runners (pytest, colcon, timing)")
app.add_typer(build.app, name="build", help="C++ colcon builds")
app.add_typer(nn_trim.app, name="nn-trim", help="Neural network trim analysis")
app.add_typer(sim.app, name="sim", help="Simulation management (K3s ConfigMap-driven)")
app.add_typer(stack.app, name="stack", help="Local ROS2 stack lifecycle")
app.add_typer(debug.app, name="debug", help="Interactive debugging helpers")
app.add_typer(docker.app, name="docker", help="Docker compose commands")
app.add_typer(experiments.app, name="experiments", help="Run performance experiments (wind, etc.)")
app.add_typer(tracker.app, name="tracker", help="Tracker evaluation, open scenarios, and TUI")
app.add_typer(lint.app, name="lint", help="Lint and format")


@app.command(name="web")
def web_shortcut(
    scenario: str = typer.Option("open_circle", "--scenario", "-s", help="Initial scenario"),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
    host: str = typer.Option("127.0.0.1", "--host", "-h", help="Host interface to bind"),
    port: int = typer.Option(8080, "--port", "-p", help="Port to listen on"),
    no_browser: bool = typer.Option(False, "--no-browser", help="Don't open browser automatically"),
) -> None:
    """Launch the 3D Web simulation visualizer (FastAPI + Three.js)."""
    tracker.web_cmd(scenario=scenario, speed=speed, host=host, port=port, no_browser=no_browser)


@app.command(name="tui")
def tui_shortcut(
    scenario: str = typer.Option("open_circle", "--scenario", "-s", help="Initial scenario"),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
) -> None:
    """Launch the interactive terminal dashboard for tracker monitoring and orchestration."""
    tracker.tui_cmd(scenario=scenario, speed=speed)


@app.command(name="start")
def start(
    target: str = typer.Option(
        "simple",
        "--target",
        "-t",
        help="Target to start: 'simple' (simple_sim via K3s), 'gazebo' (PX4 Gazebo SITL via K3s), 'stop' (stop sim), or 'stack' (local ROS2 stack)",
    ),
) -> None:
    """Start project runtime (simple, gazebo, stop, or stack)."""
    if target in ("simple", "simple-sim"):
        sim.simple()
    elif target == "gazebo":
        sim.gazebo()
    elif target in ("stop", "down"):
        sim.stop()
    elif target == "stack":
        stack.start()
    else:
        typer.echo(
            f"Unknown target '{target}'. Choose 'simple', 'gazebo', 'stop', or 'stack'.", err=True
        )
        raise typer.Exit(1)


def main() -> None:
    app()
