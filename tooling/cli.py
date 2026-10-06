import sys

import typer

from tooling.commands import (
    benchmark,
    build,
    docker,
    gen_nmpc_lib,
    lint,
    sim,
    telemetry,
    test,
)

app = typer.Typer(no_args_is_help=True, help="AP-PnC tailsitter project CLI.")

app.add_typer(
    gen_nmpc_lib.app, name="gen-nmpc-lib", help="Generate acados NMPC code and bundle lib"
)
app.add_typer(benchmark.app, name="benchmark", help="Planning benchmark experiments")
app.add_typer(test.app, name="test", help="Test runners (pytest, colcon)")
app.add_typer(build.app, name="build", help="C++ colcon builds")
app.add_typer(sim.app, name="sim", help="Simulation management (K3s ConfigMap-driven)")
app.add_typer(docker.app, name="docker", help="Docker compose commands")
app.add_typer(lint.app, name="lint", help="Lint and format")
app.add_typer(telemetry.app, name="telemetry", help="Local OpenTelemetry collector and probes")


@app.command(name="start")
def start(
    target: str = typer.Option(
        "simple",
        "--target",
        "-t",
        help="Target to start: 'simple' (simple_sim via K3s), 'gazebo' (PX4 Gazebo SITL via K3s), 'stop' (stop sim)",
    ),
) -> None:
    """Start project runtime (simple, gazebo or stop)."""
    if target in ("simple", "simple-sim"):
        sim.simple()
    elif target == "gazebo":
        sim.gazebo()
    elif target in ("stop", "down"):
        sim.stop()
    else:
        typer.echo(
            f"Unknown target '{target}'. Choose 'simple', 'gazebo' or 'stop'.", err=True
        )
        raise typer.Exit(1)


def main() -> None:
    from tooling.observability import operation, session

    commands = {
        "start",
        "gen-nmpc-lib",
        "benchmark",
        "test",
        "build",
        "sim",
        "docker",
        "lint",
        "telemetry",
    }
    command = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in commands else "ap-pnc"
    with session(), operation("cli.command", {"ap.pnc.command": command}):
        app()
