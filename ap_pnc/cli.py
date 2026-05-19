import typer

from ap_pnc.recipes import (
    aerodyn,
    benchmark,
    build,
    cocp,
    docker,
    install,
    lint,
    nn_trim,
    test,
    traj,
)

app = typer.Typer(no_args_is_help=True, help="AP-PnC tailsitter project CLI.")

app.add_typer(install.app, name="install", help="Install everything (venv, acados, deps)")
app.add_typer(cocp.app, name="cocp", help="Generate acados OCP C code")
app.add_typer(traj.app, name="traj", help="Trajectory visualization and tools")
app.add_typer(aerodyn.app, name="aerodyn", help="Aerodynamics data generation, viz, ML")
app.add_typer(benchmark.app, name="benchmark", help="Compute timing benchmarks")
app.add_typer(test.app, name="test", help="Test runners")
app.add_typer(build.app, name="build", help="C++ colcon builds")
app.add_typer(nn_trim.app, name="nn-trim", help="Neural network trim analysis")
app.add_typer(docker.app, name="docker", help="Docker compose commands")
app.add_typer(lint.app, name="lint", help="Lint and format")


def main() -> None:
    app()
