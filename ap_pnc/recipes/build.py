import subprocess

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


@app.command()
def aero() -> None:
    """Build aerodynamics package (colcon)."""
    setup_env()
    git_root = get_git_root()
    script = git_root / "scripts" / "tools" / "build"
    subprocess.run(
        [str(script), "--packages-select", "aerodynamics", "--allow-overriding", "aerodynamics"],
        check=True,
    )


@app.command(name="traj-server")
def traj_server() -> None:
    """Build trajectory_server package (colcon)."""
    setup_env()
    git_root = get_git_root()
    script = git_root / "scripts" / "tools" / "build"
    subprocess.run(
        [str(script), "--packages-select", "trajectory_server", "--allow-overriding", "trajectory_server"],
        check=True,
    )


@app.command()
def all_() -> None:
    """Build all C++ packages."""
    aero()
    traj_server()
