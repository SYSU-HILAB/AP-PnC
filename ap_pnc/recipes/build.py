import os
import shutil
import subprocess

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


def _colcon_env() -> dict[str, str]:
    """Build env for colcon: ROS2 sourced, venv vars cleaned, system Python."""
    setup_env(ros=True)
    env = os.environ.copy()
    for var in [
        "VIRTUAL_ENV", "CONDA_PREFIX", "CONDA_SHLVL",
        "CONDA_DEFAULT_ENV", "CONDA_PROMPT_MODIFIER", "PYTHONPATH",
    ]:
        env.pop(var, None)
    cleaned = []
    for d in env.get("PATH", "").split(":"):
        if not any(k in d for k in ("env", "venv", ".virtualenv", "conda", "anaconda", "miniconda", "IsaacLab")):
            cleaned.append(d)
    env["PATH"] = ":".join(cleaned)
    return env


def _colcon_build(packages: list[str], clean: bool = False) -> None:
    git_root = get_git_root()
    env = _colcon_env()
    if clean:
        for d in ("build", "install", "log"):
            shutil.rmtree(git_root / d, ignore_errors=True)
    cmd = ["colcon", "build", "--symlink-install"]
    for pkg in packages:
        cmd.extend(["--packages-select", pkg, "--allow-overriding", pkg])
    subprocess.run(cmd, cwd=git_root, env=env, check=True)


@app.command()
def aero(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build aerodynamics package (colcon)."""
    _colcon_build(["aerodynamics"], clean=clean)


@app.command(name="traj-server")
def traj_server(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build trajectory_server package (colcon)."""
    _colcon_build(["trajectory_server"], clean=clean)


@app.command()
def all_(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build all C++ packages."""
    _colcon_build(["aerodynamics", "trajectory_server"], clean=clean)
