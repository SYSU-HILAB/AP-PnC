import os
import shutil
import subprocess
import sys

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


def _colcon_build(
    packages: list[str], paths: list[str] | None = None, clean: bool = False
) -> None:
    git_root = get_git_root()
    env = _colcon_env()
    base = git_root / ".artifacts" / "colcon"
    if clean:
        shutil.rmtree(base, ignore_errors=True)
    env["COLCON_LOG_PATH"] = str(base / "log")
    cmd = [
        "colcon", "build", "--symlink-install", "--merge-install",
        "--build-base", str(base / "build"),
        "--install-base", str(base / "install"),
    ]
    if paths:
        cmd.extend(["--base-paths", *paths])
    if packages:
        for pkg in packages:
            cmd.extend(["--packages-select", pkg, "--allow-overriding", pkg])
    # --cmake-args is greedy and must stay last
    cmd.extend(["--cmake-args", f"-DCMAKE_PREFIX_PATH={base / 'install'}"])
    subprocess.run(cmd, cwd=git_root, env=env, check=True)


@app.command()
def aero(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build aerodynamics package (colcon)."""
    _colcon_build(
        ["aerodynamics"],
        paths=["simulations/aerodynamics", "ros_packages"],
        clean=clean,
    )


@app.command(name="traj-server")
def traj_server(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build trajectory_server package (colcon)."""
    _colcon_build(
        ["trajectory_server"],
        paths=["ros_packages"],
        clean=clean,
    )


@app.command(name="simple-sim")
def simple_sim(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build simple_sim package (colcon)."""
    _colcon_build(
        ["simple_sim"],
        paths=["ros_packages", "simulations/aerodynamics", "simulations/simple_sim/src"],
        clean=clean,
    )


@app.command(name="cnmpc")
def cnmpc(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build c_nmpc and px4_interface packages (colcon)."""
    _colcon_build(
        ["c_nmpc", "px4_interface"],
        paths=["ros_packages"],
        clean=clean,
    )


@app.command()
def all_(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build the whole workspace (all discovered packages)."""
    _colcon_build(
        [],
        paths=["ros_packages", "bringup", "simulations/aerodynamics",
               "simulations/simple_sim/src"],
        clean=clean,
    )


@app.command()
def bindings() -> None:
    """Build aerodynamics pybind11 bindings into .artifacts/bindings."""
    git_root = get_git_root()
    bindings_dir = git_root / ".artifacts" / "bindings"
    bindings_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [sys.executable, "setup.py", "build_ext", "--build-lib", str(bindings_dir)],
        cwd=git_root / "simulations" / "aerodynamics" / "python",
        check=True,
    )
    so_files = list(bindings_dir.glob("aerodynamics_bindings*.so"))
    if not so_files:
        typer.echo("ERROR: no aerodynamics_bindings .so produced", err=True)
        raise typer.Exit(1)
    for f in so_files:
        typer.echo(f"Bindings: {f}")
