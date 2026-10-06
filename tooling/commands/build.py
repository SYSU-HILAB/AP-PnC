import os
import shutil
import subprocess
import sys

import typer

from tooling import simple_stack
from tooling.env import get_git_root, setup_env

app = typer.Typer()


def _colcon_env() -> dict[str, str]:
    """Build env for colcon: ROS2 sourced, venv vars cleaned, system Python."""
    setup_env(ros=True)
    env = os.environ.copy()
    for var in [
        "VIRTUAL_ENV",
        "CONDA_PREFIX",
        "CONDA_SHLVL",
        "CONDA_DEFAULT_ENV",
        "CONDA_PROMPT_MODIFIER",
        "PYTHONPATH",
    ]:
        env.pop(var, None)
    cleaned = []
    for d in env.get("PATH", "").split(":"):
        if not any(
            k in d
            for k in ("env", "venv", ".virtualenv", "conda", "anaconda", "miniconda", "IsaacLab")
        ):
            cleaned.append(d)
    env["PATH"] = ":".join(cleaned)
    return env


def _colcon_build(
    packages: list[str],
    paths: list[str] | None = None,
    clean: bool = False,
    cmake_args: list[str] | None = None,
) -> None:
    git_root = get_git_root()
    env = _colcon_env()
    base = git_root / ".artifacts" / "colcon"
    if clean:
        shutil.rmtree(base, ignore_errors=True)
    env["COLCON_LOG_PATH"] = str(base / "log")
    cmd = [
        "colcon",
        "build",
        "--merge-install",
        "--build-base",
        str(base / "build"),
        "--install-base",
        str(base / "install"),
    ]
    if paths:
        cmd.extend(["--base-paths", *(str((git_root / p).resolve()) for p in paths)])
    if packages:
        cmd.extend(["--packages-select", *packages])
    # --cmake-args is greedy and must stay last
    cmd.extend(["--cmake-args", f"-DCMAKE_PREFIX_PATH={base / 'install'}", *(cmake_args or [])])
    subprocess.run(cmd, cwd=git_root, env=env, check=True)


@app.command()
def aero(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build aerodynamics package (colcon)."""
    _colcon_build(
        ["aerodynamics"],
        paths=["infra/sim_infra/aerodynamics", "core/ros_packages"],
        clean=clean,
    )


@app.command(name="planner")
def planner(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build the planner package (colcon)."""
    _colcon_build(
        ["planner"],
        paths=["core/ros_packages"],
        clean=clean,
    )


@app.command(name="simple-sim")
def simple_sim() -> None:
    """Build the whole offline stack: images, Rust dora nodes and the C++ simulator.

    This is the build the quick start needs. It runs in Docker, so no host ROS,
    Rust or acados installation is involved.
    """
    simple_stack.build()



@app.command(name="nmpc")
def nmpc(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build the control packages: nmpc (core) + px4ctrl (nav_infra)."""
    _colcon_build(
        ["nmpc", "px4ctrl"],
        paths=["core/ros_packages", "infra/nav_infra/ros_packages"],
        clean=clean,
    )


@app.command()
def all_(
    clean: bool = typer.Option(False, "--clean", "-c", help="Clean all build artifacts"),
) -> None:
    """Build the whole workspace (core + nav_infra code, all packages).

    NOTE: infra/nav_infra/bringup is deliberately NOT in the base paths —
    it is a same-name `bringup` package that lives only inside the
    nav_infra container (entry-environment isolation, never co-installed
    with core/bringup in one workspace).
    """
    _colcon_build(
        [],
        paths=[
            "core/ros_packages",
            "infra/nav_infra/ros_packages",
            "core/bringup",
            "infra/sim_infra/aerodynamics",
            "infra/sim_infra/simple_sim",
        ],
        clean=clean,
    )


@app.command()
def bindings() -> None:
    """Build aerodynamics pybind11 bindings into .artifacts/bindings."""
    git_root = get_git_root()
    bindings_dir = git_root / ".artifacts" / "bindings"
    bindings_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            str(git_root / "infra/sim_infra/aerodynamics/python/setup.py"),
            "build_ext",
            "--build-lib",
            str(bindings_dir),
            "--build-temp",
            str(bindings_dir / "build"),
        ],
        cwd=git_root / "infra" / "sim_infra" / "aerodynamics" / "python",
        check=True,
    )
    so_files = list(bindings_dir.glob("aerodynamics_bindings*.so"))
    if not so_files:
        typer.echo("ERROR: no aerodynamics_bindings .so produced", err=True)
        raise typer.Exit(1)
    for f in so_files:
        typer.echo(f"Bindings: {f}")


@app.command(name="planner-bindings")
def planner_bindings() -> None:
    """Build the pure planning-core pybind11 module into .artifacts/bindings.

    Compiles planner_core sources directly (no colcon/ROS); used by the
    offline benchmark pipeline.
    """
    git_root = get_git_root()
    bindings_dir = git_root / ".artifacts" / "bindings"
    bindings_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            str(git_root / "core/ros_packages/planner/python/setup.py"),
            "build_ext",
            "--build-lib",
            str(bindings_dir),
            "--build-temp",
            str(bindings_dir / "build"),
        ],
        cwd=git_root / "core" / "ros_packages" / "planner" / "python",
        check=True,
    )
    so_files = list(bindings_dir.glob("planner_bindings*.so"))
    if not so_files:
        typer.echo("ERROR: no planner_bindings .so produced", err=True)
        raise typer.Exit(1)
    for f in so_files:
        typer.echo(f"Bindings: {f}")
