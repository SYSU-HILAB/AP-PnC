import os
import subprocess
import sys

import typer

from tooling.env import get_git_root, project_path, setup_env

app = typer.Typer()


def _pytest(path: str, plugin_disable: bool = True) -> None:
    setup_env()
    env = os.environ.copy()
    if plugin_disable:
        env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    root = get_git_root()
    outputs = root / ".artifacts" / "tests"
    outputs.mkdir(parents=True, exist_ok=True)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    subprocess.run(
        [sys.executable, "-m", "pytest", str(project_path(path)),
         "--basetemp", str(outputs / "pytest_tmp"),
         "-o", f"cache_dir={outputs / 'pytest_cache'}"],
        cwd=root, env=env, check=True,
    )


def _run_script(rel_path: str, extra_env: dict | None = None) -> None:
    setup_env()
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    subprocess.run(
        [sys.executable, str(project_path(rel_path))],
        cwd=get_git_root(), env=env, check=True,
    )


@app.command()
def all_() -> None:
    """Run all lu_mpc tests."""
    _pytest("test/lu_mpc")


@app.command()
def vmap() -> None:
    """Run JAX vmap tests."""
    _pytest("test/lu_mpc/model/sim/test_gd_model_jax.py")


@app.command(name="jax-trim")
def jax_trim() -> None:
    """Run JAX trim analysis."""
    _run_script("test/lu_mpc/analysis/user_test_jax_trim.py", {"JAX_PLATFORM_NAME": "cpu"})


@app.command()
def casadi() -> None:
    """Run CasADi symbolic model script."""
    _run_script("tooling/paper/methods/lu/gd_model_casadi.py")


@app.command(name="frame-conversion")
def frame_conversion() -> None:
    """Run frame conversion test."""
    code = "from tooling.paper.methods.lu.gd_model_casadi import test_frame_conversion; test_frame_conversion()"
    setup_env()
    subprocess.run([sys.executable, "-c", code], cwd=get_git_root(), check=True)


@app.command(name="colcon")
def colcon() -> None:
    """Run colcon test for the ROS2 workspace."""
    setup_env(ros=True)
    git_root = get_git_root()
    base = git_root / ".artifacts" / "colcon"
    subprocess.run(
        ["colcon", "--log-base", str(base / "log"), "test", "--merge-install",
         "--build-base", str(base / "build"),
         "--install-base", str(base / "install")],
        cwd=git_root,
        check=True,
    )


@app.command(name="colcon-result")
def colcon_result() -> None:
    """Show colcon test results."""
    setup_env(ros=True)
    git_root = get_git_root()
    base = git_root / ".artifacts" / "colcon"
    subprocess.run(
        ["colcon", "--log-base", str(base / "log"), "test-result", "--test-result-base", str(base / "build")],
        cwd=git_root,
        check=True,
    )
