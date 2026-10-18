"""Numerical and ROS tests run in Docker; uv only dispatches the commands."""

import typer

from tooling import container_tools

app = typer.Typer()
ROOT = container_tools.CONTAINER_ROOT


def _pytest(path: str, *, arch: str = "native") -> None:
    outputs = ROOT / ".artifacts/tests"
    container_tools.run(
        ["env", "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1", "python3", "-m", "pytest",
         str(ROOT / path), "--basetemp", str(outputs / "pytest_tmp"),
         "-o", f"cache_dir={outputs / 'pytest_cache'}"],
        arch=arch,
    )


def _run_script(path: str, *, arch: str = "native") -> None:
    container_tools.run(
        ["env", "JAX_PLATFORM_NAME=cpu", "python3", str(ROOT / path)], arch=arch
    )


@app.command(name="all")
def all_(arch: str = "native") -> None:
    """Run lu_mpc numerical tests in Docker."""
    _pytest("test/lu_mpc", arch=arch)


@app.command()
def vmap(arch: str = "native") -> None:
    """Run JAX vmap tests in Docker."""
    _pytest("test/lu_mpc/model/sim/test_gd_model_jax.py", arch=arch)


@app.command(name="jax-trim")
def jax_trim(arch: str = "native") -> None:
    """Run JAX trim analysis in Docker."""
    _run_script("test/lu_mpc/analysis/user_test_jax_trim.py", arch=arch)


@app.command()
def casadi(arch: str = "native") -> None:
    """Run the CasADi symbolic model in Docker."""
    _run_script("tooling/paper/methods/lu/gd_model_casadi.py", arch=arch)


@app.command(name="frame-conversion")
def frame_conversion(arch: str = "native") -> None:
    """Run the numerical frame conversion test in Docker."""
    code = (
        "from tooling.paper.methods.lu.gd_model_casadi import test_frame_conversion; "
        "test_frame_conversion()"
    )
    container_tools.run(["python3", "-c", code], arch=arch)


@app.command(name="colcon")
def colcon(arch: str = "native") -> None:
    """Run tests in the architecture-specific Docker ROS workspace."""
    base = ROOT / ".artifacts/colcon"
    container_tools.run(
        ["colcon", "--log-base", str(base / "log"), "test", "--merge-install",
         "--build-base", str(base / "build"), "--install-base", str(base / "install")],
        arch=arch,
    )


@app.command(name="colcon-result")
def colcon_result(arch: str = "native") -> None:
    """Read test results from the architecture-specific Docker workspace."""
    base = ROOT / ".artifacts/colcon"
    container_tools.run(
        ["colcon", "--log-base", str(base / "log"), "test-result",
         "--test-result-base", str(base / "build")],
        arch=arch,
    )
