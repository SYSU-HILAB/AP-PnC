import subprocess
import sys

import typer

from ap_pnc.env import setup_env

app = typer.Typer()


@app.command()
def jax() -> None:
    """Benchmark JAX compute timing."""
    setup_env()
    code = "from ap_pnc.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"
    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def torch() -> None:
    """Benchmark Torch compute timing."""
    setup_env()
    code = "from ap_pnc.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"
    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def compare() -> None:
    """Run timing benchmarks across all backends (JAX, Torch, CasADi)."""
    setup_env()
    codes = [
        "from ap_pnc.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()",
        "from ap_pnc.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()",
    ]
    for code in codes:
        subprocess.run([sys.executable, "-c", code], check=True)


def get_git_root():
    from ap_pnc.env import get_git_root as _get_git_root
    return _get_git_root()
