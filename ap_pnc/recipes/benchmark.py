import subprocess
import sys

import typer

from ap_pnc.env import setup_env

app = typer.Typer()


@app.command()
def jax() -> None:
    """Benchmark JAX compute timing."""
    setup_env()
    code = "from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"
    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def torch() -> None:
    """Benchmark Torch compute timing."""
    setup_env()
    code = "from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"
    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def compare(
    mesh_size: int = typer.Argument(361, help="Mesh size for timing comparison"),
    num_runs: int = typer.Argument(10, help="Number of timing runs"),
) -> None:
    """Compare compute timing across JAX/Torch/CasADi."""
    setup_env()
    script = get_git_root() / "scripts" / "compare_timing.py"
    subprocess.run([sys.executable, str(script), str(mesh_size), str(num_runs)], check=True)


def get_git_root():
    from ap_pnc.env import get_git_root as _get_git_root
    return _get_git_root()
