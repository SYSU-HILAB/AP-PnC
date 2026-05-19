import os
import subprocess
import sys

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


@app.command()
def run(
    aero_type: str = typer.Option("", "--aero-type", help="Aero type: lyu, bspline, phi, advanced"),
    mass: float = typer.Option(2.0, help="Mass [kg]"),
    v_max: float = typer.Option(13.0, help="Maximum velocity [m/s]"),
    num_points: int = typer.Option(130, help="Number of trim points"),
) -> None:
    """Run NN-based trim analysis."""
    setup_env()
    os.environ["JAX_PLATFORM_NAME"] = "cpu"
    git_root = get_git_root()
    script = git_root / "test" / "aerodynamics" / "test_nn_trim_analysis.py"
    args = [
        sys.executable,
        str(script),
        "--mass", str(mass),
        "--v-max", str(v_max),
        "--num-points", str(num_points),
    ]
    if aero_type:
        args.extend(["--aero-type", aero_type])
    subprocess.run(args, check=True)


@app.command()
def plot(
    mass: float = typer.Option(2.0, help="Mass [kg]"),
) -> None:
    """Plot existing NN trim CSV files."""
    setup_env()
    os.environ["JAX_PLATFORM_NAME"] = "cpu"
    git_root = get_git_root()
    script = git_root / "test" / "aerodynamics" / "test_nn_trim_analysis.py"
    subprocess.run(
        [sys.executable, str(script), "--mass", str(mass), "--plot-only"],
        check=True,
    )
