import subprocess
import sys

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()
ml_app = typer.Typer(help="Aero ML pipeline commands")
app.add_typer(ml_app, name="ml")

_ENGINES = ["jax", "casadi", "torch"]


@app.command()
def gen(
    engine: str = typer.Option("jax", help="Compute engine: jax, casadi, torch"),
    type_: str = typer.Option(
        "coordinated", "--type", help="Data type: coordinated, mesh"
    ),
    size: int = typer.Option(361, help="Mesh size (for mesh type)"),
) -> None:
    """Generate aerodynamic data."""
    if engine not in _ENGINES:
        typer.echo(f"Error: engine must be one of {_ENGINES}", err=True)
        raise typer.Exit(1)

    setup_env()

    if type_ == "coordinated":
        if engine == "jax":
            code = "from feature.lu_mpc.model.sim.gd_model_jax import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"
        elif engine == "casadi":
            code = "from feature.lu_mpc.model.sim.gd_model_casadi import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"
        else:
            code = "from feature.lu_mpc.model.sim.gd_model_torch import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"
    elif type_ == "mesh":
        if engine == "jax":
            code = f"from feature.lu_mpc.model.sim.gd_model_jax import generate_mesh_data_jax; generate_mesh_data_jax(mesh_size={size})"
        elif engine == "casadi":
            code = "from feature.lu_mpc.model.sim.gd_model_casadi import generate_mesh_data; generate_mesh_data()"
        else:
            code = f"from feature.lu_mpc.model.sim.gd_model_torch import generate_mesh_data_torch; generate_mesh_data_torch(mesh_size={size})"
    else:
        typer.echo(f"Error: type must be coordinated or mesh", err=True)
        raise typer.Exit(1)

    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def viz(
    type_: str = typer.Option("all", "--type", help="Plot type: all, mesh, single"),
    engine: str = typer.Option("jax", help="Engine for mesh plots: jax, casadi, torch"),
    azimuth: str = typer.Option("-127.5", help="Azimuth angle for mesh plots"),
    no_show: bool = typer.Option(False, help="Save plots without displaying"),
) -> None:
    """Visualize aerodynamic data."""
    setup_env()
    git_root = get_git_root()

    if type_ == "all":
        data_file = (
            git_root / "feature" / "lu_mpc" / ".cache" / "all_aero_coeffs_data.npy"
        )
        code = f"from feature.lu_mpc.plotting.aero_plots import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs('{data_file}')"
        subprocess.run([sys.executable, "-c", code], check=True)
    elif type_ == "mesh":
        if engine not in _ENGINES:
            typer.echo(f"Error: engine must be one of {_ENGINES}", err=True)
            raise typer.Exit(1)
        cache_name = f"mesh_aero_data{'_torch' if engine == 'torch' else '_jax' if engine == 'jax' else '_casadi'}.npy"
        data_file = git_root / "feature" / "lu_mpc" / ".cache" / cache_name
        func_map = {
            "jax": "plot_aerodynamic_mesh_jax",
            "torch": "plot_aerodynamic_mesh_torch",
            "casadi": "plot_aerodynamic_mesh_casadi",
        }
        extra = ", show_plot=False" if no_show else ""
        code = (
            f"from feature.lu_mpc.plotting.mesh_plots import {func_map[engine]}; "
            f"{func_map[engine]}('{data_file}', azimuth='{azimuth}'{extra})"
        )
        subprocess.run([sys.executable, "-c", code], check=True)
    elif type_ == "single":
        data_file = (
            git_root / "feature" / "lu_mpc" / ".cache" / "coordinated_aero_data_jax.npy"
        )
        subprocess.run(
            [
                sys.executable,
                "-m",
                "feature.lu_mpc.plotting.aero_plots",
                str(data_file),
                "--no-show",
            ],
            check=True,
        )
    else:
        typer.echo(f"Error: type must be all, mesh, or single", err=True)
        raise typer.Exit(1)


@ml_app.command()
def train(
    aero_type: str = typer.Option("lyu", help="Aero type: lyu, bspline, phi, advanced"),
) -> None:
    """Train MLP model for a specific aero type."""
    os.environ["JAX_PLATFORM_NAME"] = "cpu"
    code = f"from feature.aerodynamics.training.trainer import train_single_aero_type; train_single_aero_type('{aero_type}')"
    subprocess.run([sys.executable, "-c", code], check=True)


@ml_app.command()
def pipeline() -> None:
    """Assemble ML dataset + train all MLP models."""
    setup_env()
    os.environ["JAX_PLATFORM_NAME"] = "cpu"
    git_root = get_git_root()
    code = (
        "from feature.aerodynamics.data.dataset import assemble_parquet_dataset; "
        "from feature.aerodynamics.training.trainer import train_all_aero_types; "
        "import os; "
        f"csv_dir = '{git_root / 'artifacts' / 'simulations' / 'ml_aero_data'}'; "
        f"output = '{git_root / 'artifacts' / 'simulations' / 'aero_ml_dataset.parquet'}'; "
        "os.makedirs(os.path.dirname(output), exist_ok=True); "
        "assemble_parquet_dataset(csv_dir, output); "
        "train_all_aero_types()"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


@app.command()
def show(
    aero_type: str = typer.Option(
        "", "--aero-type", help="Specific aero type to compare"
    ),
) -> None:
    """Compare derivatives across aero types."""
    setup_env()
    script = get_git_root() / "scripts" / "tools" / "compare_aero_derivatives.py"
    args = [sys.executable, str(script)]
    if aero_type:
        args.extend(["--aero-type", aero_type])
    else:
        args.append("--all")
    subprocess.run(args, check=True)


@app.command(name="direct-show")
def direct_show() -> None:
    """Plot raw CSV aero data."""
    setup_env()
    script = get_git_root() / "test" / "aerodynamics" / "test_plot_csv_data.py"
    subprocess.run([sys.executable, str(script)], check=True)


@app.command(name="csv-vs-model")
def csv_vs_model() -> None:
    """Plot CSV data vs trained model comparison."""
    setup_env()
    script = get_git_root() / "test" / "aerodynamics" / "test_csv_vs_model_plot.py"
    subprocess.run([sys.executable, str(script)], check=True)
