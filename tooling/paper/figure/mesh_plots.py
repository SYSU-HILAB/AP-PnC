"""Render frame-stamped aerodynamic meshes with explicit FLU moment axes.

The scalar fit families CLL, CM and CN refer to body Z, Y and X respectively.
All four plotting entry points share validation and labels; no output rotation
or unstamped-cache compatibility path is applied during plotting.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from tooling.env import artifact_path, project_path
from tooling.frame_contract import require_frame

MESH_KEYS = ("ALPHA_DEG", "BETA_DEG", "CL", "CD", "CY", "CLL", "CM", "CN")
PANELS = (
    (1, "CL", "Lift Coefficient"),
    (3, "CY", "Left-Side Force Coefficient"),
    (5, "CD", "Drag Coefficient"),
    (2, "CLL", "Body-Z Moment Coefficient"),
    (4, "CM", "Body-Y Moment Coefficient"),
    (6, "CN", "Body-X Moment Coefficient"),
)


def _validate_mesh(data):
    require_frame(data.get("frame_convention"))
    arrays = {key: np.asarray(data[key]) for key in MESH_KEYS}
    shape = arrays["ALPHA_DEG"].shape
    if len(shape) != 2 or min(shape) < 2:
        raise ValueError("mesh must contain at least a 2x2 grid")
    if any(array.shape != shape or not np.isfinite(array).all() for array in arrays.values()):
        raise ValueError("mesh arrays must be finite and have matching shapes")
    return arrays


def load_mesh_data(data_file, auto_generate=False, mesh_size=361, backend="jax"):
    """Load a trusted local NPY mesh, or generate a new frame-stamped mesh."""
    path = project_path(data_file)
    if path.exists():
        data = np.load(path, allow_pickle=True).item()
    else:
        if not auto_generate:
            raise FileNotFoundError(path)
        # New generated files must remain under the project artifact root.
        path = artifact_path(path)
        if backend == "jax":
            from tooling.paper.methods.lu.gd_model_jax import generate_mesh_data_jax as generate
        elif backend in ("torch", "pytorch"):
            from tooling.paper.methods.lu.gd_model_torch import generate_mesh_data_torch as generate
        elif backend in ("casadi", "casadi_symbolic"):
            from tooling.paper.methods.lu.gd_model_casadi import generate_mesh_data as generate
        else:
            raise ValueError(f"Unknown backend: {backend}")
        data = generate(filename=path.stem, cache_dir=str(path.parent), mesh_size=mesh_size)
    _validate_mesh(data)
    return data


def _plot_mesh(
    data_file, data, azimuth, show_plot, auto_generate, mesh_size, save_path, *, backend
):
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_mesh_data(data_file, auto_generate, mesh_size, backend)
    arrays = _validate_mesh(data)
    azimuth = float(azimuth)
    if not np.isfinite(azimuth):
        raise ValueError("azimuth must be finite")
    if save_path is None:
        name = Path(data_file).stem if data_file else f"mesh_aero_data_{backend}"
        save_path = Path(".artifacts/paper/figures/mesh") / f"{name}_azim{azimuth:.1f}.png"
    output = artifact_path(save_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15, 12))
    try:
        fig.suptitle(
            f"Aerodynamic Force and Moment Coefficients ({backend}, body FLU)", fontsize=16
        )
        for position, key, title in PANELS:
            axis = fig.add_subplot(3, 2, position, projection="3d")
            axis.view_init(elev=10, azim=azimuth)
            surface = axis.plot_surface(
                arrays["ALPHA_DEG"], arrays["BETA_DEG"], arrays[key], cmap="viridis", alpha=0.8
            )
            axis.set_xlabel("alpha [deg]")
            axis.set_ylabel("beta, positive left [deg]")
            axis.set_zlabel(key)
            axis.set_title(title)
            fig.colorbar(surface, ax=axis, shrink=0.5)
        fig.tight_layout()
        fig.savefig(output, dpi=150, bbox_inches="tight")
        if show_plot:
            plt.show()
    finally:
        plt.close(fig)
    print(f"Plot saved to: {output}")
    return data


def plot_aerodynamic_mesh(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """Plot a numeric coefficient mesh; missing data uses the CasADi model."""
    return _plot_mesh(
        data_file, data, azimuth, show_plot, auto_generate, mesh_size, save_path, backend="casadi"
    )


def plot_aerodynamic_mesh_jax(
    data_file=None,
    data=None,
    azimuth=125.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """Plot a JAX coefficient mesh."""
    return _plot_mesh(
        data_file, data, azimuth, show_plot, auto_generate, mesh_size, save_path, backend="jax"
    )


def plot_aerodynamic_mesh_torch(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """Plot a PyTorch coefficient mesh."""
    return _plot_mesh(
        data_file, data, azimuth, show_plot, auto_generate, mesh_size, save_path, backend="torch"
    )


def plot_aerodynamic_mesh_casadi(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """Plot a CasADi coefficient mesh."""
    return _plot_mesh(
        data_file, data, azimuth, show_plot, auto_generate, mesh_size, save_path, backend="casadi"
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Plot native-FLU aerodynamic mesh data")
    parser.add_argument("data_file", help="Project-root-relative or absolute NPY file")
    parser.add_argument("--azimuth", "-a", type=float, default=-127.5)
    parser.add_argument("--backend", "-b", choices=["torch", "jax", "casadi"], default="torch")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    _plot_mesh(
        args.data_file, None, args.azimuth, not args.no_show, True, 361, None, backend=args.backend
    )
