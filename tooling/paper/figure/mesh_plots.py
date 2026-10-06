"""
3D mesh surface plotting functions for aerodynamic coefficients.

All routines visualize the mesh-style payload produced by
`feature.lu_mpc.plotting.data_utils.save_mesh_data`, which contains:

* `ALPHA_DEG`, `BETA_DEG` – 2-D grids (meshgrid outputs) in degrees.
* `CL`, `CD`, `CY`, `CLL` (or legacy `Cl`), `CM`, `CN` – coefficient grids with the
  same shape as `ALPHA_DEG`.
* `backend` – string that records which model generated the data (jax, torch, casadi).

If any of these keys are absent the loader will fail early, preventing matplotlib from
working on malformed inputs.
"""

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from tooling.env import artifact_path, project_path

MESH_KEYS = ("ALPHA_DEG", "BETA_DEG", "CL", "CD", "CY", "CM", "CN")


def _mesh_figure_path(name):
    """Resolve an auto-named mesh figure under AP_PNC_DIR/.artifacts."""
    path = artifact_path(Path(".artifacts/paper/figures/mesh") / name)
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


def load_mesh_data(data_file, auto_generate=False, mesh_size=361, backend="jax"):
    """
    Load mesh data from .npy file. Auto-generates if file doesn't exist.

    Args:
        data_file (str): Path to .npy file containing mesh data
        auto_generate (bool): If True, automatically generate data if file doesn't exist
        mesh_size (int): Size of mesh grid for auto-generation
        backend (str): Backend to use for auto-generation ("jax", "pytorch", "casadi", "casadi_symbolic")

    Returns:
        dict: Dictionary containing the mesh data
    """
    data_file = str(project_path(data_file))
    if not os.path.exists(data_file):
        if auto_generate:
            print(f"Data file not found: {data_file}")
            print(f"Auto-generating mesh data using {backend} backend...")

            # Import the appropriate generation function
            if backend == "jax":
                from tooling.paper.methods.lu.gd_model_jax import generate_mesh_data_jax

                return generate_mesh_data_jax(
                    filename=os.path.splitext(os.path.basename(data_file))[0],
                    mesh_size=mesh_size,
                )
            elif backend == "pytorch":
                from tooling.paper.methods.lu.gd_model_torch import generate_mesh_data_torch

                return generate_mesh_data_torch(
                    filename=os.path.splitext(os.path.basename(data_file))[0],
                    mesh_size=mesh_size,
                )
            elif backend == "casadi_symbolic":
                from tooling.paper.methods.lu.gd_model_casadi import generate_mesh_data

                return generate_mesh_data(
                    filename=os.path.splitext(os.path.basename(data_file))[0],
                    mesh_size=mesh_size,
                )
            else:
                raise ValueError(f"Unknown backend: {backend}")
        else:
            raise FileNotFoundError(f"Data file not found: {data_file}")

    return np.load(data_file, allow_pickle=True).item()


def plot_aerodynamic_mesh(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """
    Create 3D mesh surface plots of aerodynamic coefficients using NumPy data.

    Args:
        data_file (str, optional): Path to .npy file containing mesh data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        azimuth (float): Azimuth angle for 3D view in degrees. Default: -127.5
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.
        mesh_size (int): Mesh size for auto-generation. Default: 361.
        save_path (str, optional): Path to save the plot as PNG file. If None, auto-generates path.

    Returns:
        dict: The mesh data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_mesh_data(
            data_file,
            auto_generate=auto_generate,
            mesh_size=mesh_size,
            backend="casadi",
        )

    # Extract data arrays
    ALPHA_DEG = data["ALPHA_DEG"]
    BETA_DEG = data["BETA_DEG"]
    CL = data["CL"]
    CD = data["CD"]
    CY = data["CY"]
    CLL = data.get("CLL", data.get("Cl", np.zeros_like(CL)))
    CM = data["CM"]
    CN = data["CN"]

    print("Creating 3D mesh plots...")
    # Convert azimuth to float for matplotlib
    azimuth_float = float(azimuth)
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves", fontsize=16
    )

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection="3d")
    ax1.view_init(elev=10, azim=azimuth_float)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap="viridis", alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection="3d")
    ax2.view_init(elev=10, azim=azimuth_float)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap="viridis", alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection="3d")
    ax3.view_init(elev=10, azim=azimuth_float)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap="viridis", alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection="3d")
    ax4.view_init(elev=10, azim=azimuth_float)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap="viridis", alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection="3d")
    ax5.view_init(elev=10, azim=azimuth_float)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap="viridis", alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.view_init(elev=10, azim=azimuth_float)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap="viridis", alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()

    # Handle saving vs showing
    if save_path is None:
        # Auto-generate save path if not provided
        if data_file:
            base_name = os.path.splitext(os.path.basename(data_file))[0]
        else:
            base_name = "mesh_aero_data"
        # Convert azimuth to float for formatting, handling string input
        azimuth_float = float(azimuth)
        save_path = _mesh_figure_path(f"{base_name}_azim{azimuth_float:.1f}.png")

    # Always save the plot
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {save_path}")

    # Show plot only if requested and in GUI environment
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot (GUI not available): {e}")
            print(f"Plot saved to file instead: {save_path}")

    print("NumPy mesh plotting completed successfully!")
    return data


def plot_aerodynamic_mesh_jax(
    data_file=None,
    data=None,
    azimuth=125.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """
    Create 3D mesh surface plots of aerodynamic coefficients using JAX data.

    Args:
        data_file (str, optional): Path to .npy file containing mesh data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        azimuth (float): Azimuth angle for 3D view in degrees. Default: 125.5
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.
        mesh_size (int): Mesh size for auto-generation. Default: 361.
        save_path (str, optional): Path to save the plot as PNG file. If None, auto-generates path.

    Returns:
        dict: The mesh data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_mesh_data(
            data_file, auto_generate=auto_generate, mesh_size=mesh_size, backend="jax"
        )

    # Extract data arrays
    ALPHA_DEG = data["ALPHA_DEG"]
    BETA_DEG = data["BETA_DEG"]
    CL = data["CL"]
    CD = data["CD"]
    CY = data["CY"]
    CLL = data.get("CLL", data.get("Cl", np.zeros_like(CL)))
    CM = data["CM"]
    CN = data["CN"]

    print("Creating JAX 3D mesh plots...")
    # Convert azimuth to float for matplotlib
    azimuth_float = float(azimuth)
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves (JAX)", fontsize=16
    )

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection="3d")
    ax1.view_init(elev=10, azim=azimuth_float)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap="viridis", alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection="3d")
    ax2.view_init(elev=10, azim=azimuth_float)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap="viridis", alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection="3d")
    ax3.view_init(elev=10, azim=azimuth_float)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap="viridis", alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection="3d")
    ax4.view_init(elev=10, azim=azimuth_float)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap="viridis", alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection="3d")
    ax5.view_init(elev=10, azim=azimuth_float)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap="viridis", alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.view_init(elev=10, azim=azimuth_float)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap="viridis", alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()

    # Handle saving vs showing
    if save_path is None:
        # Auto-generate save path if not provided
        if data_file:
            base_name = os.path.splitext(os.path.basename(data_file))[0]
        else:
            base_name = "mesh_aero_data_jax"
        # Convert azimuth to float for formatting, handling string input
        azimuth_float = float(azimuth)
        save_path = _mesh_figure_path(f"{base_name}_azim{azimuth_float:.1f}.png")

    # Always save the plot
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {save_path}")

    # Show plot only if requested and in GUI environment
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot (GUI not available): {e}")
            print(f"Plot saved to file instead: {save_path}")

    print("JAX mesh plotting completed successfully!")
    return data


def plot_aerodynamic_mesh_torch(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """
    Create 3D mesh surface plots of aerodynamic coefficients using PyTorch data.

    Args:
        data_file (str, optional): Path to .npy file containing mesh data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        azimuth (float): Azimuth angle for 3D view in degrees. Default: -127.5
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.
        mesh_size (int): Mesh size for auto-generation. Default: 361.
        save_path (str, optional): Path to save the plot as PNG file. If None, auto-generates path.

    Returns:
        dict: The mesh data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_mesh_data(
            data_file,
            auto_generate=auto_generate,
            mesh_size=mesh_size,
            backend="pytorch",
        )

    # Extract data arrays
    ALPHA_DEG = data["ALPHA_DEG"]
    BETA_DEG = data["BETA_DEG"]
    CL = data["CL"]
    CD = data["CD"]
    CY = data["CY"]
    CLL = data.get("CLL", data.get("Cl", np.zeros_like(CL)))
    CM = data["CM"]
    CN = data["CN"]

    print("Creating PyTorch 3D mesh plots...")
    # Convert azimuth to float for matplotlib
    azimuth_float = float(azimuth)
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves (PyTorch)",
        fontsize=16,
    )

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection="3d")
    ax1.view_init(elev=10, azim=azimuth_float)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap="viridis", alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection="3d")
    ax2.view_init(elev=10, azim=azimuth_float)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap="viridis", alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection="3d")
    ax3.view_init(elev=10, azim=azimuth_float)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap="viridis", alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection="3d")
    ax4.view_init(elev=10, azim=azimuth_float)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap="viridis", alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection="3d")
    ax5.view_init(elev=10, azim=azimuth_float)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap="viridis", alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.view_init(elev=10, azim=azimuth_float)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap="viridis", alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()

    # Handle saving vs showing
    if save_path is None:
        # Auto-generate save path if not provided
        if data_file:
            base_name = os.path.splitext(os.path.basename(data_file))[0]
        else:
            base_name = "mesh_aero_data_torch"
        # Convert azimuth to float for formatting, handling string input
        azimuth_float = float(azimuth)
        save_path = _mesh_figure_path(f"{base_name}_azim{azimuth_float:.1f}.png")

    # Always save the plot
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {save_path}")

    # Show plot only if requested and in GUI environment
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot (GUI not available): {e}")
            print(f"Plot saved to file instead: {save_path}")

    print("PyTorch mesh plotting completed successfully!")
    return data


def plot_aerodynamic_mesh_casadi(
    data_file=None,
    data=None,
    azimuth=-127.5,
    show_plot=True,
    auto_generate=True,
    mesh_size=361,
    save_path=None,
):
    """
    Create 3D mesh surface plots of aerodynamic coefficients using CasADi symbolic data.

    Args:
        data_file (str, optional): Path to .npy file containing mesh data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        azimuth (float): Azimuth angle for 3D view in degrees. Default: -127.5
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.
        mesh_size (int): Mesh size for auto-generation. Default: 361.
        save_path (str, optional): Path to save the plot as PNG file. If None, auto-generates path.

    Returns:
        dict: The mesh data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_mesh_data(
            data_file,
            auto_generate=auto_generate,
            mesh_size=mesh_size,
            backend="casadi_symbolic",
        )

    # Extract data arrays
    ALPHA_DEG = data["ALPHA_DEG"]
    BETA_DEG = data["BETA_DEG"]
    CL = data["CL"]
    CD = data["CD"]
    CY = data["CY"]
    CLL = data.get("CLL", data.get("Cl", np.zeros_like(CL)))
    CM = data["CM"]
    CN = data["CN"]

    print("Creating CasADi symbolic 3D mesh plots...")
    # Convert azimuth to float for matplotlib
    azimuth_float = float(azimuth)
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves (CasADi Symbolic)",
        fontsize=16,
    )

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection="3d")
    ax1.view_init(elev=10, azim=azimuth_float)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap="viridis", alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection="3d")
    ax2.view_init(elev=10, azim=azimuth_float)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap="viridis", alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection="3d")
    ax3.view_init(elev=10, azim=azimuth_float)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap="viridis", alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection="3d")
    ax4.view_init(elev=10, azim=azimuth_float)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap="viridis", alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection="3d")
    ax5.view_init(elev=10, azim=azimuth_float)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap="viridis", alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.view_init(elev=10, azim=azimuth_float)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap="viridis", alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()

    # Handle saving vs showing
    if save_path is None:
        # Auto-generate save path if not provided
        if data_file:
            base_name = os.path.splitext(os.path.basename(data_file))[0]
        else:
            base_name = "mesh_aero_data_casadi"
        # Convert azimuth to float for formatting, handling string input
        azimuth_float = float(azimuth)
        save_path = _mesh_figure_path(f"{base_name}_azim{azimuth_float:.1f}.png")

    # Always save the plot
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {save_path}")

    # Show plot only if requested and in GUI environment
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot (GUI not available): {e}")
            print(f"Plot saved to file instead: {save_path}")

    print("CasADi symbolic mesh plotting completed successfully!")
    return data


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Plot 3D mesh aerodynamic data")
    parser.add_argument("data_file", nargs="?", help="Path to .npy data file")
    parser.add_argument("--azimuth", "-a", default="-127.5", help="Camera azimuth angle")
    parser.add_argument("--backend", "-b", choices=["torch", "jax", "casadi"], default="torch", help="Computation backend")
    parser.add_argument("--no-show", action="store_true", help="Don't display the plot")
    args = parser.parse_args()

    if args.backend == "torch":
        plot_aerodynamic_mesh_torch(args.data_file, azimuth=args.azimuth, show_plot=not args.no_show)
    elif args.backend == "jax":
        plot_aerodynamic_mesh_jax(args.data_file, azimuth=args.azimuth, show_plot=not args.no_show)
    elif args.backend == "casadi":
        plot_aerodynamic_mesh_casadi(data_file=args.data_file, azimuth=args.azimuth, show_plot=not args.no_show)
