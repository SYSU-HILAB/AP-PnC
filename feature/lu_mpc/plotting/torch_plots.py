"""
PyTorch-based plotting functions for aerodynamic coefficients.

These functions load pre-computed PyTorch data and create visualizations.
The data generation is separated from the plotting to enable clean pipeline workflows.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


def load_torch_data(data_file, auto_generate=False):
    """
    Load PyTorch aerodynamic data from .npy file. Auto-generates if file doesn't exist.

    Args:
        data_file (str): Path to .npy file containing PyTorch data
        auto_generate (bool): If True, automatically generate data if file doesn't exist

    Returns:
        dict: Dictionary containing the PyTorch data
    """
    if not os.path.exists(data_file):
        if auto_generate:
            print(f"Data file not found: {data_file}")
            print("Auto-generating torch_gd data...")

            from ..torch_gd import generate_torch_gd_data
            return generate_torch_gd_data(
                filename=os.path.splitext(os.path.basename(data_file))[0]
            )
        else:
            raise FileNotFoundError(f"Data file not found: {data_file}")

    return np.load(data_file, allow_pickle=True).item()


def plot_torch_gd(data_file=None, data=None, azimuth=-127.5, show_plot=True, auto_generate=True, save_path=None):
    """
    Plot aerodynamic coefficients with customizable azimuth angle using PyTorch data.

    Args:
        data_file (str, optional): Path to .npy file containing PyTorch data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        azimuth (float): Azimuth angle for 3D view in degrees. Default: -127.5
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.
        save_path (str, optional): Path to save the plot as PNG file. If None, auto-generates path.

    Returns:
        dict: The PyTorch data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_torch_data(data_file, auto_generate=auto_generate)

    # Extract data arrays
    ALPHA_DEG = data['ALPHA_DEG']
    BETA_DEG = data['BETA_DEG']
    CL = data['CL']
    CD = data['CD']
    CY = data['CY']
    CLL = data.get('CLL', data.get('Cl', np.zeros_like(CL)))
    CM = data['CM']
    CN = data['CN']

    print("Creating PyTorch aerodynamic coefficient plots...")
    # Convert azimuth to float for matplotlib
    azimuth_float = float(azimuth)
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle("Aerodynamic Force and Moment Coefficient Wind Tunnel Curves (PyTorch GD)", fontsize=16)

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection='3d')
    ax1.view_init(elev=10, azim=azimuth_float)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap='viridis', alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection='3d')
    ax2.view_init(elev=10, azim=azimuth_float)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap='viridis', alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection='3d')
    ax3.view_init(elev=10, azim=azimuth_float)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap='viridis', alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection='3d')
    ax4.view_init(elev=10, azim=azimuth_float)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap='viridis', alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection='3d')
    ax5.view_init(elev=10, azim=azimuth_float)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap='viridis', alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection='3d')
    ax6.view_init(elev=10, azim=azimuth_float)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap='viridis', alpha=0.8)
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
            base_name = "torch_gd_data"
        # Convert azimuth to float for formatting, handling string input
        azimuth_float = float(azimuth)
        save_path = f"{base_name}_azim{azimuth_float:.1f}.png"

    # Always save the plot
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {save_path}")

    # Show plot only if requested and in GUI environment
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot (GUI not available): {e}")
            print(f"Plot saved to file instead: {save_path}")

    print("PyTorch GD plotting completed successfully!")
    return data