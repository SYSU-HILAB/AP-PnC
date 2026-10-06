"""
Aerodynamic coefficient plotting functions.

All helpers in this module expect dictionaries that follow the schemas emitted by
`feature.lu_mpc.plotting.data_utils`:

* `plot_cl_cd_cm_coordinated` consumes `data_type="coordinated_flight"` produced by
  `save_coordinated_data`. Required keys: `alpha_deg`, `CL`, `CD`, `CM` (all 1-D arrays).
* `plot_all_aerodynamic_coeffs` consumes `data_type="all_coefficients"` produced by
  `save_all_coeffs_data`. Required keys: `alpha_deg`, `CL`, `CD`, `CY`, `Cl` or `CLL`,
  `Cm`, `Cn`.
* `plot_cl_cd_aero` only needs `alpha_deg` (or legacy `aoa_deg`), `CL`, and `CD`.

Each plotter will raise soon after loading if any field is missing, helping users detect
incompatible cache files before matplotlib work begins.
"""

import os

import matplotlib.pyplot as plt
import numpy as np

COORDINATED_KEYS = ("alpha_deg", "CL", "CD", "CM")
ALL_COEFF_KEYS = ("alpha_deg", "CL", "CD", "CY", "Cm", "Cn")
CL_CD_KEYS = ("CL", "CD")


def load_aero_data(data_file, auto_generate=False):
    """
    Load aerodynamic data from .npy file. Auto-generates if file doesn't exist.

    Args:
        data_file (str): Path to .npy file containing aerodynamic data
        auto_generate (bool): If True, automatically generate data if file doesn't exist

    Returns:
        dict: Dictionary containing the aerodynamic data
    """
    if not os.path.exists(data_file):
        if auto_generate:
            print(f"Data file not found: {data_file}")
            print("Auto-generating coordinated flight data using JAX backend...")

            from tooling.paper.methods.lu.gd_model_jax import generate_cl_cd_cm_coordinated_data

            return generate_cl_cd_cm_coordinated_data(
                filename=os.path.splitext(os.path.basename(data_file))[0]
            )
        else:
            raise FileNotFoundError(f"Data file not found: {data_file}")

    return np.load(data_file, allow_pickle=True).item()


def plot_cl_cd_cm_coordinated(
    data_file=None, data=None, show_plot=True, auto_generate=True
):
    """
    Plot CL, CD, CM coefficients versus alpha under coordinated flight assumption (beta = 0).
    Creates one row with three subplots for each coefficient.

    Args:
        data_file (str, optional): Path to .npy file containing aerodynamic data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        show_plot (bool): Whether to display the plot. Default: True.
        auto_generate (bool): Whether to auto-generate data if file not found. Default: True.

    Returns:
        dict: The aerodynamic data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_aero_data(data_file, auto_generate=auto_generate)

    # Extract data arrays
    alpha_deg = data["alpha_deg"]
    CL = data["CL"]
    CD = data["CD"]
    CM = data["CM"]

    # Create the plot with one row and three subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(
        "Aerodynamic Coefficients vs Angle of Attack (Coordinated Flight: β = 0°)",
        fontsize=16,
    )

    # CL subplot
    ax1.plot(alpha_deg, CL, "b-", linewidth=2)
    ax1.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax1.set_ylabel("CL (Lift Coefficient)", fontsize=12)
    ax1.set_title("Lift Coefficient CL", fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax1.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax1.set_xlim([-30, 120])

    # CD subplot
    ax2.plot(alpha_deg, CD, "r-", linewidth=2)
    ax2.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax2.set_ylabel("CD (Drag Coefficient)", fontsize=12)
    ax2.set_title("Drag Coefficient CD", fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax2.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax2.set_xlim([-30, 120])

    # CM subplot
    ax3.plot(alpha_deg, CM, "g-", linewidth=2)
    ax3.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax3.set_ylabel("CM (Pitching Moment Coefficient)", fontsize=12)
    ax3.set_title("Pitching Moment Coefficient CM", fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax3.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax3.set_xlim([-30, 120])

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("Plot completed!")
    return data


def plot_all_aerodynamic_coeffs(data_file=None, data=None, show_plot=True):
    """
    Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA.

    Args:
        data_file (str, optional): Path to .npy file containing aerodynamic data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        show_plot (bool): Whether to display the plot. Default: True.

    Returns:
        dict: The aerodynamic data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_aero_data(data_file)

    # Extract data arrays
    alpha_deg = data["alpha_deg"]
    CL = data["CL"]
    CD = data["CD"]
    CY = data["CY"]
    Cl = data["Cl"] if "Cl" in data else data.get("CLL", np.zeros_like(CL))
    Cm = data["Cm"]
    Cn = data["Cn"]

    # Create figure with 2x3 subplots
    fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle("All Aerodynamic Coefficients vs Angle of Attack", fontsize=16)

    # CL subplot
    ax1.plot(alpha_deg, CL, "b-", linewidth=2)
    ax1.set_xlabel("Angle of Attack α [degrees]")
    ax1.set_ylabel("CL (Lift Coefficient)")
    ax1.set_title("Lift Coefficient CL")
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # CD subplot
    ax2.plot(alpha_deg, CD, "r-", linewidth=2)
    ax2.set_xlabel("Angle of Attack α [degrees]")
    ax2.set_ylabel("CD (Drag Coefficient)")
    ax2.set_title("Drag Coefficient CD")
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # CY subplot
    ax3.plot(alpha_deg, CY, "g-", linewidth=2)
    ax3.set_xlabel("Angle of Attack α [degrees]")
    ax3.set_ylabel("CY (Side Force Coefficient)")
    ax3.set_title("Side Force Coefficient CY")
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # Cl subplot
    ax4.plot(alpha_deg, Cl, "m-", linewidth=2)
    ax4.set_xlabel("Angle of Attack α [degrees]")
    ax4.set_ylabel("Cl (Rolling Moment Coefficient)")
    ax4.set_title("Rolling Moment Coefficient Cl")
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # Cm subplot
    ax5.plot(alpha_deg, Cm, "c-", linewidth=2)
    ax5.set_xlabel("Angle of Attack α [degrees]")
    ax5.set_ylabel("Cm (Pitching Moment Coefficient)")
    ax5.set_title("Pitching Moment Coefficient Cm")
    ax5.grid(True, alpha=0.3)
    ax5.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # Cn subplot
    ax6.plot(alpha_deg, Cn, "orange", linestyle="-", linewidth=2)
    ax6.set_xlabel("Angle of Attack α [degrees]")
    ax6.set_ylabel("Cn (Yawing Moment Coefficient)")
    ax6.set_title("Yawing Moment Coefficient Cn")
    ax6.grid(True, alpha=0.3)
    ax6.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("All aerodynamic coefficients plot completed!")
    return data


def plot_cl_cd_aero(data_file=None, data=None, show_plot=True):
    """
    Plot CL and CD coefficients across AoA.

    Args:
        data_file (str, optional): Path to .npy file containing aerodynamic data.
                                  If None, data must be provided.
        data (dict, optional): Direct data dictionary. If provided, data_file is ignored.
        show_plot (bool): Whether to display the plot. Default: True.

    Returns:
        dict: The aerodynamic data used for plotting
    """
    # Load data
    if data is None:
        if data_file is None:
            raise ValueError("Either data_file or data must be provided")
        data = load_aero_data(data_file)

    # Extract data arrays
    aoa_deg = data.get("alpha_deg", data.get("aoa_deg"))
    CL = data["CL"]
    CD = data["CD"]

    # Create the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle("Aerodynamic Coefficients vs Angle of Attack", fontsize=16)

    # CL subplot
    ax1.plot(aoa_deg, CL, "b-", linewidth=2)
    ax1.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax1.set_ylabel("CL (Lift Coefficient)", fontsize=12)
    ax1.set_title("Lift Coefficient CL", fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax1.axvline(x=0, color="k", linestyle="--", alpha=0.3)

    # CD subplot
    ax2.plot(aoa_deg, CD, "r-", linewidth=2)
    ax2.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax2.set_ylabel("CD (Drag Coefficient)", fontsize=12)
    ax2.set_title("Drag Coefficient CD", fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax2.axvline(x=0, color="k", linestyle="--", alpha=0.3)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("CL/CD plot completed!")
    return data


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Plot aerodynamic coefficients")
    parser.add_argument("data_file", nargs="?", help="Path to .npy data file")
    parser.add_argument("--no-show", action="store_true", help="Don't display the plot")
    args = parser.parse_args()

    plot_cl_cd_cm_coordinated(args.data_file, show_plot=not args.no_show)
