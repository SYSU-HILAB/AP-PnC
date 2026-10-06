#!/usr/bin/env python
"""Direct CSV aerodynamics data visualization.

Reads and plots aerodynamics CSV data directly from <AP_PNC_DIR>/.artifacts/aerodynamics/

NOTE: Pybind11 verification is DISABLED due to a known bug in the pybind11
cleanup code that causes segfault during object destruction.
The CSV data itself is valid - this is purely a binding issue.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from tooling.env import project_path

# Pybind11 verification is disabled due to cleanup code segfault
HAS_PYBIND = False

# Aero type configuration (all 4 types, using CSV data source)
AERO_TYPES = ["lyu", "bspline", "phi", "advanced"]
AERO_COLORS = {
    "lyu": "#1f77b4",      # blue
    "bspline": "#ff7f0e",   # orange
    "phi": "#2ca02c",       # green
    "advanced": "#d62728",  # red
}


def plot_all_csv_aero_data(git_root: Path | None = None) -> None:
    """Plot all aerodynamics CSV files on a 2x1 subplot.

    Args:
        git_root: Repository root path. If None, auto-detected.
    """
    if git_root is None:
        git_root = Path(__file__).parent.parent.parent

    aero_data_dir = project_path(".artifacts/aerodynamics")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    for aero_type in AERO_TYPES:
        csv_path = aero_data_dir / f"{aero_type}_across_AoAs.csv"
        if not csv_path.exists():
            print(f"Warning: {csv_path} not found, skipping...")
            continue

        df = pd.read_csv(csv_path)
        color = AERO_COLORS[aero_type]

        # Plot cx vs alpha
        ax1.plot(df["alpha(rad)"], df["cx"], label=aero_type, linewidth=2, color=color)

        # Plot cz vs alpha
        ax2.plot(df["alpha(rad)"], df["cz"], label=aero_type, linewidth=2, color=color)

    # Configure cx subplot
    ax1.set_xlabel(r"$\alpha$ (rad)")
    ax1.set_ylabel("$C_x$")
    ax1.set_title("Drag Coefficient $C_x$ vs $\\alpha$")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    # Configure cz subplot
    ax2.set_xlabel(r"$\alpha$ (rad)")
    ax2.set_ylabel("$C_z$")
    ax2.set_title("Lift Coefficient $C_z$ vs $\\alpha$")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_all_csv_aero_data()
