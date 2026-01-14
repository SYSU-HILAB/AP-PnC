#!/usr/bin/env python
# -*- coding: future_fstrings -*-
#
# Visualization Script for df_phi_alpha/df_lyu_alpha Time Series
# Created on 2025-01-15
#
# This script reads the CSV output from global_planning and visualizes
# the differential-flatness and lookup alpha commands over time so that
# operators can quickly check whether the profiles align with expectations.
#

import os
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from feature.utils.color_print import Colors, color_print


def find_latest_planning_csv(dir_path: Path) -> str | None:
    """Find the latest planning*.csv file in the directory."""
    pattern = re.compile(r"planning.*\.csv")
    files = [f for f in os.listdir(dir_path) if pattern.match(f)]

    if not files:
        return None

    files_with_time = [(f, os.path.getmtime(dir_path / f)) for f in files]
    latest_file = max(files_with_time, key=lambda x: x[1])[0]
    return str(dir_path / latest_file)


def load_alpha_columns(csv_path: str) -> dict:
    """
    Load only the columns required for df_phi_alpha/df_lyu_alpha visualization.

    CSV schema (subset):
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,thrust,df_phi_alpha,df_lyu_alpha,...
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "df_phi_alpha": data[:, 8],
        "df_lyu_alpha": data[:, 9],
    }


def plot_alpha_time_series(
    data: dict, output_path: str = None, show: bool = True
) -> None:
    """Plot df_phi_alpha and df_lyu_alpha over time (degrees)."""
    t = data["time"]
    df_phi_alpha = np.degrees(data["df_phi_alpha"])
    df_lyu_alpha = np.degrees(data["df_lyu_alpha"])
    delta = df_phi_alpha - df_lyu_alpha

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("df_phi_alpha / df_lyu_alpha Time Series", fontsize=14)

    axes[0].plot(t, df_phi_alpha, color="firebrick", linewidth=1.8)
    axes[0].set_ylabel("df_phi_alpha (deg)")
    axes[0].set_title("Differential Flatness Alpha Command")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, df_lyu_alpha, color="seagreen", linewidth=1.8)
    axes[1].set_ylabel("df_lyu_alpha (deg)")
    axes[1].set_title("Lookup/Force Balance Alpha Command")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, delta, color="slateblue", linewidth=1.5)
    axes[2].axhline(0.0, color="k", linestyle="--", linewidth=1, alpha=0.5)
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Difference (deg)")
    axes[2].set_title("df_phi_alpha - df_lyu_alpha")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✅ Plot saved to {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def print_alpha_statistics(data: dict) -> None:
    """Print descriptive statistics for the alpha signals."""
    def describe(label: str, values: np.ndarray) -> None:
        deg = np.degrees(values)
        print(f"{label}:")
        print(f"  Range: [{deg.min():.2f}°, {deg.max():.2f}°]")
        print(f"  Mean:  {deg.mean():.2f}°")
        print(f"  Std:   {deg.std():.2f}°")

    df_phi_alpha = data["df_phi_alpha"]
    df_lyu_alpha = data["df_lyu_alpha"]
    delta = df_phi_alpha - df_lyu_alpha
    corr = np.corrcoef(df_phi_alpha, df_lyu_alpha)[0, 1]

    print("\n" + "=" * 60)
    print("Alpha Command Time-Series Statistics")
    print("=" * 60)
    print(f"Time span: {data['time'].min():.2f}s -> {data['time'].max():.2f}s")
    print(f"Samples:   {len(data['time'])}")
    print()
    describe("df_phi_alpha", df_phi_alpha)
    print()
    describe("df_lyu_alpha", df_lyu_alpha)
    print()
    describe("Difference (phi - lyu)", delta)
    print()
    print("Correlation coefficient (rad): ", end="")
    if corr > 0.8:
        color_print(f"{corr:.4f}", Colors.BOLD + Colors.GREEN)
    elif corr > 0.5:
        color_print(f"{corr:.4f}", Colors.YELLOW)
    else:
        color_print(f"{corr:.4f}", Colors.RED)
    print("=" * 60)


def main() -> int:
    """CLI entry point."""
    repo_root = Path(os.getenv("AP_PNC_DIR") or Path(__file__).resolve().parents[2])
    temp_dir = repo_root / "artifacts" / "simulations" / "temp_data"

    args = [arg for arg in sys.argv[1:] if arg != "--no-show"]
    show_plot = "--no-show" not in sys.argv[1:]

    csv_path = None
    output_path = None

    if args:
        csv_path = args[0]
        if len(args) > 1:
            output_path = args[1]
            show_plot = False
    else:
        print("No CSV file specified, searching for latest planning*.csv...")
        csv_path = find_latest_planning_csv(temp_dir)
        if csv_path is None:
            print(f"❌ No planning*.csv files found in {temp_dir}")
            print()
            print("Usage: user_test_viz_alpha_time_series.py [csv_file] [output_path]")
            print()
            print("Arguments:")
            print("  csv_file    - Path to trajectory CSV (defaults to latest planning*.csv)")
            print("  output_path - Optional path to save the figure")
            return 1
        print(f"✅ Found: {csv_path}")

    if not Path(csv_path).exists():
        print(f"❌ File not found: {csv_path}")
        return 1

    print(f"Loading trajectory data from {csv_path}...")
    data = load_alpha_columns(csv_path)
    print(f"✅ Loaded {len(data['time'])} samples")

    print_alpha_statistics(data)
    print("\nGenerating visualization...")
    plot_alpha_time_series(data, output_path=output_path, show=show_plot)

    if output_path:
        print(f"\n✅ Done! Plot saved to {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
