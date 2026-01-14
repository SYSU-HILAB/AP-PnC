#!/usr/bin/env python
# -*- coding: future_fstrings -*-
#
# Integration test for angle unwrapping in AeroAlphaSolver.
# Loads CSV data and verifies df_lyu_alpha continuity.
#
# Copyright (c) 2025, Sun Yat-sen University
# Authors: Hanamy: rongerch@outlook.com

import os
import re
import sys
from pathlib import Path

import numpy as np

from feature.utils.color_print import Colors, color_print


def find_latest_planning_csv(dir_path: Path) -> str | None:
    """Find the latest planning*.csv file in the directory."""
    pattern = re.compile(r"planning.*\.csv")
    files = [f for f in os.listdir(dir_path) if pattern.match(f)]

    if not files:
        return None

    # Sort by modification time, get the latest
    files_with_time = [(f, os.path.getmtime(dir_path / f)) for f in files]
    latest_file = max(files_with_time, key=lambda x: x[1])[0]
    return str(dir_path / latest_file)


def load_trajectory_csv(csv_path: str) -> dict:
    """
    Load trajectory CSV file from global_planning.

    CSV columns (based on current format):
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,thrust,df_phi_alpha,df_lyu_alpha,
    aero_force_z,yb_x,yb_y,yb_z,df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,
    df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,ap_x,ap_y,ap_z,
    S_ap_x,S_ap_z,B_Omega_b_y,alpha_dot,B_aero_force_z
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "thrust": data[:, 7],
        "df_phi_alpha": data[:, 8],   # Differential flatness alpha (radians)
        "df_lyu_alpha": data[:, 9],   # DF Lyu alpha from force balance (radians)
        "aero_force_z": data[:, 10],
        "yb": data[:, 11:14],
        "df_phi_quat": data[:, 14:18],
        "df_lyu_quat": data[:, 18:22],
        "ap": data[:, 22:25],
        "S_ap_x": data[:, 25],
        "S_ap_z": data[:, 26],
        "B_Omega_b_y": data[:, 27],
        "alpha_dot": data[:, 28],
        "B_aero_force_z": data[:, 29],
    }


def test_alpha_continuity(csv_path: str, max_jump_deg: float = 180.0) -> bool:
    """
    Test that df_lyu_alpha has no discontinuities > max_jump_deg.

    Args:
        csv_path: Path to planning CSV file
        max_jump_deg: Maximum allowed jump between consecutive samples (degrees)

    Returns:
        True if test passes (no large jumps), False otherwise
    """
    # Load data
    data = load_trajectory_csv(csv_path)
    df_lyu_alpha = np.degrees(data["df_lyu_alpha"])

    # Compute consecutive differences
    alpha_diff = np.abs(np.diff(df_lyu_alpha))

    # Find discontinuities
    large_jumps = np.where(alpha_diff > max_jump_deg)[0]

    if len(large_jumps) > 0:
        print(f"FAIL: Found {len(large_jumps)} discontinuities > {max_jump_deg}°")
        print(f"   Jump locations (time indices): {large_jumps[:10]}")
        print(f"   Jump magnitudes: {alpha_diff[large_jumps[:10]]}")
        return False
    else:
        color_print(f"PASS: No discontinuities > {max_jump_deg}° detected", Colors.GREEN)
        print(f"   Max jump: {alpha_diff.max():.2f}°")
        return True


def plot_alpha_with_jumps(csv_path: str, output_path: str = None):
    """Plot df_lyu_alpha highlighting discontinuities."""
    data = load_trajectory_csv(csv_path)
    t = data["time"]
    df_lyu_alpha = np.degrees(data["df_lyu_alpha"])
    df_phi_alpha = np.degrees(data["df_phi_alpha"])

    # Find jumps
    alpha_diff = np.abs(np.diff(df_lyu_alpha))
    jump_indices = np.where(alpha_diff > 90.0)[0]  # Jumps > 90 degrees

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Alpha Continuity Test", fontsize=14)

    # Plot df_lyu_alpha
    ax = axes[0]
    ax.plot(t, df_lyu_alpha, "b-", linewidth=1.5, label="df_lyu_alpha")
    if len(jump_indices) > 0:
        ax.scatter(
            t[jump_indices],
            df_lyu_alpha[jump_indices],
            c="red",
            s=50,
            zorder=5,
            label="Discontinuities",
        )
    ax.set_ylabel("Alpha (degrees)")
    ax.set_title("df_lyu_alpha (Force Balance)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot df_phi_alpha for comparison
    ax = axes[1]
    ax.plot(t, df_phi_alpha, "g-", linewidth=1.5, label="df_phi_alpha")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Alpha (degrees)")
    ax.set_title("df_phi_alpha (Differential Flatness)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Plot saved to {output_path}")
    else:
        plt.show()


def print_statistics(data: dict) -> None:
    """Print statistics about the alpha values."""
    df_phi_alpha = np.degrees(data["df_phi_alpha"])
    df_lyu_alpha = np.degrees(data["df_lyu_alpha"])

    print("\n" + "=" * 60)
    print("Alpha Unwrapping Test Statistics")
    print("=" * 60)
    print(f"Time range: {data['time'].min():.2f} - {data['time'].max():.2f} s")
    print(f"Number of samples: {len(data['time'])}")
    print()
    print(f"df_phi_alpha:   {df_phi_alpha.min():.2f}° to {df_phi_alpha.max():.2f}° (mean: {df_phi_alpha.mean():.2f}°)")
    print(f"df_lyu_alpha:   {df_lyu_alpha.min():.2f}° to {df_lyu_alpha.max():.2f}° (mean: {df_lyu_alpha.mean():.2f}°)")
    print()

    # Calculate correlation
    corr = np.corrcoef(df_phi_alpha, df_lyu_alpha)[0, 1]
    print("Correlation coefficient: ", end="")
    color_print(f"{corr:.4f}", Colors.BOLD + Colors.GREEN)
    print()

    # Check maximum consecutive jump
    max_jump = np.abs(np.diff(df_lyu_alpha)).max()
    print(f"Max consecutive jump in df_lyu_alpha: ", end="")
    if max_jump < 180.0:
        color_print(f"{max_jump:.2f}°", Colors.GREEN)
    else:
        color_print(f"{max_jump:.2f}° (FAIL!)", Colors.RED)
    print("=" * 60)


def main():
    """Main entry point."""
    # Get repo root and temp data directory
    repo_root = Path(os.getenv("AP_PNC_DIR") or Path(__file__).resolve().parents[2])
    temp_dir = repo_root / "artifacts" / "simulations" / "temp_data"

    # Parse arguments
    csv_path = None
    output_path = None
    show_plot = True

    # Handle --no-show flag
    args = [arg for arg in sys.argv[1:] if arg != "--no-show"]
    show_plot = "--no-show" not in sys.argv[1:]

    if len(args) >= 1:
        csv_path = args[0]
        output_path = args[1] if len(args) > 1 else None
        if output_path:
            show_plot = False
    else:
        # Find latest planning CSV file
        print("No CSV file specified, searching for latest planning*.csv...")
        csv_path = find_latest_planning_csv(temp_dir)
        if csv_path is None:
            print(f"No planning*.csv files found in {temp_dir}")
            print()
            print("Usage: test_alpha_unwrapping.py [csv_file] [output_path]")
            print()
            print("Arguments:")
            print("  csv_file    - Path to trajectory CSV file from global_planning")
            print("                 (default: latest planning*.csv in temp_data)")
            print("  output_path - Optional path to save the plot")
            print("                 (default: show interactively)")
            sys.exit(1)
        print(f"Found: {csv_path}")

    # Check if file exists
    if not Path(csv_path).exists():
        print(f"File not found: {csv_path}")
        sys.exit(1)

    print(f"Loading trajectory data from {csv_path}...")
    data = load_trajectory_csv(csv_path)
    print(f"Loaded {len(data['time'])} data points")

    # Print statistics
    print_statistics(data)

    # Run test
    print("\nRunning continuity test...")
    passed = test_alpha_continuity(str(csv_path), max_jump_deg=180.0)

    # Generate plot
    print("\nGenerating visualization...")
    if output_path:
        plot_alpha_with_jumps(str(csv_path), output_path)
    elif show_plot:
        plot_alpha_with_jumps(str(csv_path))

    if output_path:
        print(f"\nDone! Plot saved to {output_path}")

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    sys.exit(main())
