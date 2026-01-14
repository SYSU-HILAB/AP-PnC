#!/usr/bin/env python
# -*- coding: future_fstrings -*-
#
# Visualization Script for df_alpha vs real_alpha Comparison
# Created on 2025-01-14
#
# This script reads the CSV output from global_planning and visualizes
# the comparison between df_alpha (differential flatness) and real_alpha
# (from force balance equation).
#

import os
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


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

    CSV columns:
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,thrust,df_alpha,real_alpha,
    aero_force_z,yb_x,yb_y,yb_z,quat_w,quat_x,quat_y,quat_z,ap_x,ap_y,ap_z,
    S_ap_x,S_ap_z,B_Omega_b_y,alpha_dot,B_aero_force_z
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "thrust": data[:, 7],
        "df_alpha": data[:, 8],  # Differential flatness alpha (radians)
        "real_alpha": data[:, 9],  # Real alpha from force balance (radians)
        "aero_force_z": data[:, 10],
        "yb": data[:, 11:14],
        "quat": data[:, 14:18],
        "ap": data[:, 18:21],
        "S_ap_x": data[:, 21],
        "S_ap_z": data[:, 22],
        "B_Omega_b_y": data[:, 23],
        "alpha_dot": data[:, 24],
        "B_aero_force_z": data[:, 25],
    }


def plot_alpha_comparison(
    data: dict, output_path: str = None, show: bool = True
) -> None:
    """
    Plot comparison between df_alpha and real_alpha.

    Args:
        data: Dictionary containing trajectory data
        output_path: Path to save the plot (optional)
        show: Whether to display the plot interactively
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Differential Flatness Alpha vs Real Alpha (Force Balance)", fontsize=14)

    t = data["time"]
    df_alpha = data["df_alpha"]
    real_alpha = data["real_alpha"]

    # Convert to degrees for better readability
    df_alpha_deg = np.degrees(df_alpha)
    real_alpha_deg = np.degrees(real_alpha)

    # Plot 1: Time series comparison
    ax = axes[0, 0]
    ax.plot(t, df_alpha_deg, label="df_alpha (differential flatness)", linewidth=1.5)
    ax.plot(t, real_alpha_deg, label="real_alpha (force balance)", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Alpha (degrees)")
    ax.set_title("Alpha Over Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 2: Difference
    ax = axes[0, 1]
    alpha_diff = real_alpha_deg - df_alpha_deg
    ax.plot(t, alpha_diff, color="purple", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Difference (degrees)")
    ax.set_title("Real Alpha - DF Alpha")
    ax.grid(True, alpha=0.3)

    # Plot 3: Scatter plot (correlation)
    ax = axes[1, 0]
    ax.scatter(df_alpha_deg, real_alpha_deg, alpha=0.5, s=10)
    ax.plot(
        [df_alpha_deg.min(), df_alpha_deg.max()],
        [df_alpha_deg.min(), df_alpha_deg.max()],
        "r--",
        label="y=x",
        linewidth=1,
    )
    ax.set_xlabel("df_alpha (degrees)")
    ax.set_ylabel("real_alpha (degrees)")
    ax.set_title("Correlation Plot")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 4: Normalized comparison (z-score)
    ax = axes[1, 1]
    df_alpha_norm = (df_alpha_deg - df_alpha_deg.mean()) / df_alpha_deg.std()
    real_alpha_norm = (real_alpha_deg - real_alpha_deg.mean()) / real_alpha_deg.std()
    ax.plot(t, df_alpha_norm, label="df_alpha (normalized)", linewidth=1.5)
    ax.plot(t, real_alpha_norm, label="real_alpha (normalized)", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Value (z-score)")
    ax.set_title("Normalized Comparison (Trend Similarity)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✅ Plot saved to {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def print_statistics(data: dict) -> None:
    """Print statistics about the alpha comparison."""
    df_alpha = data["df_alpha"]
    real_alpha = data["real_alpha"]

    df_alpha_deg = np.degrees(df_alpha)
    real_alpha_deg = np.degrees(real_alpha)

    diff = real_alpha_deg - df_alpha_deg

    print("\n" + "=" * 60)
    print("Alpha Comparison Statistics")
    print("=" * 60)
    print(f"Time range: {data['time'].min():.2f} - {data['time'].max():.2f} s")
    print(f"Number of samples: {len(data['time'])}")
    print()
    print(f"df_alpha:   {df_alpha_deg.min():.2f}° to {df_alpha_deg.max():.2f}° (mean: {df_alpha_deg.mean():.2f}°)")
    print(f"real_alpha: {real_alpha_deg.min():.2f}° to {real_alpha_deg.max():.2f}° (mean: {real_alpha_deg.mean():.2f}°)")
    print()
    print(f"Difference (real - df):")
    print(f"  Mean:   {diff.mean():.2f}°")
    print(f"  Std:    {diff.std():.2f}°")
    print(f"  Min:    {diff.min():.2f}°")
    print(f"  Max:    {diff.max():.2f}°")
    print()

    # Calculate correlation
    corr = np.corrcoef(df_alpha_deg, real_alpha_deg)[0, 1]
    print(f"Correlation coefficient: {corr:.4f}")
    print()

    # Check if trends are similar (high correlation)
    if corr > 0.8:
        print("✅ Trends are SIMILAR (correlation > 0.8)")
    elif corr > 0.5:
        print("⚠️  Trends are MODERATELY similar (correlation > 0.5)")
    else:
        print("❌ Trends are DIFFERENT (correlation < 0.5)")

    # Check if values are close
    mean_abs_diff = np.abs(diff).mean()
    if mean_abs_diff < 5:
        print("✅ Values are CLOSE (mean absolute difference < 5°)")
    else:
        print("⚠️  Values are NOT close (mean absolute difference > 5°)")
        print("   This is EXPECTED - df_alpha and real_alpha serve different purposes")
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

    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
        output_path = sys.argv[2] if len(sys.argv) > 2 else None
        show_plot = output_path is None
    else:
        # Find latest planning CSV file
        print("No CSV file specified, searching for latest planning*.csv...")
        csv_path = find_latest_planning_csv(temp_dir)
        if csv_path is None:
            print(f"❌ No planning*.csv files found in {temp_dir}")
            print()
            print("Usage: user_test_viz_df_alpha_real_alpha.py [csv_file] [output_path]")
            print()
            print("Arguments:")
            print("  csv_file    - Path to trajectory CSV file from global_planning")
            print("                 (default: latest planning*.csv in temp_data)")
            print("  output_path - Optional path to save the plot")
            print("                 (default: show interactively)")
            sys.exit(1)
        print(f"✅ Found: {csv_path}")

    # Check if file exists
    if not Path(csv_path).exists():
        print(f"❌ File not found: {csv_path}")
        sys.exit(1)

    print(f"Loading trajectory data from {csv_path}...")
    data = load_trajectory_csv(csv_path)
    print(f"✅ Loaded {len(data['time'])} data points")

    # Print statistics
    print_statistics(data)

    # Create visualization
    print("\nGenerating visualization...")
    plot_alpha_comparison(data, output_path=output_path, show=show_plot)

    if output_path:
        print(f"\n✅ Done! Plot saved to {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
