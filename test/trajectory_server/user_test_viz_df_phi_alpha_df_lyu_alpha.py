#!/usr/bin/env python
#
# Visualization Script for df_phi_alpha vs df_lyu_alpha Comparison
# Created on 2025-01-14
#
# This script reads the CSV output from global_planning and visualizes
# the comparison between df_phi_alpha (differential flatness) and df_lyu_alpha
# (from force balance equation).
#

import os
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ap_pnc.utils.color_print import Colors, color_print


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

    CSV columns (32 total):
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,B_ap_x_df_lyu,df_phi_alpha,df_lyu_alpha,
    yb_x,yb_y,yb_z,df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,
    df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,Wap_x,Wap_y,Wap_z,
    S_ap_x,S_ap_z,S_ap_x_dot,S_ap_z_dot,S_Omega_x,S_Omega_y,S_Omega_z,alpha_dot
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "B_ap_x_df_lyu": data[:, 7],
        "df_phi_alpha": data[:, 8],  # Differential flatness alpha (radians)
        "df_lyu_alpha": data[:, 9],  # DF Lyu alpha from force balance (radians)
        "yb": data[:, 10:13],
        "df_phi_quat": data[:, 13:17],
        "df_lyu_quat": data[:, 17:21],
        "Wap": data[:, 21:24],
        "S_ap_x": data[:, 24],
        "S_ap_z": data[:, 25],
        "S_ap_x_dot": data[:, 26],
        "S_ap_z_dot": data[:, 27],
        "S_Omega_x": data[:, 28],
        "S_Omega_y": data[:, 29],
        "S_Omega_z": data[:, 30],
        "alpha_dot": data[:, 31],
    }


def plot_alpha_comparison(
    data: dict, output_path: str = None, show: bool = True
) -> None:
    """
    Plot comparison between df_phi_alpha and df_lyu_alpha.

    Args:
        data: Dictionary containing trajectory data
        output_path: Path to save the plot (optional)
        show: Whether to display the plot interactively
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("df_phi_alpha vs df_lyu_alpha (Force Balance)", fontsize=14)

    t = data["time"]
    df_phi_alpha = data["df_phi_alpha"]
    df_lyu_alpha = data["df_lyu_alpha"]

    # Convert to degrees for better readability
    df_phi_alpha_deg = np.degrees(df_phi_alpha)
    df_lyu_alpha_deg = np.degrees(df_lyu_alpha)

    # Plot 1: Time series comparison
    ax = axes[0, 0]
    ax.plot(t, df_phi_alpha_deg, label="df_phi_alpha (differential flatness)", linewidth=1.5)
    ax.plot(t, df_lyu_alpha_deg, label="df_lyu_alpha (force balance)", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Alpha (degrees)")
    ax.set_title("Alpha Over Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 2: Scatter plot (correlation)
    ax = axes[0, 1]
    ax.scatter(df_phi_alpha_deg, df_lyu_alpha_deg, alpha=0.5, s=10)
    ax.plot(
        [df_phi_alpha_deg.min(), df_phi_alpha_deg.max()],
        [df_phi_alpha_deg.min(), df_phi_alpha_deg.max()],
        "r--",
        label="y=x",
        linewidth=1,
    )
    ax.set_xlabel("df_phi_alpha (degrees)")
    ax.set_ylabel("df_lyu_alpha (degrees)")
    ax.set_title("Correlation Plot")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Calculate and display correlation coefficient
    corr = np.corrcoef(df_phi_alpha_deg, df_lyu_alpha_deg)[0, 1]
    ax.text(
        0.05,
        0.95,
        f"Correlation: {corr:.4f}",
        transform=ax.transAxes,
        fontsize=12,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    # Plot 3: Normalized comparison (z-score)
    ax = axes[1, 0]
    df_phi_alpha_norm = (df_phi_alpha_deg - df_phi_alpha_deg.mean()) / df_phi_alpha_deg.std()
    df_lyu_alpha_norm = (df_lyu_alpha_deg - df_lyu_alpha_deg.mean()) / df_lyu_alpha_deg.std()
    ax.plot(t, df_phi_alpha_norm, label="df_phi_alpha (normalized)", linewidth=1.5)
    ax.plot(t, df_lyu_alpha_norm, label="df_lyu_alpha (normalized)", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Value (z-score)")
    ax.set_title("Normalized Comparison (Trend Similarity)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 4: Remove (empty)
    axes[1, 1].axis("off")

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
    df_phi_alpha = data["df_phi_alpha"]
    df_lyu_alpha = data["df_lyu_alpha"]

    df_phi_alpha_deg = np.degrees(df_phi_alpha)
    df_lyu_alpha_deg = np.degrees(df_lyu_alpha)

    print("\n" + "=" * 60)
    print("Alpha Comparison Statistics")
    print("=" * 60)
    print(f"Time range: {data['time'].min():.2f} - {data['time'].max():.2f} s")
    print(f"Number of samples: {len(data['time'])}")
    print()
    print(f"df_phi_alpha:   {df_phi_alpha_deg.min():.2f}° to {df_phi_alpha_deg.max():.2f}° (mean: {df_phi_alpha_deg.mean():.2f}°)")
    print(f"df_lyu_alpha: {df_lyu_alpha_deg.min():.2f}° to {df_lyu_alpha_deg.max():.2f}° (mean: {df_lyu_alpha_deg.mean():.2f}°)")
    print()

    # Calculate correlation
    corr = np.corrcoef(df_phi_alpha_deg, df_lyu_alpha_deg)[0, 1]
    print("Correlation coefficient: ", end="")
    color_print(f"{corr:.4f}", Colors.BOLD + Colors.GREEN)
    print()

    # Check if trends are similar (high correlation)
    if corr > 0.8:
        color_print("✅ Trends are SIMILAR (correlation > 0.8)", Colors.GREEN)
    elif corr > 0.5:
        color_print("⚠️  Trends are MODERATELY similar (correlation > 0.5)", Colors.YELLOW)
    else:
        color_print("❌ Trends are DIFFERENT (correlation < 0.5)", Colors.RED)
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
            print(f"❌ No planning*.csv files found in {temp_dir}")
            print()
            print("Usage: user_test_viz_df_phi_alpha_df_lyu_alpha.py [csv_file] [output_path]")
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
