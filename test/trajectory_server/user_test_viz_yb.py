#!/usr/bin/env python
#
# Visualization Script for yb_[x,y,z] (Body Y-axis) Over Time
# Created on 2025-01-15
#
# This script reads the CSV output from global_planning and visualizes
# the body Y-axis direction vector components (yb_x, yb_y, yb_z) over time.
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
        "df_phi_alpha": data[:, 8],
        "df_lyu_alpha": data[:, 9],
        "yb": data[:, 10:13],  # Body Y-axis vector [x, y, z]
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


def plot_yb_over_time(
    data: dict, output_path: str = None, show: bool = True
) -> None:
    """
    Plot yb_[x,y,z] components over time.

    Args:
        data: Dictionary containing trajectory data
        output_path: Path to save the plot (optional)
        show: Whether to display the plot interactively
    """
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Body Y-axis Direction Vector (yb_[x,y,z]) Over Time", fontsize=14)

    # Create subplots with 3D projection for bottom-left
    ax00 = fig.add_subplot(2, 2, 1)
    ax01 = fig.add_subplot(2, 2, 2)
    ax10 = fig.add_subplot(2, 2, 3, projection="3d")
    ax11 = fig.add_subplot(2, 2, 4)

    axes = [[ax00, ax01], [ax10, ax11]]

    t = data["time"]
    yb = data["yb"]
    yb_x = yb[:, 0]
    yb_y = yb[:, 1]
    yb_z = yb[:, 2]

    # Plot 1: Time series of all components
    ax00.plot(t, yb_x, label="yb_x", linewidth=1.5)
    ax00.plot(t, yb_y, label="yb_y", linewidth=1.5)
    ax00.plot(t, yb_z, label="yb_z", linewidth=1.5)
    ax00.set_xlabel("Time (s)")
    ax00.set_ylabel("Component Value")
    ax00.set_title("Body Y-axis Components Over Time")
    ax00.legend()
    ax00.grid(True, alpha=0.3)

    # Plot 2: Magnitude of yb vector (should be ~1 for unit vector)
    yb_magnitude = np.sqrt(yb_x**2 + yb_y**2 + yb_z**2)
    ax01.plot(t, yb_magnitude, color="purple", linewidth=1.5)
    ax01.axhline(y=1.0, color="r", linestyle="--", alpha=0.7, label="Unit vector")
    ax01.set_xlabel("Time (s)")
    ax01.set_ylabel("|yb| (magnitude)")
    ax01.set_title("Body Y-axis Vector Magnitude")
    ax01.legend()
    ax01.grid(True, alpha=0.3)

    # Plot 3: 3D trajectory colored by yb direction
    pos = data["pos"]
    sc = ax10.scatter(
        pos[:, 0], pos[:, 1], pos[:, 2], c=t, cmap="viridis", s=10, alpha=0.6
    )
    ax10.set_xlabel("X (m)")
    ax10.set_ylabel("Y (m)")
    ax10.set_zlabel("Z (m)")
    ax10.set_title("3D Trajectory (colored by time)")
    plt.colorbar(sc, ax=ax10, label="Time (s)")

    # Plot 4: yb direction angles (azimuth, elevation)
    yb_azimuth = np.arctan2(yb_y, yb_x)
    yb_elevation = np.arcsin(np.clip(yb_z, -1, 1))
    ax11.plot(t, np.degrees(yb_azimuth), label="Azimuth (deg)", linewidth=1.5)
    ax11.plot(t, np.degrees(yb_elevation), label="Elevation (deg)", linewidth=1.5)
    ax11.set_xlabel("Time (s)")
    ax11.set_ylabel("Angle (degrees)")
    ax11.set_title("Body Y-axis Direction Angles")
    ax11.legend()
    ax11.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✅ Plot saved to {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def print_statistics(data: dict) -> None:
    """Print statistics about the yb vector."""
    yb = data["yb"]
    yb_x = yb[:, 0]
    yb_y = yb[:, 1]
    yb_z = yb[:, 2]

    yb_magnitude = np.sqrt(yb_x**2 + yb_y**2 + yb_z**2)

    print("\n" + "=" * 60)
    print("Body Y-axis (yb) Statistics")
    print("=" * 60)
    print(f"Time range: {data['time'].min():.2f} - {data['time'].max():.2f} s")
    print(f"Number of samples: {len(data['time'])}")
    print()
    print("yb_x component:")
    print(f"  Range: [{yb_x.min():.4f}, {yb_x.max():.4f}]")
    print(f"  Mean:  {yb_x.mean():.4f}")
    print(f"  Std:   {yb_x.std():.4f}")
    print()
    print("yb_y component:")
    print(f"  Range: [{yb_y.min():.4f}, {yb_y.max():.4f}]")
    print(f"  Mean:  {yb_y.mean():.4f}")
    print(f"  Std:   {yb_y.std():.4f}")
    print()
    print("yb_z component:")
    print(f"  Range: [{yb_z.min():.4f}, {yb_z.max():.4f}]")
    print(f"  Mean:  {yb_z.mean():.4f}")
    print(f"  Std:   {yb_z.std():.4f}")
    print()
    print("yb magnitude (should be ~1.0 for unit vector):")
    print(f"  Range: [{yb_magnitude.min():.6f}, {yb_magnitude.max():.6f}]")
    print(f"  Mean:  {yb_magnitude.mean():.6f}")
    print(f"  Std:   {yb_magnitude.std():.6f}")

    # Check if yb is approximately a unit vector
    mean_magnitude = yb_magnitude.mean()
    if 0.99 <= mean_magnitude <= 1.01:
        color_print("  ✅ yb is a unit vector (magnitude ≈ 1.0)", Colors.GREEN)
    else:
        color_print(
            f"  ⚠️  yb magnitude deviation from 1.0: {abs(mean_magnitude - 1.0):.6f}",
            Colors.YELLOW,
        )
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
            print("Usage: user_test_viz_yb.py [csv_file] [output_path]")
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
    plot_yb_over_time(data, output_path=output_path, show=show_plot)

    if output_path:
        print(f"\n✅ Done! Plot saved to {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
