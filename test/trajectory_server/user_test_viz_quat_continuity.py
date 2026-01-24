#!/usr/bin/env python
#
# Visualization Script for Quaternion Continuity Test
# Created on 2025-01-15
#
# This script reads the CSV output from global_planning and visualizes
# the quaternion continuity by converting quaternions to Euler angles.
# This helps verify that df_lyu_quat discontinuities are fixed.
#
# Copyright (c) 2025, Sun Yat-sen University
# Authors: Hanamy: rongerch@outlook.com

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

    # Sort by modification time, get the latest
    files_with_time = [(f, os.path.getmtime(dir_path / f)) for f in files]
    latest_file = max(files_with_time, key=lambda x: x[1])[0]
    return str(dir_path / latest_file)


def load_trajectory_csv(csv_path: str) -> dict:
    """
    Load trajectory CSV file from global_planning.

    CSV columns (30 total):
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,B_ap_x_df_lyu,df_phi_alpha,df_lyu_alpha,
    yb_x,yb_y,yb_z,df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,
    df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,Wap_x,Wap_y,Wap_z,
    S_ap_x,S_ap_z,S_Omega_x,S_Omega_y,S_Omega_z,alpha_dot
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "B_ap_x_df_lyu": data[:, 7],
        "df_phi_alpha": np.degrees(data[:, 8]),
        "df_lyu_alpha": np.degrees(data[:, 9]),
        "yb": data[:, 10:13],
        "df_phi_quat": data[:, 13:17],
        "df_lyu_quat": data[:, 17:21],
        "Wap": data[:, 21:24],
        "S_ap_x": data[:, 24],
        "S_ap_z": data[:, 25],
        "S_Omega_x": data[:, 26],
        "S_Omega_y": data[:, 27],
        "S_Omega_z": data[:, 28],
        "alpha_dot": data[:, 29],
    }


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Uses Z-X-Y intrinsic Euler convention (same as in math_utils.py)

    Args:
        q: (N, 4) array of quaternions [w, x, y, z]

    Returns:
        (N, 3) array of Euler angles in degrees [roll, pitch, yaw]
    """
    w = q[:, 0]
    x = q[:, 1]
    y = q[:, 2]
    z = q[:, 3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.where(
        np.abs(sinp) >= 1, np.copysign(np.pi / 2, sinp), np.arcsin(sinp)
    )

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


def detect_discontinuities(data: np.ndarray, threshold: float = 90.0) -> np.ndarray:
    """Detect discontinuities in a time series."""
    diff = np.abs(np.diff(data, axis=0))
    # Check each dimension separately
    max_jumps = np.max(diff, axis=1)
    return np.where(max_jumps > threshold)[0]


def plot_quat_continuity(data: dict, output_path: str = None, show: bool = True):
    """
    Plot quaternion Euler angles to check continuity.

    Args:
        data: Dictionary containing trajectory data
        output_path: Path to save the plot (optional)
        show: Whether to display the plot interactively
    """
    t = data["time"]

    # Convert to Euler angles
    phi_euler = quat_to_euler(data["df_phi_quat"])
    lyu_euler = quat_to_euler(data["df_lyu_quat"])

    # Detect discontinuities
    phi_jumps = detect_discontinuities(phi_euler, threshold=90.0)
    lyu_jumps = detect_discontinuities(lyu_euler, threshold=90.0)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Quaternion Continuity Test (Euler Angles)", fontsize=14)

    angles = ["Roll (φ)", "Pitch (θ)", "Yaw (ψ)"]
    for i, ax in enumerate(axes):
        ax.plot(t, phi_euler[:, i], label="df_phi_quat", alpha=0.7, linewidth=1.5)
        ax.plot(t, lyu_euler[:, i], label="df_lyu_quat", alpha=0.7, linewidth=1.5)

        # Highlight discontinuities
        if len(phi_jumps) > 0:
            ax.scatter(
                t[phi_jumps],
                phi_euler[phi_jumps, i],
                c="orange",
                s=30,
                zorder=5,
                marker="x",
                label="df_phi_quat jumps",
            )
        if len(lyu_jumps) > 0:
            ax.scatter(
                t[lyu_jumps],
                lyu_euler[lyu_jumps, i],
                c="red",
                s=30,
                zorder=5,
                marker="o",
                label="df_lyu_quat jumps",
            )

        ax.set_ylabel(f"{angles[i]} (deg)")
        ax.set_title(f"{angles[i]} Comparison")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved to {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def print_statistics(data: dict) -> None:
    """Print statistics about quaternion continuity."""
    t = data["time"]

    # Convert to Euler angles
    phi_euler = quat_to_euler(data["df_phi_quat"])
    lyu_euler = quat_to_euler(data["df_lyu_quat"])

    # Detect discontinuities
    phi_jumps = detect_discontinuities(phi_euler, threshold=90.0)
    lyu_jumps = detect_discontinuities(lyu_euler, threshold=90.0)

    print("\n" + "=" * 60)
    print("Quaternion Continuity Test Statistics")
    print("=" * 60)
    print(f"Time range: {t.min():.2f} - {t.max():.2f} s")
    print(f"Number of samples: {len(t)}")
    print()

    # Print df_phi_quat statistics
    print("df_phi_quat (Differential Flatness):")
    angles = ["Roll", "Pitch", "Yaw"]
    for i, name in enumerate(angles):
        print(f"  {name:>6}: {phi_euler[:, i].min():7.2f}° to {phi_euler[:, i].max():7.2f}°")
    print(f"  Discontinuities > 90°: {len(phi_jumps)}")

    # Print df_lyu_quat statistics
    print("\ndf_lyu_quat (Force Balance):")
    for i, name in enumerate(angles):
        print(f"  {name:>6}: {lyu_euler[:, i].min():7.2f}° to {lyu_euler[:, i].max():7.2f}°")
    print(f"  Discontinuities > 90°: {len(lyu_jumps)}")

    # Overall assessment
    print()
    if len(lyu_jumps) == 0:
        color_print("PASS: df_lyu_quat shows no discontinuities", Colors.GREEN)
    else:
        color_print(
            f"WARN: df_lyu_quat has {len(lyu_jumps)} discontinuities", Colors.YELLOW
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
            print(f"No planning*.csv files found in {temp_dir}")
            print()
            print("Usage: user_test_viz_quat_continuity.py [csv_file] [output_path]")
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

    # Create visualization
    print("\nGenerating visualization...")
    plot_quat_continuity(data, output_path=output_path, show=show_plot)

    if output_path:
        print(f"\nDone! Plot saved to {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
