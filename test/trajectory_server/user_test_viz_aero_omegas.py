#!/usr/bin/env python
#
# Visualization Script for Aerodynamic Angles and Omegas
# Created on 2025-01-24
#
# This script reads the CSV output from global_planning and visualizes:
# (1) df_lyu_alpha over time
# (2) Omega components (spatial and body frame)
# (3) Alpha comparison (df_phi_alpha vs df_lyu_alpha)
# (4) B_ap_x_df_lyu (proper acceleration) and B_fa_z_df_lyu (aero force, mass-scaled)
# (5) S_ap_x and S_ap_x_dot (spatial aero force x and its derivative)
# (6) S_ap_z and S_ap_z_dot (spatial aero force z and its derivative)
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

    # Sort by modification time, get the latest
    files_with_time = [(f, os.path.getmtime(dir_path / f)) for f in files]
    latest_file = max(files_with_time, key=lambda x: x[1])[0]
    return str(dir_path / latest_file)


def load_trajectory_csv(csv_path: str) -> dict:
    """
    Load trajectory CSV file from global_planning.

    CSV columns (37 total):
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,B_ap_x_df_lyu,df_phi_alpha,df_lyu_alpha,
    yb_x,yb_y,yb_z,df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,
    df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,Wap_x,Wap_y,Wap_z,
    S_ap_x,S_ap_z,S_ap_x_dot,S_ap_z_dot,S_Omega_x,S_Omega_y,S_Omega_z,
    B_Omega_x_df_lyu,B_Omega_y_df_lyu,B_Omega_z_df_lyu,B_fa_z_df_lyu,alpha_dot,alpha_residual
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    return {
        "time": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "B_ap_x_df_lyu": data[:, 7],
        "df_phi_alpha": data[:, 8],
        "df_lyu_alpha": data[:, 9],
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
        "B_Omega_x_df_lyu": data[:, 31],
        "B_Omega_y_df_lyu": data[:, 32],
        "B_Omega_z_df_lyu": data[:, 33],
        "B_fa_z_df_lyu": data[:, 34],
        "alpha_dot": data[:, 35],
        "alpha_residual": data[:, 36],
    }


def plot_aero_omegas(
    data: dict, output_path: str = None, show: bool = True
) -> None:
    """
    Plot 8 subplots: df_lyu_alpha, Omegas, Alpha comparison,
    B_ap_x_df_lyu (proper acc) and B_fa_z_df_lyu (aero force),
    S_ap_x and S_ap_x_dot, S_ap_z and S_ap_z_dot,
    alpha & alpha_dot (twin plot), alpha & alpha_residual (twin plot).

    Args:
        data: Dictionary containing trajectory data
        output_path: Path to save the plot (optional)
        show: Whether to display the plot interactively
    """
    fig, axes = plt.subplots(4, 2, figsize=(14, 18))
    fig.suptitle("Aerodynamic Angles and Omegas Analysis", fontsize=14)

    t = data["time"]

    # Plot (1): df_lyu_alpha over time
    ax1 = axes[0, 0]
    ax1.plot(t, np.degrees(data["df_lyu_alpha"]), color="blue", linewidth=1.5, label="df_lyu_alpha")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Angle (degrees)")
    ax1.set_title("(1) Lyu Alpha (df_lyu_alpha) Over Time")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot (2): Omega components (stability and body frame)
    ax2 = axes[0, 1]
    # Stability frame omegas
    ax2.plot(t, data["S_Omega_x"], label="S_Omega_x (stability)", linewidth=1.5)
    ax2.plot(t, data["S_Omega_y"], label="S_Omega_y (stability)", linewidth=1.5)
    ax2.plot(t, data["S_Omega_z"], label="S_Omega_z (stability)", linewidth=1.5)
    # Body frame omegas
    ax2.plot(t, data["B_Omega_x_df_lyu"], label="B_Omega_x (body)", linewidth=1.5, linestyle="--")
    ax2.plot(t, data["B_Omega_y_df_lyu"], label="B_Omega_y (body)", linewidth=1.5, linestyle="--")
    ax2.plot(t, data["B_Omega_z_df_lyu"], label="B_Omega_z (body)", linewidth=1.5, linestyle="--")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angular Velocity (rad/s)")
    ax2.set_title("(2) Omegas: Stability (solid) & Body (dashed)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot (3): Alpha comparison (df_phi_alpha vs df_lyu_alpha)
    ax3 = axes[1, 0]
    ax3.plot(t, np.degrees(data["df_phi_alpha"]), label="df_phi_alpha", linewidth=1.5, alpha=0.8)
    ax3.plot(t, np.degrees(data["df_lyu_alpha"]), label="df_lyu_alpha", linewidth=1.5, alpha=0.8)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angle (degrees)")
    ax3.set_title("(3) Alpha Comparison (df_phi vs df_lyu)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot (4): B_ap_x_df_lyu (proper acceleration) and B_fa_z_df_lyu (aero force, mass-scaled)
    ax4 = axes[1, 1]
    ax4.plot(t, data["B_ap_x_df_lyu"], color="red", linewidth=1.5,
             label="B_ap_x_df_lyu (proper acc, x-axis)")
    ax4.plot(t, data["B_fa_z_df_lyu"]/2.0, color="blue", linewidth=1.5,
             label="B_fa_z_df_lyu (aero force, z-axis, mass-scaled)")
    ax4.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Acceleration (m/s²) / Force per mass (N/kg)")
    ax4.set_title("(4) Body Frame: B_ap_x (accel) & B_fa_z (aero force, mass-scaled)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # Plot (5): S_ap_x and S_ap_x_dot (spatial aerodynamic derivatives)
    ax5 = axes[2, 0]
    ax5.plot(t, data["S_ap_x"], color="green", linewidth=1.5, label="S_ap_x")
    ax5.plot(t, data["S_ap_x_dot"], color="orange", linewidth=1.5,
             linestyle="--", label="S_ap_x_dot (derivative)")
    ax5.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Specific Force (m/s²) / Rate (m/s³)")
    ax5.set_title("(5) S_ap_x (spatial aero force x) & S_ap_x_dot")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # Plot (6): S_ap_z and S_ap_z_dot (spatial aerodynamic derivatives)
    ax6 = axes[2, 1]
    ax6.plot(t, data["S_ap_z"], color="purple", linewidth=1.5, label="S_ap_z")
    ax6.plot(t, data["S_ap_z_dot"], color="magenta", linewidth=1.5,
             linestyle="--", label="S_ap_z_dot (derivative)")
    ax6.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Specific Force (m/s²) / Rate (m/s³)")
    ax6.set_title("(6) S_ap_z (spatial aero force z) & S_ap_z_dot")
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    # Plot (7): Alpha and alpha_dot (twin plot)
    ax7 = axes[3, 0]
    ax7.plot(t, np.degrees(data["df_lyu_alpha"]), color="blue", linewidth=1.5, label="df_lyu_alpha")
    ax7.set_xlabel("Time (s)")
    ax7.set_ylabel("Angle (degrees)", color="blue")
    ax7.tick_params(axis="y", labelcolor="blue")
    ax7.set_title("(7) Alpha (df_lyu_alpha) & Alpha_dot")
    ax7.grid(True, alpha=0.3)

    # Twin axis for alpha_dot
    ax7_dot = ax7.twinx()
    ax7_dot.plot(t, data["alpha_dot"], color="red", linewidth=1.5,
                 linestyle="--", label="alpha_dot", alpha=0.7)
    ax7_dot.set_ylabel("Alpha_dot (rad/s)", color="red")
    ax7_dot.tick_params(axis="y", labelcolor="red")

    # Combine legends
    lines1, labels1 = ax7.get_legend_handles_labels()
    lines2, labels2 = ax7_dot.get_legend_handles_labels()
    ax7.legend(lines1 + lines2, labels1 + labels2, loc="best")

    # Plot (8): Alpha and alpha_residual (twin plot)
    ax8 = axes[3, 1]
    ax8.plot(t, np.degrees(data["df_lyu_alpha"]), color="blue", linewidth=1.5, label="df_lyu_alpha")
    ax8.set_xlabel("Time (s)")
    ax8.set_ylabel("Angle (degrees)", color="blue")
    ax8.tick_params(axis="y", labelcolor="blue")
    ax8.set_title("(8) Alpha (df_lyu_alpha) & Alpha_residual")
    ax8.grid(True, alpha=0.3)

    # Twin axis for alpha_residual
    ax8_residual = ax8.twinx()
    ax8_residual.plot(t, data["alpha_residual"], color="green", linewidth=1.5,
                      linestyle="--", label="alpha_residual", alpha=0.7)
    ax8_residual.set_ylabel("Residual", color="green")
    ax8_residual.tick_params(axis="y", labelcolor="green")

    # Combine legends
    lines1, labels1 = ax8.get_legend_handles_labels()
    lines2, labels2 = ax8_residual.get_legend_handles_labels()
    ax8.legend(lines1 + lines2, labels1 + labels2, loc="best")

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✅ Plot saved to {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def print_statistics(data: dict) -> None:
    """Print statistics about the aerodynamic data."""
    print("\n" + "=" * 60)
    print("Aerodynamic Data Statistics")
    print("=" * 60)
    print(f"Time range: {data['time'].min():.2f} - {data['time'].max():.2f} s")
    print(f"Number of samples: {len(data['time'])}")
    print()

    # df_lyu_alpha statistics
    lyu_alpha_deg = np.degrees(data["df_lyu_alpha"])
    print("df_lyu_alpha (Lyu alpha):")
    print(f"  Range: [{lyu_alpha_deg.min():.2f}, {lyu_alpha_deg.max():.2f}] degrees")
    print(f"  Mean:  {lyu_alpha_deg.mean():.2f} degrees")
    print(f"  Std:   {lyu_alpha_deg.std():.2f} degrees")
    print()

    # Alpha comparison
    phi_alpha_deg = np.degrees(data["df_phi_alpha"])
    alpha_diff = np.abs(phi_alpha_deg - lyu_alpha_deg)
    print("Alpha comparison (df_phi_alpha vs df_lyu_alpha):")
    print(f"  Max difference: {alpha_diff.max():.4f} degrees")
    print(f"  Mean difference: {alpha_diff.mean():.4f} degrees")
    print()

    # B_ap_x_df_lyu statistics (proper acceleration, not force)
    b_ap_x = data["B_ap_x_df_lyu"]
    print("B_ap_x_df_lyu (Body frame X-axis proper acceleration):")
    print(f"  Range: [{b_ap_x.min():.4f}, {b_ap_x.max():.4f}] m/s²")
    print(f"  Mean:  {b_ap_x.mean():.4f} m/s²")
    print(f"  Std:   {b_ap_x.std():.4f} m/s²")
    print()

    # B_fa_z_df_lyu statistics (aerodynamic force, mass-scaled)
    b_fa_z = data["B_fa_z_df_lyu"]
    print("B_fa_z_df_lyu (Body frame Z-axis aero force, mass-scaled):")
    print(f"  Range: [{b_fa_z.min():.4f}, {b_fa_z.max():.4f}] N/kg")
    print(f"  Mean:  {b_fa_z.mean():.4f} N/kg")
    print(f"  Std:   {b_fa_z.std():.4f} N/kg")
    print()

    # S_Omega statistics
    print("Stability frame omegas (S_Omega):")
    for name, values in [("x", data["S_Omega_x"]), ("y", data["S_Omega_y"]), ("z", data["S_Omega_z"])]:
        print(f"  S_Omega_{name}: [{values.min():.4f}, {values.max():.4f}] rad/s")
    print()

    # S_ap statistics (spatial aerodynamic specific force components)
    print("Spatial aerodynamic specific force components (S_ap):")
    for name, values in [("x", data["S_ap_x"]), ("z", data["S_ap_z"])]:
        print(f"  S_ap_{name}: [{values.min():.4f}, {values.max():.4f}] m/s²")
    print()

    # S_ap_dot statistics (spatial aerodynamic specific force rates)
    print("Spatial aerodynamic specific force rates (S_ap_dot):")
    for name, values in [("x", data["S_ap_x_dot"]), ("z", data["S_ap_z_dot"])]:
        print(f"  S_ap_{name}_dot: [{values.min():.4f}, {values.max():.4f}] m/s³")
    print()

    # B_Omega_df_lyu statistics
    print("Body frame omegas (B_Omega_df_lyu):")
    for name, values in [
        ("x", data["B_Omega_x_df_lyu"]),
        ("y", data["B_Omega_y_df_lyu"]),
        ("z", data["B_Omega_z_df_lyu"]),
    ]:
        print(f"  B_Omega_{name}_df_lyu: [{values.min():.4f}, {values.max():.4f}] rad/s")
    print()

    # Alpha_dot statistics
    print("Alpha_dot (from alpha solver):")
    print(f"  Range: [{data['alpha_dot'].min():.6f}, {data['alpha_dot'].max():.6f}] rad/s")
    print(f"  Mean:  {data['alpha_dot'].mean():.6f} rad/s")
    print(f"  Std:   {data['alpha_dot'].std():.6f} rad/s")
    print()

    # Alpha_residual statistics
    print("Alpha_residual (solver residual |f(alpha)|):")
    print(f"  Range: [{data['alpha_residual'].min():.6e}, {data['alpha_residual'].max():.6e}]")
    print(f"  Mean:  {data['alpha_residual'].mean():.6e}")
    print(f"  Std:   {data['alpha_residual'].std():.6e}")
    print(f"  Max residual: {data['alpha_residual'].max():.6e} (should be < tolerance)")

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
            print("Usage: user_test_viz_aero_omegas.py [csv_file] [output_path]")
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
    plot_aero_omegas(data, output_path=output_path, show=show_plot)

    if output_path:
        print(f"\n✅ Done! Plot saved to {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
