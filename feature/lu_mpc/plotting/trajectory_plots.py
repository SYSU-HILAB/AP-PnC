"""
Trajectory plotting functions for time-series data from global_planning.

The CSV files coming out of `artifacts/simulations/temp_data` share a fixed schema.
Every plotter in this module needs the following columns (all float):

* `time` – seconds since the trajectory start.
* `vel_x`, `vel_y`, `vel_z` – body velocity components in meters per second.
* `df_alpha` – differential-flatness command for angle of attack (radians).
* `real_alpha` – lookup-table corrected angle of attack (radians).

These columns are parsed into a structured numpy array via `np.genfromtxt` so consumers
can access fields as `data['time']` etc.
"""

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_planning_csv(csv_path):
    """
    Load planning data from CSV file.

    Args:
        csv_path (str): Path to the planning CSV file.

    Returns:
        dict: Dictionary containing the trajectory data.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    # Load CSV with header
    data = np.genfromtxt(csv_path, delimiter=',', names=True)

    return data


def plot_trajectory_over_time(
    csv_path=None,
    data=None,
    show_plot=True,
    save_path=None
):
    """
    Plot velocity magnitude, df_alpha, and real_alpha over time.

    Args:
        csv_path (str, optional): Path to the planning CSV file.
        data (numpy.ndarray, optional): Pre-loaded data with named columns.
        show_plot (bool): Whether to display the plot. Default: True.
        save_path (str, optional): Path to save the plot figure.

    Returns:
        numpy.ndarray: The trajectory data used for plotting.
    """
    # Load data
    if data is None:
        if csv_path is None:
            raise ValueError("Either csv_path or data must be provided")
        data = load_planning_csv(csv_path)

    # Extract data columns
    t = data['time']
    vel_x = data['vel_x']
    vel_y = data['vel_y']
    vel_z = data['vel_z']
    df_alpha = data['df_alpha']
    real_alpha = data['real_alpha']

    # Calculate velocity magnitude
    vel = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

    # Create figure with two rows and two columns
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Trajectory Time Series Analysis', fontsize=16)

    # Velocity magnitude over time
    ax1 = axs[0, 0]
    ax1.plot(t, vel, 'b-', linewidth=2, label='Velocity Magnitude')
    ax1.set_xlabel('Time [s]', fontsize=12)
    ax1.set_ylabel('Velocity [m/s]', fontsize=12)
    ax1.set_title('Velocity Magnitude over Time', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # df_alpha over time
    ax2 = axs[0, 1]
    ax2.plot(t, np.degrees(df_alpha), 'r-', linewidth=2, label='df_alpha')
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('Angle of Attack [deg]', fontsize=12)
    ax2.set_title('df_alpha (Differential Flatness) over Time', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    # real_alpha over time
    ax3 = axs[1, 0]
    ax3.plot(t, np.degrees(real_alpha), 'g-', linewidth=2, label='real_alpha')
    ax3.set_xlabel('Time [s]', fontsize=12)
    ax3.set_ylabel('Angle of Attack [deg]', fontsize=12)
    ax3.set_title('real_alpha (Lookup Table Corrected) over Time', fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    # df_alpha vs real_alpha comparison
    ax4 = axs[1, 1]
    ax4.plot(t, np.degrees(df_alpha), 'r-', linewidth=2, label='df_alpha', alpha=0.7)
    ax4.plot(t, np.degrees(real_alpha), 'g-', linewidth=2, label='real_alpha', alpha=0.7)
    ax4.set_xlabel('Time [s]', fontsize=12)
    ax4.set_ylabel('Angle of Attack [deg]', fontsize=12)
    ax4.set_title('df_alpha vs real_alpha Comparison', fontsize=14)
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")

    if show_plot:
        plt.show()

    print("Trajectory time series plot completed!")
    return data


def find_latest_planning_csv(artifacts_dir=None):
    """
    Find the latest planning CSV file in the artifacts directory.

    Args:
        artifacts_dir (str, optional): Path to artifacts/simulations/temp_data.
                                      If None, uses environment variable or default.

    Returns:
        str: Path to the latest planning CSV file, or None if not found.
    """
    if artifacts_dir is None:
        if ap_pnc_dir := os.getenv('AP_PNC_DIR'):
            artifacts_dir = os.path.join(ap_pnc_dir, 'artifacts', 'simulations', 'temp_data')
        else:
            artifacts_dir = 'artifacts/simulations/temp_data'

    artifacts_path = Path(artifacts_dir)
    if not artifacts_path.exists():
        print(f"Artifacts directory not found: {artifacts_dir}")
        return None

    # Find all planning_*.csv files
    planning_files = list(artifacts_path.glob('planning_*.csv'))
    if not planning_files:
        print(f"No planning CSV files found in: {artifacts_dir}")
        return None

    # Sort by modification time, return the latest
    latest_file = max(planning_files, key=lambda p: p.stat().st_mtime)
    print(f"Found latest planning CSV: {latest_file}")
    return str(latest_file)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Plot trajectory time series data from planning CSV"
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        help="Path to the planning CSV file. If not provided, searches for the latest file."
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Don't display the plot"
    )
    parser.add_argument(
        "--save",
        type=str,
        help="Path to save the plot figure"
    )

    args = parser.parse_args()

    # Determine CSV path
    csv_path = args.csv_path
    if csv_path is None:
        csv_path = find_latest_planning_csv()
        if csv_path is None:
            print("Error: No CSV file found. Please specify a path.")
            exit(1)

    plot_trajectory_over_time(
        csv_path=csv_path,
        show_plot=not args.no_show,
        save_path=args.save
    )
