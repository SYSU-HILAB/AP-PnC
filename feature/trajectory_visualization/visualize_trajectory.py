#!/usr/bin/env python3
"""
Trajectory Visualization Tool
Creates 2D position visualization from trajectory planning data with velocity-based coloring
and optional routes scatter points.

Author: Claude Code
"""

import os
import sys
from pathlib import Path

import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

REPO_ROOT = Path(os.environ.get('AP_PNC_DIR', Path(__file__).resolve().parents[2]))
ARTIFACTS_ROOT = REPO_ROOT / "artifacts"
TEMP_DATA_DIR = ARTIFACTS_ROOT / "simulations" / "temp_data"
DEFAULT_FIGS_DIR = ARTIFACTS_ROOT / "visualization" / "figs"

def validate_input_data(csv_path):
    """
    Validate that the input CSV file exists and contains required columns.

    Args:
        csv_path (str): Path to the CSV file

    Raises:
        ValueError: If file doesn't exist or missing required columns
    """
    if not os.path.exists(csv_path):
        raise ValueError("Planning data not found")

    try:
        df = pd.read_csv(csv_path)
        required_columns = ['pos_x', 'pos_y', 'vel_x', 'vel_y', 'vel_z']

        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            raise ValueError(f"Missing data: required columns {missing_columns} not found")

        return df
    except Exception as e:
        if isinstance(e, ValueError):
            raise
        raise ValueError(f"Error reading CSV file: {e}")


def validate_routes_data(csv_path):
    """
    Validate that the routes CSV file exists and contains required columns.

    Args:
        csv_path (str): Path to the routes CSV file

    Raises:
        ValueError: If file doesn't exist or missing required columns
    """
    if not os.path.exists(csv_path):
        raise ValueError("Routes data not found")

    try:
        df = pd.read_csv(csv_path)
        required_columns = ['pos_x', 'pos_y']

        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            raise ValueError(f"Missing routes data: required columns {missing_columns} not found")

        return df
    except Exception as e:
        if isinstance(e, ValueError):
            raise
        raise ValueError(f"Error reading routes CSV file: {e}")


def load_config(config_path):
    """
    Load configuration parameters from YAML file.

    Args:
        config_path (str): Path to the YAML configuration file

    Returns:
        dict: Configuration parameters
    """
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Extract parameters from /global_planning_node/ros__parameters
        params = config.get('/global_planning_node', {}).get('ros__parameters', {})

        return {
            'thrust_max': params.get('thrust_max', 8.5),
            'thrust_min': params.get('thrust_min', -3.5),
            'vel_max': params.get('v_max', 14.0),
            'BasicTrajectoryType': params.get('BasicTrajectoryType', 1)
        }
    except Exception as e:
        print(f"Warning: Could not load config file {config_path}: {e}")
        # Return default values
        return {
            'thrust_max': 8.5,
            'thrust_min': -3.5,
            'vel_max': 14.0,
            'BasicTrajectoryType': 1
        }


def calculate_velocity_magnitude(df):
    """
    Calculate velocity magnitude from velocity components.

    Args:
        df (pd.DataFrame): DataFrame with velocity columns

    Returns:
        np.array: Velocity magnitude values
    """
    vel_magnitude = np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2)
    return vel_magnitude


def apply_color_mapping(vel_magnitude):
    """
    Apply color mapping with normalization and cubic transformation.

    Args:
        vel_magnitude (np.array): Velocity magnitude values

    Returns:
        np.array: Color values for mapping
    """
    # Normalize to [0, 1]
    vel_min, vel_max = vel_magnitude.min(), vel_magnitude.max()
    if vel_max > vel_min:
        normalized_val = (vel_magnitude - vel_min) / (vel_max - vel_min)
    else:
        normalized_val = np.zeros_like(vel_magnitude)

    # Apply cubic transformation: x^3
    color_val = normalized_val ** 3

    return color_val


def create_visualization(df, config, output_path, routes_df=None):
    """
    Create 2D trajectory visualization with velocity-based coloring and routes scatter points.

    Args:
        df (pd.DataFrame): Trajectory data
        config (dict): Configuration parameters
        output_path (str): Output file path for the PNG
        routes_df (pd.DataFrame, optional): Routes data to plot as scatter points
    """
    # Calculate velocity magnitude and color mapping
    vel_magnitude = calculate_velocity_magnitude(df)
    color_values = apply_color_mapping(vel_magnitude)

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))

    # Create line segments with interpolated colors for smooth transitions
    from matplotlib.collections import LineCollection

    # Prepare points for LineCollection
    points = np.array([df['pos_x'].values, df['pos_y'].values]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Create color values for each segment (use average of endpoint colors)
    segment_colors = (color_values[:-1] + color_values[1:]) / 2

    # Create line collection with smooth color transitions
    lc = LineCollection(segments, cmap='jet', linewidth=2)
    lc.set_array(segment_colors)

    # Add the line collection to the plot
    line = ax.add_collection(lc)

    # Add routes as scatter points if provided
    if routes_df is not None:
        # Plot routes as red scatter points with some transparency
        ax.scatter(routes_df['pos_x'], routes_df['pos_y'],
                  c='gray', s=20, alpha=0.7, zorder=5, label='Waypoints')

    # Set axis limits based on both trajectory and routes data
    x_min = min(df['pos_x'].min(), routes_df['pos_x'].min() if routes_df is not None else float('inf'))
    x_max = max(df['pos_x'].max(), routes_df['pos_x'].max() if routes_df is not None else float('-inf'))
    y_min = min(df['pos_y'].min(), routes_df['pos_y'].min() if routes_df is not None else float('inf'))
    y_max = max(df['pos_y'].max(), routes_df['pos_y'].max() if routes_df is not None else float('-inf'))

    # Add some padding to the limits
    x_padding = (x_max - x_min) * 0.05
    y_padding = (y_max - y_min) * 0.05

    ax.set_xlim(x_min - x_padding, x_max + x_padding)
    ax.set_ylim(y_min - y_padding, y_max + y_padding)

    # Remove axes, grids, and labels as specified
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(False)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')

    # Add colorbar with feathered edge
    cbar = plt.colorbar(line, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Velocity Magnitude', rotation=270, labelpad=15)
    # Remove numerical annotations from colorbar
    cbar.set_ticks([])

    # Add legend if routes are plotted
    if routes_df is not None:
        ax.legend(loc='upper right', frameon=False)

    # Set aspect ratio for better visualization
    ax.set_aspect('equal', adjustable='box')

    # Save the figure
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()


def generate_output_filename(config, file_extension='png'):
    """
    Generate output filename based on configuration parameters.

    Args:
        config (dict): Configuration parameters
        file_extension (str): File extension ('png' or 'csv')

    Returns:
        str: Generated filename
    """
    trajectory_type = config.get('BasicTrajectoryType', 1)
    vel_max = config.get('vel_max', 14.0)
    thrust_min = config.get('thrust_min', -3.5)
    thrust_max = config.get('thrust_max', 8.5)

    filename = f"{trajectory_type}_vel{vel_max}_thrust{thrust_min}-{thrust_max}.{file_extension}"
    return filename


def find_planning_file():
    """
    Auto-detect the first available planning CSV file in the artifacts/simulations/temp_data directory.

    Returns:
        str: Path to the found planning file, or default if none found
    """
    planning_files = sorted(TEMP_DATA_DIR.glob('planning_*.csv'))
    if planning_files:
        return str(planning_files[0])
    return str(TEMP_DATA_DIR / 'planning.csv')


def main():
    """Main function to execute trajectory visualization with routes."""
    parser = argparse.ArgumentParser(description='Create trajectory visualization from planning data with routes')
    parser.add_argument('--csv', default=None,
                       help='Path to planning CSV file (auto-detected if not specified)')
    parser.add_argument('--routes', default=None,
                       help='Path to routes CSV file (optional)')
    parser.add_argument('--config', default='src/trajectory_server/config/global_planning.yaml',
                       help='Path to planning configuration YAML file')
    parser.add_argument('--output-dir', default=str(DEFAULT_FIGS_DIR),
                       help='Output directory for generated figures')

    args = parser.parse_args()

    # Auto-detect planning file if not specified
    if args.csv is None:
        args.csv = find_planning_file()
        print(f"Auto-detected planning file: {args.csv}")

    # Auto-detect routes file if planning file is specified but routes is not
    if args.routes is None and args.csv is not None:
        # Replace 'planning_' with 'routes_' in the planning file path
        planning_path = Path(args.csv)
        if 'planning_' in planning_path.name:
            candidate = planning_path.with_name(planning_path.name.replace('planning_', 'routes_', 1))
            if candidate.exists():
                args.routes = str(candidate)
                print(f"Auto-detected routes file: {args.routes}")
            else:
                args.routes = None

    try:
        # Validate input data
        print("Validating input data...")
        df = validate_input_data(args.csv)
        print(f"✓ Loaded {len(df)} data points from {args.csv}")

        # Load and validate routes data if provided
        routes_df = None
        if args.routes:
            routes_df = validate_routes_data(args.routes)
            print(f"✓ Loaded {len(routes_df)} route points from {args.routes}")

        # Load configuration
        print("Loading configuration...")
        config = load_config(args.config)
        print(f"✓ Loaded configuration: {config}")

        # Create output directory if it doesn't exist
        output_dir = Path(args.output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate output filenames
        png_filename = generate_output_filename(config, 'png')
        csv_filename = generate_output_filename(config, 'csv')

        png_path = output_dir / png_filename
        csv_output_path = output_dir / csv_filename

        # Copy/rename CSV file to output directory with new name
        print("Creating output files...")
        df.to_csv(csv_output_path, index=False)
        print(f"✓ Saved renamed CSV to {csv_output_path}")

        # Create visualization
        print("Creating visualization...")
        if routes_df is not None:
            print("  - Including routes as scatter points")
        create_visualization(df, config, str(png_path), routes_df)
        print(f"✓ Saved visualization to {png_path}")

        print("\n✅ Trajectory visualization completed successfully!")
        print(f"📊 Generated files:")
        print(f"   - PNG: {png_path}")
        print(f"   - CSV: {csv_output_path}")

    except ValueError as e:
        print(f"❌ Validation Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
