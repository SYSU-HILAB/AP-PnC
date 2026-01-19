"""
Shared trajectory visualization helpers.

feature/trajectory_visualization keeps the CLI entry-point, while reusable
logic lives here so other tools (or notebooks) can import the plotting pieces.
"""

from __future__ import annotations

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from matplotlib.collections import LineCollection


def find_latest_planning_csv(artifacts_dir: str | Path | None = None) -> str | None:
    """
    Find the latest planning CSV file in the artifacts directory.

    Args:
        artifacts_dir: Path to artifacts/simulations/temp_data.
                      If None, searches relative to this file's location.

    Returns:
        Path to the latest planning CSV file, or None if not found.
    """
    if artifacts_dir is None:
        repo_root = Path(__file__).parent.parent.parent
        artifacts_dir = repo_root / "artifacts" / "simulations" / "temp_data"
    else:
        artifacts_dir = Path(artifacts_dir)

    if not artifacts_dir.exists():
        return None

    planning_files = list(artifacts_dir.glob("planning_*.csv"))
    if not planning_files:
        return None

    latest_file = max(planning_files, key=lambda p: p.stat().st_mtime)
    return str(latest_file)


def validate_input_data(csv_path: str) -> pd.DataFrame:
    """Ensure the trajectory CSV exists and has the expected columns."""
    if not os.path.exists(csv_path):
        raise ValueError("Planning data not found")

    df = pd.read_csv(csv_path)
    required_columns = ["pos_x", "pos_y", "vel_x", "vel_y", "vel_z"]
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        raise ValueError(f"Missing data: required columns {missing_columns} not found")
    return df


def validate_routes_data(csv_path: str) -> pd.DataFrame:
    """Ensure the optional routes CSV exists and has the expected columns."""
    if not os.path.exists(csv_path):
        raise ValueError("Routes data not found")

    df = pd.read_csv(csv_path)
    required_columns = ["pos_x", "pos_y"]
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        raise ValueError(f"Missing routes data: required columns {missing_columns} not found")
    return df


def load_config(config_path: Path | None) -> dict:
    """Load the subset of planning parameters needed for visualization."""
    if not config_path or not config_path.exists():
        params = {}
    else:
        try:
            with open(config_path, encoding="utf-8") as file:
                config = yaml.safe_load(file)
            params = config.get("/global_planning_node", {}).get("ros__parameters", {})
        except Exception as err:  # pragma: no cover - defensive logging
            print(f"Warning: Could not load config file {config_path}: {err}")
            params = {}

    return {
        "thrust_max": params.get("thrust_max", 8.5),
        "thrust_min": params.get("thrust_min", -3.5),
        "vel_max": params.get("v_max", 14.0),
        "BasicTrajectoryType": params.get("BasicTrajectoryType", 1),
    }


def calculate_velocity_magnitude(df: pd.DataFrame) -> np.ndarray:
    """Compute the velocity magnitude from component columns."""
    return np.sqrt(df["vel_x"] ** 2 + df["vel_y"] ** 2 + df["vel_z"] ** 2)


def apply_color_mapping(vel_magnitude: np.ndarray) -> np.ndarray:
    """
    Map velocity magnitudes to colors using a normalized cubic transform.
    """
    vel_min, vel_max = vel_magnitude.min(), vel_magnitude.max()
    if vel_max > vel_min:
        normalized_val = (vel_magnitude - vel_min) / (vel_max - vel_min)
    else:
        normalized_val = np.zeros_like(vel_magnitude)
    return normalized_val**3


def create_visualization(
    df: pd.DataFrame,
    config: dict,
    output_path: Path,
    routes_df: pd.DataFrame | None = None,
) -> Path:
    """Render the XY trajectory with velocity-based coloring."""
    vel_magnitude = calculate_velocity_magnitude(df)
    color_values = apply_color_mapping(vel_magnitude)

    fig, ax = plt.subplots(figsize=(10, 8))
    points = np.array([df["pos_x"].values, df["pos_y"].values]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    segment_colors = (color_values[:-1] + color_values[1:]) / 2

    lc = LineCollection(segments, cmap="jet", linewidth=2)
    lc.set_array(segment_colors)
    ax.add_collection(lc)

    if routes_df is not None:
        ax.scatter(
            routes_df["pos_x"],
            routes_df["pos_y"],
            c="gray",
            s=20,
            alpha=0.7,
            zorder=5,
            label="Waypoints",
        )

    x_min = min(
        df["pos_x"].min(), routes_df["pos_x"].min() if routes_df is not None else float("inf")
    )
    x_max = max(
        df["pos_x"].max(), routes_df["pos_x"].max() if routes_df is not None else float("-inf")
    )
    y_min = min(
        df["pos_y"].min(), routes_df["pos_y"].min() if routes_df is not None else float("inf")
    )
    y_max = max(
        df["pos_y"].max(), routes_df["pos_y"].max() if routes_df is not None else float("-inf")
    )
    x_padding = (x_max - x_min) * 0.05
    y_padding = (y_max - y_min) * 0.05

    ax.set_xlim(x_min - x_padding, x_max + x_padding)
    ax.set_ylim(y_min - y_padding, y_max + y_padding)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Trajectory Visualization with Velocity-Based Coloring")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_aspect("equal", adjustable="box")

    cbar = fig.colorbar(lc, ax=ax, label="Normalized Velocity^3")
    cbar.set_label("Velocity Intensity")

    if routes_df is not None:
        ax.legend()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return output_path


def plot_trajectory_from_csv(
    trajectory_csv: Path,
    routes_csv: Path | None = None,
    config_yaml: Path | None = None,
    output_path: Path | None = None,
) -> Path:
    """End-to-end helper used by CLI tools to render trajectories."""
    df = validate_input_data(str(trajectory_csv))
    routes_df = None
    if routes_csv is not None:
        try:
            routes_df = validate_routes_data(str(routes_csv))
        except ValueError as exc:
            print(f"Warning: {exc}")

    config = load_config(config_yaml)

    output_path = output_path or trajectory_csv.with_suffix(".png")
    return create_visualization(df, config, output_path, routes_df)


def generate_output_filename(config: dict, file_extension: str = "png") -> str:
    """Build a deterministic filename from configuration parameters."""
    trajectory_type = config.get("BasicTrajectoryType", 1)
    vel_max = config.get("vel_max", 14.0)
    thrust_min = config.get("thrust_min", -3.5)
    thrust_max = config.get("thrust_max", 8.5)

    return f"{trajectory_type}_vel{vel_max}_thrust{thrust_min}-{thrust_max}.{file_extension}"


def generate_visualization_outputs(
    trajectory_csv: Path,
    routes_csv: Path | None,
    config_yaml: Path | None,
    output_dir: Path,
) -> dict:
    """
    Produce both the visualization PNG and a renamed CSV copy.

    Returns a mapping with `png` and `csv` entries for convenience.
    """
    trajectory_csv = Path(trajectory_csv)
    routes_csv = Path(routes_csv) if routes_csv else None
    output_dir = Path(output_dir)

    df = validate_input_data(str(trajectory_csv))
    routes_df = None
    if routes_csv is not None:
        try:
            routes_df = validate_routes_data(str(routes_csv))
        except ValueError as exc:  # pragma: no cover - best-effort input
            print(f"Warning: {exc}")

    config = load_config(config_yaml)
    output_dir.mkdir(parents=True, exist_ok=True)

    png_filename = generate_output_filename(config, "png")
    csv_filename = generate_output_filename(config, "csv")
    png_path = output_dir / png_filename
    csv_path = output_dir / csv_filename

    df.to_csv(csv_path, index=False)
    create_visualization(df, config, png_path, routes_df)

    return {"png": png_path, "csv": csv_path}
