#!/usr/bin/env python3
"""
CLI entry point for trajectory visualization.

The heavy lifting now lives in feature.utils.traj_visualizer so other tools can
reuse the plotting logic without copying code or tweaking sys.path.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from feature.utils.traj_visualizer import (
    generate_visualization_outputs,
)


REPO_ROOT = Path(os.environ.get("AP_PNC_DIR", Path(__file__).resolve().parents[2]))
ARTIFACTS_ROOT = REPO_ROOT / "artifacts"
TEMP_DATA_DIR = ARTIFACTS_ROOT / "simulations" / "temp_data"
DEFAULT_FIGS_DIR = ARTIFACTS_ROOT / "visualization" / "figs"


def find_planning_file() -> Path:
    """Auto-detect the first available planning CSV file."""
    planning_files = sorted(TEMP_DATA_DIR.glob("planning_*.csv"))
    if planning_files:
        return planning_files[0]
    return TEMP_DATA_DIR / "planning.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create trajectory visualization from planning data."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Path to planning CSV file (auto-detected if not provided)",
    )
    parser.add_argument(
        "--routes",
        type=Path,
        default=None,
        help="Optional routes CSV file",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("src/trajectory_server/config/global_planning.yaml"),
        help="Planning config YAML",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_FIGS_DIR,
        help="Directory for generated figures",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    trajectory_csv = args.csv or find_planning_file()

    if args.routes is None and trajectory_csv.name.startswith("planning_"):
        candidate = trajectory_csv.with_name(
            trajectory_csv.name.replace("planning_", "routes_", 1)
        )
        if candidate.exists():
            args.routes = candidate

    generate_visualization_outputs(
        trajectory_csv=trajectory_csv,
        routes_csv=args.routes,
        config_yaml=args.config,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
