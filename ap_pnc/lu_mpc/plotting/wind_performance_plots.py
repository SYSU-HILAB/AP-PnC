"""
Wind Performance Testing Visualization

This module provides plotting utilities for wind performance test results,
including scatter plots of MaxAE (maximum absolute position error).
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import TwoSlopeNorm
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

if TYPE_CHECKING:
    from pandas import DataFrame


def set_plotting_style() -> None:
    """Set consistent plotting style for all figures."""
    import matplotlib
    matplotlib.use("pgf")
    plt.rcParams.update({
        "font.family": "Times New Roman",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "text.usetex": True,
        "pgf.rcfonts": False,
        "pgf.texsystem": "pdflatex",
    })


def plot_maxae_scatter(
    results_df: DataFrame,
    output_dir: str | Path,
) -> None:
    """
    Generate scatter plot: side wind (y-axis) vs longitudinal wind (x-axis) with MaxAE as color.

    MaxAE is the maximum absolute position error between pos_ref and pos.
    Failed cases (success=False) are marked with red "x" markers.

    Args:
        results_df: DataFrame with columns [velocity, wind_side, wind_longitudinal, max_error, success, ...]
        output_dir: Directory to save the plot
    """
    set_plotting_style()

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Get unique velocity
    velocities = sorted(results_df["velocity"].unique())

    if len(velocities) == 1:
        velocity = velocities[0]
        vel_data = results_df[results_df["velocity"] == velocity]
    else:
        vel_data = results_df

    # Separate successful and failed cases
    success_data = vel_data[vel_data["success"] == 1.0]
    failed_data = vel_data[vel_data["success"] == 0.0]

    # Create figure
    fig_width = 3.5
    fig_height = fig_width * 0.5
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    # Train classifier and create decision boundary (draw first, so it's at the bottom)
    if len(success_data) > 0:
        # Combine successful and failed cases for SVM training
        # Failed cases are treated as max_error = 12 (high error)
        X_success = success_data[["wind_side", "wind_longitudinal"]].values
        y_success = (success_data["max_error"].values >= 1.0).astype(int)

        if len(failed_data) > 0:
            X_failed = failed_data[["wind_side", "wind_longitudinal"]].values
            y_failed = np.ones(len(X_failed), dtype=int)
            X = np.vstack([X_success, X_failed])
            y = np.concatenate([y_success, y_failed])
        else:
            X = X_success
            y = y_success

        # Train Gaussian Process Classifier with RBF kernel
        kernel = C(1.0) * RBF(length_scale=2.0)
        clf = GaussianProcessClassifier(kernel=kernel)
        clf.fit(X, y)

        # Create mesh grid for decision boundary over full plot range
        ws_min, ws_max = -5.5, 5.5
        wl_min, wl_max = -7.0, 7.0
        xx, yy = np.meshgrid(
            np.linspace(ws_min, ws_max, 200),
            np.linspace(wl_min, wl_max, 200),
        )

        # Get decision probabilities (probability of class 1: maxAE >= 1m)
        Z = clf.predict_proba(np.c_[xx.ravel(), yy.ravel()])[:, 1]
        Z = Z.reshape(xx.shape)

        # Add shaded region for high error area (draw first, at bottom)
        ax.contourf(
            xx, yy, Z, levels=[0.1, 1.0], colors=["red"], alpha=0.05, zorder=0,
        )

    # Create scatter plot with color mapped to MaxAE for successful cases
    if len(success_data) > 0:
        # Use TwoSlopeNorm with vcenter=0.6 for better contrast in [0, 0.6] range
        # [0, 0.6] occupies 50% of colormap, [0.6, max] occupies 50%
        max_val = success_data["max_error"].max()
        norm = TwoSlopeNorm(vmin=0, vcenter=0.6, vmax=max_val)

        scatter = ax.scatter(
            success_data["wind_side"],
            success_data["wind_longitudinal"],
            c=success_data["max_error"],
            cmap="viridis",
            norm=norm,
            s=2,
            edgecolors="black",
            linewidths=0.2,
            zorder=5,
            label="Success",
        )

        # Add colorbar
        cbar = fig.colorbar(scatter, ax=ax)
        cbar.set_label("MaxAE [m]", rotation=270, labelpad=15)
        cbar.ax.tick_params(width=0.5, length=3)
        for spine in cbar.ax.spines.values():
            spine.set_linewidth(0.2)
        cbar.set_ticks([0.3, 1, 3, 9])

    # Plot failed cases with red "x" markers
    if len(failed_data) > 0:
        ax.scatter(
            failed_data["wind_side"],
            failed_data["wind_longitudinal"],
            marker="x",
            color="red",
            s=2,
            linewidths=0.2,
            zorder=7,
            label="NMPC Failed",
        )

    # Draw decision boundary line on top of the shaded region
    if len(success_data) > 0:
        ax.contour(
            xx, yy, Z, levels=[0.1], colors=["red"], linewidths=0.5,
            linestyles="--", zorder=6,
        )

    ax.set_xlabel("Crosswind~[m/s]")
    ax.set_ylabel("Longitudinal Wind~[m/s]")
    ax.set_xlim(-5.5, 5.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.axvline(0, color="gray", linewidth=0.5, zorder=1)
    ax.axhline(0, color="gray", linewidth=0.5, zorder=1)
    ax.grid(True, alpha=0.2)

    plt.tight_layout()
    # Save as PDF (pgf backend)
    fig_path_pdf = output_dir / "maxae_scatter.pdf"
    plt.savefig(fig_path_pdf)

    # Also save as SVG for smaller file size (switch backend temporarily)
    import matplotlib
    current_backend = matplotlib.get_backend()
    matplotlib.use("svg")
    fig_path_svg = output_dir / "maxae_scatter.svg"
    plt.savefig(fig_path_svg)
    matplotlib.use(current_backend)

    plt.close()

    print(f"[Plots] Saved {fig_path_pdf}")
    print(f"[Plots] Saved {fig_path_svg}")


def create_combined_summary_table(
    results_df: DataFrame,
    output_dir: str | Path,
) -> str:
    """
    Create a formatted summary table and save as CSV.

    Args:
        results_df: DataFrame with all results
        output_dir: Directory to save the table

    Returns:
        Path to the saved CSV file
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Sort by velocity, then wind_side, then wind_longitudinal
    results_df = results_df.sort_values(
        by=["velocity", "wind_side", "wind_longitudinal"]
    )

    # Select and rename columns for readability
    table_df = results_df[[
        "velocity",
        "wind_side",
        "wind_longitudinal",
        "rms_error",
        "max_error",
        "success",
    ]].copy()

    table_df.columns = [
        "Velocity [m/s]",
        "Side Wind [m/s]",
        "Longitudinal Wind [m/s]",
        "RMS Error [m]",
        "MaxAE [m]",
        "Success",
    ]

    # Save as CSV
    csv_path = output_dir / "wind_performance_combined.csv"
    table_df.to_csv(csv_path, index=False)

    print(f"[Plots] Saved combined table to {csv_path}")

    # Also create a human-readable text version
    txt_path = output_dir / "wind_performance_summary.txt"
    with open(txt_path, "w") as f:
        f.write("=" * 80 + "\n")
        f.write("WIND PERFORMANCE TEST RESULTS SUMMARY\n")
        f.write("=" * 80 + "\n\n")

        for velocity in sorted(results_df["velocity"].unique()):
            vel_data = results_df[results_df["velocity"] == velocity]

            f.write(f"Velocity: {velocity:.0f} m/s\n")
            f.write("-" * 80 + "\n")
            f.write(
                f"{'Side Wind':>10} | {'Long Wind':>10} | {'RMS Error':>12} | "
                f"{'MaxAE':>12} | {'Success':>8}\n"
            )
            f.write("-" * 80 + "\n")

            for _, row in vel_data.iterrows():
                f.write(
                    f"{row['wind_side']:>10.1f} | {row['wind_longitudinal']:>10.1f} | "
                    f"{row['rms_error']:>12.4f} | {row['max_error']:>12.4f} | "
                    f"{row['success']:>8.0f}\n"
                )
            f.write("\n")

    print(f"[Plots] Saved summary text to {txt_path}")

    return str(csv_path)


def generate_all_plots(
    results_df: DataFrame | list[dict],
    output_dir: str | Path = "artifacts/simulations/wind_performance",
) -> None:
    """
    Generate all plots and tables from wind performance test results.

    This is the main entry point for visualization. It creates:
    - Scatter plot: side wind vs longitudinal wind with MaxAE as color
    - Combined summary table (CSV and text)

    Args:
        results_df: DataFrame or list of dicts with test results
        output_dir: Directory to save all plots (default: artifacts/simulations/wind_performance)
    """
    # Convert to DataFrame if needed
    if isinstance(results_df, list):
        results_df = pd.DataFrame(results_df)

    print(f"\n[Plots] Generating visualizations for {len(results_df)} test results...")

    # Generate plots
    plot_maxae_scatter(results_df, output_dir)
    create_combined_summary_table(results_df, output_dir)

    print(f"[Plots] All visualizations saved to {output_dir}\n")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate wind performance plots from CSV results")
    parser.add_argument(
        "csv_file",
        type=str,
        help="Path to CSV file with test results",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="artifacts/simulations/wind_performance",
        help="Output directory for plots",
    )

    args = parser.parse_args()

    # Load results from CSV
    df = pd.read_csv(args.csv_file)

    # Generate all plots
    generate_all_plots(df, args.output_dir)
