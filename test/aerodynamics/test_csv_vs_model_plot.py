#!/usr/bin/env python
"""Plot CSV data vs trained model predictions comparison.

Creates a 1x4 subplot comparing CSV cz vs trained model cz across [-180, 180] degrees
for all 4 aerodynamics types (lyu, bspline, phi, advanced).
"""

import os
from pathlib import Path

import jax
import matplotlib.pyplot as plt
import numpy as np

# Initialize JAX CPU backend
os.environ["JAX_PLATFORM_NAME"] = "cpu"
jax.config.update("jax_platform_name", "cpu")

from aerodynamics.data.dataset import AeroMLDataset  # noqa: E402 -- after JAX backend pin
from aerodynamics.inference.predictor import AerodynamicsMLPPredictor  # noqa: E402

# Aero type configuration
AERO_TYPES = ["lyu", "bspline", "phi", "advanced"]
AERO_COLORS = {
    "lyu": "#1f77b4",      # blue
    "bspline": "#ff7f0e",   # orange
    "phi": "#2ca02c",       # green
    "advanced": "#d62728",  # red
}


def plot_csv_vs_model_comparison(git_root: Path | None = None, show_plot: bool = False) -> None:
    """Plot CSV data vs trained model predictions for all aero types.

    Creates a 1x4 subplot comparing CSV cz vs trained model cz across [-180, 180] degrees.

    Args:
        git_root: Repository root path. If None, auto-detected.
        show_plot: If True, display plot interactively. If False, save to file.
    """
    if git_root is None:
        git_root = Path(__file__).parent.parent.parent

    # Load dataset
    dataset_path = git_root / "artifacts/simulations/aero_ml_dataset.parquet"
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}. Run 'uv run ap-pnc aerodyn assemble' first.")

    dataset = AeroMLDataset.from_parquet(dataset_path)

    # Load models and create predictors
    predictors = {}
    for aero_type in AERO_TYPES:
        weight_path = git_root / f"artifacts/simulations/aero_ml_weights/{aero_type}_mlp_weights.safetensors"
        if not weight_path.exists():
            print(f"Warning: Model weights not found for {aero_type}: {weight_path}")
            print(f"  Run 'uv run ap-pnc aerodyn train-type {aero_type}' first.")
            continue

        # Load predictor from checkpoint
        predictor = AerodynamicsMLPPredictor.load_from_checkpoint(weight_path, aero_type)
        predictors[aero_type] = predictor

    if not predictors:
        raise RuntimeError("No model weights found. Run training first.")

    # Create 1x4 subplot
    fig, axes = plt.subplots(1, 4, figsize=(20, 4))
    fig.suptitle("CSV Data vs Trained Model: $C_z$ vs $\\alpha$", fontsize=16)

    # Generate alpha range in degrees [-180, 180]
    alpha_deg = np.linspace(-180, 180, 3601)
    alpha_rad = np.deg2rad(alpha_deg)

    # Plot each aero type
    for idx, aero_type in enumerate(AERO_TYPES):
        ax = axes[idx]

        if aero_type not in predictors:
            ax.text(0.5, 0.5, f"{aero_type}\nNo model found",
                   ha="center", va="center", transform=ax.transAxes)
            ax.set_title(f"{aero_type.upper()} (No Model)")
            continue

        # Get CSV data for this aero type
        aero_data = dataset.filter_by_type(aero_type)
        csv_alpha_deg = np.rad2deg(aero_data.df["alpha"].values)
        csv_cz = aero_data.df["cz"].values

        # Get model predictions
        predictor = predictors[aero_type]
        model_cx, model_cz = predictor.predict(alpha_rad)
        model_cz = np.array(model_cz)  # Convert JAX array to numpy

        # Plot CSV data
        ax.plot(csv_alpha_deg, csv_cz, label="CSV Data", linewidth=2,
                color=AERO_COLORS[aero_type], alpha=0.7)

        # Plot model predictions
        ax.plot(alpha_deg, model_cz, label="Trained Model", linewidth=2,
                color="black", linestyle="--", alpha=0.8)

        # Configure subplot
        ax.set_xlabel(r"$\alpha$ (degrees)")
        ax.set_ylabel("$C_z$")
        ax.set_title(f"{aero_type.upper()}")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        ax.set_xlim([-180, 180])

    plt.tight_layout()

    # Save or show
    if show_plot:
        plt.show()
    else:
        output_path = git_root / "artifacts/simulations/csv_vs_model_comparison.png"
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot to: {output_path}")
        plt.close()


if __name__ == "__main__":
    plot_csv_vs_model_comparison()
