"""
Test reading and plotting the aerodynamic ML dataset parquet file.
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pytest

from tooling.env import project_path


def _dataset_path() -> str:
    return str(project_path(".artifacts/aerodynamics/aero_ml_dataset.parquet"))


def _require_dataset() -> str:
    """Skip cleanly when the generated dataset is absent."""
    from pathlib import Path

    path = _dataset_path()
    if not Path(path).exists():
        pytest.skip(f"aero ML dataset not generated: {path}")
    return path


def test_read_aero_ml_dataset() -> None:
    """Read and display the aero_ml_dataset.parquet file."""
    df = pd.read_parquet(_require_dataset())
    print(df.head())
    print(f"\nDataset shape: {df.shape}")
    print(f"\nColumns: {df.columns.tolist()}")


def test_plot_aero_ml_dataset() -> None:
    """Plot cx, cz vs alpha in degrees (-180 to 180)."""
    df = pd.read_parquet(_require_dataset())

    # Convert alpha from radians to degrees
    df['alpha_deg'] = np.degrees(df['alpha'])

    # Group by aero_type
    for aero_type in df['aero_type'].unique():
        data = df[df['aero_type'] == aero_type].sort_values('alpha_deg')

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle(f'Aerodynamic Coefficients - {aero_type.upper()} Model', fontsize=14)

        # Cx subplot
        ax1.plot(data['alpha_deg'], data['cx'], 'b-', linewidth=2)
        ax1.set_xlabel('Alpha [degrees]', fontsize=12)
        ax1.set_ylabel('Cx (Drag Coefficient)', fontsize=12)
        ax1.set_title('Drag Coefficient Cx', fontsize=13)
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.axvline(x=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_xlim([-180, 180])

        # Cz subplot
        ax2.plot(data['alpha_deg'], data['cz'], 'r-', linewidth=2)
        ax2.set_xlabel('Alpha [degrees]', fontsize=12)
        ax2.set_ylabel('Cz (Lift Coefficient)', fontsize=12)
        ax2.set_title('Lift Coefficient Cz', fontsize=13)
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.axvline(x=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlim([-180, 180])

        plt.tight_layout()
        plt.savefig(f'test_aerodynamics_{aero_type}.png', dpi=150)
        print(f"Saved: test_aerodynamics_{aero_type}.png")

    plt.show()


if __name__ == '__main__':
    test_plot_aero_ml_dataset()
