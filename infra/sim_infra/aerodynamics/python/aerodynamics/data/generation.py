"""
DEPRECATED: Direct data generation using pybind11 aerodynamics bindings.

This module is DEPRECATED due to a known bug in the pybind11 cleanup code
that causes segfault during object destruction. Use CSV files directly instead.

Use feature.aerodynamics.data.dataset.assemble_parquet_dataset() with
CSV files from <AP_PNC_DIR>/.artifacts/aerodynamics/ as the data source.
"""

from pathlib import Path
from typing import TYPE_CHECKING, Literal

import numpy as np

if TYPE_CHECKING:
    from aerodynamics.data.dataset import AeroMLDataset


def _get_aerodynamics_bindings():
    """
    Import the aerodynamics_bindings pybind11 module.

    The module should be installed via:
        uv pip install -e src/aerodynamics/python/
    """
    try:
        import aerodynamics_bindings

        return aerodynamics_bindings
    except ImportError as e:
        raise ImportError(
            f"Failed to import aerodynamics_bindings. "
            f"Make sure you've installed it with: uv pip install -e src/aerodynamics/python/ "
            f"Original error: {e}"
        ) from e


def create_aerodynamics_model(
    aero_type: Literal["lyu", "bspline", "phi", "advanced"],
):
    """
    Create an aerodynamics model using pybind11 bindings.

    Args:
        aero_type: Type of aerodynamics model

    Returns:
        PyAerodynamicsModel instance
    """
    bindings = _get_aerodynamics_bindings()

    factory_map = {
        "lyu": bindings.create_lyu_model,
        "bspline": bindings.create_bspline_model,
        "phi": bindings.create_phi_model,
        "advanced": bindings.create_advanced_lift_drag_model,
    }

    if aero_type not in factory_map:
        raise ValueError(
            f"Unknown aero_type '{aero_type}'. Must be one of: {list(factory_map.keys())}"
        )

    return factory_map[aero_type]()


def generate_aero_data(
    aero_type: Literal["lyu", "bspline", "phi", "advanced"],
    alpha_min: float = -np.pi,
    alpha_max: float = np.pi,
    num_points: int = 36001,
    speed: float = 12.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate aerodynamic data for a specific type.

    Args:
        aero_type: Type of aerodynamics model
        alpha_min: Minimum angle of attack in radians
        alpha_max: Maximum angle of attack in radians
        num_points: Number of data points
        speed: Airspeed magnitude in m/s

    Returns:
        (alpha, cx, cz) arrays
    """
    model = create_aerodynamics_model(aero_type)

    # Generate alpha array
    alpha = np.linspace(alpha_min, alpha_max, num_points)

    # Compute coefficients using the array method
    cx, cz = model.compute_coefficients_array(alpha, speed)

    return alpha, cx, cz


def generate_all_aero_data(
    alpha_min: float = -np.pi,
    alpha_max: float = np.pi,
    num_points: int = 36001,
    speed: float = 12.0,
) -> dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Generate aerodynamic data for all 4 types.

    Args:
        alpha_min: Minimum angle of attack in radians
        alpha_max: Maximum angle of attack in radians
        num_points: Number of data points
        speed: Airspeed magnitude in m/s

    Returns:
        Dictionary mapping aero_type to (alpha, cx, cz) tuples
    """
    aero_types = ["lyu", "bspline", "phi", "advanced"]
    results = {}

    for aero_type in aero_types:
        print(f"Generating data for {aero_type}...")
        alpha, cx, cz = generate_aero_data(aero_type, alpha_min, alpha_max, num_points, speed)
        results[aero_type] = (alpha, cx, cz)
        print(f"  Generated {len(alpha)} points")

    return results


def assemble_parquet_dataset_from_bindings(
    alpha_min: float = -np.pi,
    alpha_max: float = np.pi,
    num_points: int = 36001,
    speed: float = 12.0,
    output_path: str | Path = ".artifacts/aerodynamics/aero_ml_dataset.parquet",
) -> "AeroMLDataset":
    """
    Generate data directly from C++ bindings and assemble into parquet dataset.

    This is the main entry point for data generation using pybind11.

    Args:
        alpha_min: Minimum angle of attack in radians
        alpha_max: Maximum angle of attack in radians
        num_points: Number of data points
        speed: Airspeed magnitude in m/s
        output_path: Output parquet file path

    Returns:
        AeroMLDataset
    """
    from aerodynamics.data.dataset import AeroMLDataset

    aero_types = ["lyu", "bspline", "phi", "advanced"]
    dfs = []

    for aero_type in aero_types:
        print(f"Generating data for {aero_type}...")
        alpha, cx, cz = generate_aero_data(aero_type, alpha_min, alpha_max, num_points, speed)

        # Create DataFrame
        import pandas as pd

        df = pd.DataFrame(
            {
                "alpha": alpha,
                "cos_alpha": np.cos(alpha),
                "sin_alpha": np.sin(alpha),
                "cx": cx,
                "cz": cz,
                "aero_type": aero_type,
                "weight_path": None,
            }
        )
        dfs.append(df)
        print(f"  Generated {len(alpha)} points")

    # Combine all data
    combined_df = pd.concat(dfs, ignore_index=True)
    dataset = AeroMLDataset(df=combined_df)

    # Save to parquet
    dataset.to_parquet(output_path)

    print(f"\nDataset assembled: {dataset}")
    print(f"Saved to: {output_path}")

    return dataset


# Convenience function for backward compatibility
def assemble_parquet_dataset(**kwargs) -> "AeroMLDataset":
    """Alias for assemble_parquet_dataset_from_bindings."""
    return assemble_parquet_dataset_from_bindings(**kwargs)
