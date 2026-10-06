"""
Data utilities for communication between data generation and plotting functions.

The plotting stack only understands three payload families and each of them must
contain the following fields (all arrays are 1-D numpy arrays unless noted).

* `coordinated_flight` (produced by `save_coordinated_data`):
  - `alpha_deg`, `CL`, `CD`, `CM`
* `all_coefficients` (produced by `save_all_coeffs_data`):
  - `alpha_deg`, `CL`, `CD`, `CY`, `Cl`/`CLL`, `Cm`, `Cn`
* `3d_mesh` (produced by `save_mesh_data`):
  - `ALPHA_DEG`, `BETA_DEG`, `CL`, `CD`, `CY`, `CLL`/`Cl`, `CM`, `CN`

All payloads also include a `data_type` and free-form `description` so that downstream
plotting helpers can quickly validate the resource before trying to visualize it.
"""

import json
import os
from pathlib import Path
from typing import Any, TypedDict

import numpy as np

from tooling.env import artifact_path


class CoordinatedFlightData(TypedDict):
    """Standardized schema for beta=0 aerodynamic tables."""

    alpha_deg: np.ndarray
    CL: np.ndarray
    CD: np.ndarray
    CM: np.ndarray
    data_type: str
    description: str


class AllCoefficientsData(TypedDict, total=False):
    """Expanded schema that includes side-force and moment coefficients."""

    alpha_deg: np.ndarray
    CL: np.ndarray
    CD: np.ndarray
    CY: np.ndarray
    Cl: np.ndarray
    CLL: np.ndarray
    Cm: np.ndarray
    Cn: np.ndarray
    data_type: str
    description: str


class MeshData(TypedDict):
    """Schema for 2-D alpha/beta aerodynamic grids."""

    ALPHA_DEG: np.ndarray
    BETA_DEG: np.ndarray
    CL: np.ndarray
    CD: np.ndarray
    CY: np.ndarray
    CLL: np.ndarray
    CM: np.ndarray
    CN: np.ndarray
    backend: str
    data_type: str
    description: str


# Default cache directory
# Project-root-relative configuration; resolve through artifact_path before every access.
CACHE_DIR = Path(".artifacts/paper/cache")


def get_data_path(filename: str, cache_dir: str | None = None) -> str:
    """
    Get full path for data file in cache directory.

    Args:
        filename (str): Name of the data file
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        str: Full path to the data file
    """
    if cache_dir is None:
        cache_dir = CACHE_DIR

    cache_path = artifact_path(cache_dir)
    file_path = artifact_path(cache_path / filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    return str(file_path)


def save_aero_data(
    data: dict[str, Any], filename: str, cache_dir: str | None = None
) -> str:
    """
    Save aerodynamic data to .npy file.

    Args:
        data (dict): Dictionary containing aerodynamic data
        filename (str): Name of the data file (should end with .npy)
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        str: Full path to the saved data file
    """
    if not filename.endswith(".npy"):
        filename += ".npy"

    file_path = get_data_path(filename, cache_dir)

    # Ensure data contains only numpy-serializable objects
    clean_data = {}
    for key, value in data.items():
        if hasattr(value, "numpy"):  # Handle JAX arrays
            clean_data[key] = np.array(value)
        elif isinstance(value, np.ndarray):
            clean_data[key] = value
        elif isinstance(value, int | float | str | bool):
            clean_data[key] = value
        else:
            # Convert to numpy array if possible
            try:
                clean_data[key] = np.array(value)
            except Exception:
                # Skip non-serializable objects
                print(f"Warning: Skipping non-serializable key: {key}")

    np.save(file_path, clean_data)
    print(f"Data saved to: {file_path}")
    return file_path


def load_aero_data(filename: str, cache_dir: str | None = None) -> dict[str, Any]:
    """
    Load aerodynamic data from .npy file.

    Args:
        filename (str): Name of the data file
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        dict: Dictionary containing the aerodynamic data
    """
    if not filename.endswith(".npy"):
        filename += ".npy"

    file_path = get_data_path(filename, cache_dir)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Data file not found: {file_path}")

    data = np.load(file_path, allow_pickle=True).item()
    print(f"Data loaded from: {file_path}")
    return data


def save_metadata(
    metadata: dict[str, Any], filename: str, cache_dir: str | None = None
) -> str:
    """
    Save metadata to JSON file.

    Args:
        metadata (dict): Dictionary containing metadata
        filename (str): Name of the metadata file (should end with .json)
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        str: Full path to the saved metadata file
    """
    if not filename.endswith(".json"):
        filename += ".json"

    file_path = get_data_path(filename, cache_dir)

    with open(file_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"Metadata saved to: {file_path}")
    return file_path


def load_metadata(filename: str, cache_dir: str | None = None) -> dict[str, Any]:
    """
    Load metadata from JSON file.

    Args:
        filename (str): Name of the metadata file
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        dict: Dictionary containing the metadata
    """
    if not filename.endswith(".json"):
        filename += ".json"

    file_path = get_data_path(filename, cache_dir)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Metadata file not found: {file_path}")

    with open(file_path) as f:
        metadata = json.load(f)

    print(f"Metadata loaded from: {file_path}")
    return metadata


def clear_cache(cache_dir: str | None = None) -> None:
    """
    Clear all cached data files.

    Args:
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache
    """
    if cache_dir is None:
        cache_dir = CACHE_DIR

    cache_path = artifact_path(cache_dir)

    if cache_path.exists():
        for file_path in cache_path.iterdir():
            if file_path.is_file():
                file_path.unlink()
                print(f"Deleted: {file_path}")
        print(f"Cache cleared: {cache_path}")
    else:
        print(f"Cache directory does not exist: {cache_path}")


def list_cache_files(cache_dir: str | None = None) -> list:
    """
    List all files in the cache directory.

    Args:
        cache_dir (str, optional): Custom cache directory. Defaults to .artifacts/paper/cache

    Returns:
        list: List of file paths in the cache directory
    """
    if cache_dir is None:
        cache_dir = CACHE_DIR

    cache_path = artifact_path(cache_dir)

    if not cache_path.exists():
        return []

    return [str(p) for p in cache_path.iterdir() if p.is_file()]


# Convenience functions for common data types
def save_coordinated_data(
    alpha_deg, CL, CD, CM, filename="coordinated_aero_data", cache_dir=None
):
    """Save coordinated flight data (beta=0)"""
    data = {
        "alpha_deg": alpha_deg,
        "CL": CL,
        "CD": CD,
        "CM": CM,
        "data_type": "coordinated_flight",
        "description": "Aerodynamic coefficients under coordinated flight (beta = 0)",
    }
    return save_aero_data(data, filename, cache_dir)


def save_all_coeffs_data(
    alpha_deg, CL, CD, CY, Cl, Cm, Cn, filename="all_aero_coeffs_data", cache_dir=None
):
    """Save all aerodynamic coefficients data"""
    data = {
        "alpha_deg": alpha_deg,
        "CL": CL,
        "CD": CD,
        "CY": CY,
        "Cl": Cl,
        "Cm": Cm,
        "Cn": Cn,
        "data_type": "all_coefficients",
        "description": "All aerodynamic coefficients vs angle of attack",
    }
    return save_aero_data(data, filename, cache_dir)


def save_mesh_data(
    ALPHA_DEG,
    BETA_DEG,
    CL,
    CD,
    CY,
    CLL,
    CM,
    CN,
    filename="mesh_aero_data",
    cache_dir=None,
    backend="numpy",
):
    """Save 3D mesh data"""
    data = {
        "ALPHA_DEG": ALPHA_DEG,
        "BETA_DEG": BETA_DEG,
        "CL": CL,
        "CD": CD,
        "CY": CY,
        "CLL": CLL,
        "CM": CM,
        "CN": CN,
        "data_type": "3d_mesh",
        "backend": backend,
        "description": f"3D aerodynamic coefficient mesh ({backend} backend)",
    }
    return save_aero_data(data, filename, cache_dir)
