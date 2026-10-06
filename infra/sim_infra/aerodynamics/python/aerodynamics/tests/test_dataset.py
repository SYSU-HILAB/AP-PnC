"""
Tests for aerodynamics dataset module.
"""

import numpy as np
import pandas as pd
import pytest

from aerodynamics.data.dataset import AERO_TYPE_MAP, AeroMLDataset

rng = np.random.default_rng(0)


def test_aero_type_map():
    """Test aerodynamic type mapping."""
    assert "lyu_across_AoAs.csv" in AERO_TYPE_MAP
    assert AERO_TYPE_MAP["lyu_across_AoAs.csv"] == "lyu"
    assert AERO_TYPE_MAP["bspline_across_AoAs.csv"] == "bspline"
    assert AERO_TYPE_MAP["phi_across_AoAs.csv"] == "phi"
    assert AERO_TYPE_MAP["advanced_across_AoAs.csv"] == "advanced"


def test_dataset_from_csv_files(tmp_path):
    """Test loading dataset from CSV files."""
    # Create temporary CSV files
    for csv_file, _aero_type in AERO_TYPE_MAP.items():
        csv_path = tmp_path / csv_file
        alpha = np.linspace(-np.pi, np.pi, 101)
        cx = rng.standard_normal(101) * 0.1
        cz = rng.standard_normal(101) * 0.5

        df = pd.DataFrame({"alpha(rad)": alpha, "cx": cx, "cz": cz})
        df.to_csv(csv_path, index=False)

    # Load dataset
    dataset = AeroMLDataset.from_csv_files(tmp_path)

    # Verify structure
    assert len(dataset) == 101 * 4  # 4 aero types
    assert "aero_type" in dataset.df.columns
    assert "alpha" in dataset.df.columns
    assert "cos_alpha" in dataset.df.columns
    assert "sin_alpha" in dataset.df.columns
    assert "cx" in dataset.df.columns
    assert "cz" in dataset.df.columns
    assert "weight_path" in dataset.df.columns

    # Verify aero types
    aero_types = dataset.df["aero_type"].unique()
    assert set(aero_types) == {"lyu", "bspline", "phi", "advanced"}


def test_dataset_filter_by_type(tmp_path):
    """Test filtering dataset by aerodynamic type."""
    # Create temporary CSV file
    csv_path = tmp_path / "lyu_across_AoAs.csv"
    alpha = np.linspace(-np.pi, np.pi, 101)
    cx = rng.standard_normal(101) * 0.1
    cz = rng.standard_normal(101) * 0.5

    df = pd.DataFrame({"alpha(rad)": alpha, "cx": cx, "cz": cz})
    df.to_csv(csv_path, index=False)

    # Load and filter
    dataset = AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])
    filtered = dataset.filter_by_type("lyu")

    assert len(filtered) == 101
    assert filtered.df["aero_type"].unique()[0] == "lyu"


def test_dataset_split_train_val(tmp_path):
    """Test train/validation split."""
    # Create temporary CSV file
    csv_path = tmp_path / "lyu_across_AoAs.csv"
    alpha = np.linspace(-np.pi, np.pi, 100)
    cx = rng.standard_normal(100) * 0.1
    cz = rng.standard_normal(100) * 0.5

    df = pd.DataFrame({"alpha(rad)": alpha, "cx": cx, "cz": cz})
    df.to_csv(csv_path, index=False)

    # Load and split
    dataset = AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])
    train, val = dataset.split_train_val(train_ratio=0.8, seed=42)

    assert len(train) == 80
    assert len(val) == 20


def test_dataset_parquet_roundtrip(tmp_path):
    """Test saving and loading from parquet."""
    # Create temporary CSV file
    csv_path = tmp_path / "lyu_across_AoAs.csv"
    alpha = np.linspace(-np.pi, np.pi, 101)
    cx = rng.standard_normal(101) * 0.1
    cz = rng.standard_normal(101) * 0.5

    df = pd.DataFrame({"alpha(rad)": alpha, "cx": cx, "cz": cz})
    df.to_csv(csv_path, index=False)

    # Load, save, and reload
    dataset = AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])

    parquet_path = tmp_path / "test_dataset.parquet"
    dataset.to_parquet(parquet_path)

    loaded = AeroMLDataset.from_parquet(parquet_path)

    assert len(loaded) == len(dataset)
    assert list(loaded.df.columns) == list(dataset.df.columns)


def test_dataset_get_features_targets(tmp_path):
    """Test getting features and targets."""
    # Create temporary CSV file
    csv_path = tmp_path / "lyu_across_AoAs.csv"
    alpha = np.linspace(-np.pi, np.pi, 101)
    cx = rng.standard_normal(101) * 0.1
    cz = rng.standard_normal(101) * 0.5

    df = pd.DataFrame({"alpha(rad)": alpha, "cx": cx, "cz": cz})
    df.to_csv(csv_path, index=False)

    # Load and get features/targets
    dataset = AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])

    features = dataset.get_features()
    targets = dataset.get_targets()

    assert features.shape == (101, 2)
    assert targets.shape == (101, 2)
    assert features.dtype == np.float32
    assert targets.dtype == np.float32


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
