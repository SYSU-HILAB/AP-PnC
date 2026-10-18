"""Dataset structure, numerical integrity and persistent FLU frame contract."""

import numpy as np
import pandas as pd
import pytest

from aerodynamics.data.dataset import AERO_TYPE_MAP, AeroMLDataset
from aerodynamics.frames import FRAME_CONVENTION


def samples(count=101):
    rng = np.random.default_rng(0)
    return pd.DataFrame(
        {
            "alpha(rad)": np.linspace(-np.pi, np.pi, count),
            "cx": rng.standard_normal(count) * 0.1,
            "cz": rng.standard_normal(count) * 0.5,
            "frame_convention": FRAME_CONVENTION,
        }
    )


def load_fixture(folder, count=101):
    samples(count).to_csv(folder / "lyu_across_AoAs.csv", index=False)
    return AeroMLDataset.from_csv_files(folder, aero_types=["lyu"])


def test_aero_type_map():
    assert AERO_TYPE_MAP == {
        "lyu_across_AoAs.csv": "lyu",
        "bspline_across_AoAs.csv": "bspline",
        "phi_across_AoAs.csv": "phi",
        "advanced_across_AoAs.csv": "advanced",
    }


def test_dataset_from_csv_files(tmp_path):
    for filename in AERO_TYPE_MAP:
        samples().to_csv(tmp_path / filename, index=False)
    dataset = AeroMLDataset.from_csv_files(tmp_path)
    assert len(dataset) == 101 * 4
    assert set(dataset.df.columns) == {
        "aero_type",
        "alpha",
        "cos_alpha",
        "sin_alpha",
        "cx",
        "cz",
        "weight_path",
        "frame_convention",
    }
    assert set(dataset.df["aero_type"].unique()) == {"lyu", "bspline", "phi", "advanced"}
    assert dataset.df["frame_convention"].eq(FRAME_CONVENTION).all()


def test_dataset_filter_by_type(tmp_path):
    dataset = load_fixture(tmp_path)
    filtered = dataset.filter_by_type("lyu")
    assert len(filtered) == 101
    assert filtered.df["aero_type"].unique().tolist() == ["lyu"]
    with pytest.raises(ValueError, match="not found"):
        dataset.filter_by_type("missing")


def test_dataset_split_train_val(tmp_path):
    dataset = load_fixture(tmp_path, 100)
    train, val = dataset.split_train_val(train_ratio=0.8, seed=42)
    assert len(train) == 80 and len(val) == 20
    assert train.df["frame_convention"].eq(FRAME_CONVENTION).all()
    assert val.df["frame_convention"].eq(FRAME_CONVENTION).all()


def test_dataset_parquet_roundtrip(tmp_path):
    dataset = load_fixture(tmp_path)
    path = tmp_path / "test_dataset.parquet"
    dataset.to_parquet(path)
    loaded = AeroMLDataset.from_parquet(path)
    pd.testing.assert_frame_equal(loaded.df, dataset.df)


def test_dataset_get_features_targets(tmp_path):
    dataset = load_fixture(tmp_path)
    features, targets = dataset.get_features(), dataset.get_targets()
    assert features.shape == targets.shape == (101, 2)
    assert features.dtype == targets.dtype == np.float32
    np.testing.assert_allclose(features[:, 0], np.cos(dataset.df["alpha"]), atol=1e-7)
    np.testing.assert_allclose(features[:, 1], np.sin(dataset.df["alpha"]), atol=1e-7)
    np.testing.assert_allclose(targets, dataset.df[["cx", "cz"]], atol=1e-7)


@pytest.mark.parametrize("stamp", [None, "unknown"])
@pytest.mark.parametrize("format", ["csv", "parquet"])
def test_unstamped_or_incompatible_frame_rejected(tmp_path, stamp, format):
    data = samples()
    if stamp is None:
        data = data.drop(columns="frame_convention")
    else:
        data["frame_convention"] = stamp
    if format == "csv":
        data.to_csv(tmp_path / "lyu_across_AoAs.csv", index=False)
        with pytest.raises(ValueError, match="body-FLU"):
            AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])
    else:
        path = tmp_path / "incompatible.parquet"
        data.rename(columns={"alpha(rad)": "alpha"}).to_parquet(path, index=False)
        with pytest.raises(ValueError, match="body-FLU"):
            AeroMLDataset.from_parquet(path)


@pytest.mark.parametrize("invalid", [np.nan, np.inf])
def test_invalid_force_samples_are_not_fabricated(tmp_path, invalid):
    data = samples()
    data.loc[3, "cx"] = invalid
    data.to_csv(tmp_path / "lyu_across_AoAs.csv", index=False)
    with pytest.raises(ValueError, match="finite samples"):
        AeroMLDataset.from_csv_files(tmp_path, aero_types=["lyu"])
