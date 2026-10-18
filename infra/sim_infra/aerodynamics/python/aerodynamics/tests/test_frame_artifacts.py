"""Frame-stamped files and one-time output-axis import, using synthetic data."""

import numpy as np
import pandas as pd
import pytest
from flax.core import freeze
from infra.sim_infra.aerodynamics.data.ma_params_writer import (
    MaParamsError,
    _load_fit_results,
    write_ma_yaml,
)
from safetensors import safe_open
from safetensors.numpy import save_file

from aerodynamics.data.import_flu import (
    import_thrust_normal_checkpoint,
    import_thrust_normal_csv,
)
from aerodynamics.frames import FRAME_CONVENTION
from aerodynamics.training.checkpoints import load_safetensors, save_safetensors


def test_csv_axis_import_preserves_source_and_rejects_repeat(tmp_path):
    source, output = tmp_path / "source.csv", tmp_path / "flu.csv"
    pd.DataFrame({"alpha(rad)": [0, 0.2], "cx": [1, 2], "cz": [3, 4]}).to_csv(source, index=False)
    original = source.read_bytes()
    import_thrust_normal_csv(source, output)
    result = pd.read_csv(output)
    np.testing.assert_array_equal(result[["cx", "cz"]], [[3, 1], [4, 2]])
    assert result["frame_convention"].eq(FRAME_CONVENTION).all()
    assert source.read_bytes() == original
    with pytest.raises(FileExistsError):
        import_thrust_normal_csv(source, output)
    with pytest.raises(ValueError, match="second axis import"):
        import_thrust_normal_csv(output, tmp_path / "twice.csv")


def test_checkpoint_import_swaps_outputs_not_inputs_or_hidden_parameters(tmp_path):
    source, output = tmp_path / "source.safetensors", tmp_path / "flu.safetensors"
    parameters = {
        "out.kernel": np.array([[1, 2], [3, 4]], dtype=np.float32),
        "out.bias": np.array([5, 6], dtype=np.float32),
        "hidden.bias": np.array([7, 8], dtype=np.float32),
    }
    save_file(parameters, str(source))
    original = source.read_bytes()
    with pytest.raises(ValueError, match="body-FLU"):
        load_safetensors(source)
    import_thrust_normal_checkpoint(source, output, output_layer="out")
    native = load_safetensors(output)
    features = np.array([[0.5, -0.25], [-0.7, 0.1]], dtype=np.float32)
    expected = (features @ parameters["out.kernel"] + parameters["out.bias"])[:, [1, 0]]
    np.testing.assert_array_equal(features @ native["out.kernel"] + native["out.bias"], expected)
    np.testing.assert_array_equal(native["hidden.bias"], parameters["hidden.bias"])
    assert source.read_bytes() == original
    with pytest.raises(ValueError, match="second axis import"):
        import_thrust_normal_checkpoint(output, tmp_path / "twice.safetensors", output_layer="out")
    with pytest.raises(ValueError, match="absent"):
        import_thrust_normal_checkpoint(
            source, tmp_path / "wrong.safetensors", output_layer="missing"
        )


def test_fit_loader_rejects_unstamped_npz_and_stages_native_yaml(tmp_path, monkeypatch):
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    fit = tmp_path / ".artifacts/aerodynamics/bspline_fit_results"
    fit.mkdir(parents=True)
    parameters = {
        "cx_coefs": np.arange(4),
        "cz_coefs": np.arange(4) + 10,
        "cx_knots": np.array([0] * 4 + [1] * 4),
        "cz_knots": np.array([0] * 4 + [1] * 4),
    }
    for axis in ("cx", "cz"):
        np.savez(fit / f"bspline_fit_{axis}.npz", **parameters)
    with pytest.raises(MaParamsError, match="body-FLU"):
        _load_fit_results(tmp_path)
    for axis in ("cx", "cz"):
        np.savez(fit / f"bspline_fit_{axis}.npz", frame_convention=FRAME_CONVENTION, **parameters)
    loaded = _load_fit_results(tmp_path)
    for key, expected in parameters.items():
        np.testing.assert_array_equal(loaded[key], expected)
    output = write_ma_yaml(tmp_path, loaded, 1.0)
    assert output == tmp_path / ".artifacts/aerodynamics/ma.yaml"
    assert "body_frame: FLU" in output.read_text()
    assert not (tmp_path / "infra").exists()


def test_checkpoint_writer_persists_contract_and_nested_frozen_parameters(tmp_path):
    path = tmp_path / "new.safetensors"
    params = freeze({"params": {"Dense_0": {"bias": np.array([1, 2], dtype=np.float32)}}})
    save_safetensors(params, path, metadata={"aero_type": "synthetic"})
    np.testing.assert_array_equal(load_safetensors(path)["params.Dense_0.bias"], [1, 2])
    with safe_open(str(path), framework="np") as checkpoint:
        assert checkpoint.metadata()["frame_convention"] == FRAME_CONVENTION
    with pytest.raises(ValueError, match="body-FLU"):
        save_safetensors(
            params, tmp_path / "bad.safetensors", metadata={"frame_convention": "other"}
        )
