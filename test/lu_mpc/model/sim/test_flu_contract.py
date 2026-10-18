"""Native body-axis agreement and stamped paper-cache boundaries."""

import jax
import jax.numpy as jnp
import numpy as np
import pytest

from tooling.frame_contract import FRAME_CONVENTION
from tooling.paper.figure.data_utils import load_aero_data, save_aero_data
from tooling.paper.figure.mesh_plots import PANELS, load_mesh_data, plot_aerodynamic_mesh_jax
from tooling.paper.methods.lu import gd_model_jax
from tooling.paper.methods.lu.aero_coefficients import get_numpy_coefficients
from tooling.paper.methods.lu.gd_model_casadi import SymbolicGdModel
from tooling.paper.methods.lu.gd_model_jax import aerodynamics_B_vb, generate_mesh_data_jax


def test_symbolic_and_jax_forces_moments_and_angles(monkeypatch):
    values = [-12.0, -0.49, -1e-5, 0.0, 1e-5, 0.49, 12.0]
    velocity = np.array(np.meshgrid(values, values, values)).reshape(3, -1).T
    # Compare algebra at equal coefficient/compute precision. Production JAX
    # defaults to float32; its high-order fits have cancellation near zero.
    with jax.experimental.enable_x64(), jax.disable_jit(), monkeypatch.context() as patch:
        for name, coefficients in vars(get_numpy_coefficients()).items():
            patch.setattr(gd_model_jax, name, jnp.asarray(coefficients))
        force, moment, alpha, beta = map(np.asarray, aerodynamics_B_vb(jnp.asarray(velocity)))
    symbolic = SymbolicGdModel()
    rows = [symbolic.evaluate(v) for v in velocity]
    sf, sm, sa, sb = (np.stack([row[i] for row in rows]) for i in range(4))
    np.testing.assert_allclose(force, sf, atol=1e-10, rtol=1e-10)
    np.testing.assert_allclose(moment, sm, atol=1e-10, rtol=1e-10)
    expected_alpha = np.arctan2(velocity[:, 0], velocity[:, 2])
    expected_beta = np.arctan2(velocity[:, 1], np.hypot(velocity[:, 0], velocity[:, 2]))
    for actual, expected in (
        (alpha, expected_alpha),
        (sa, expected_alpha),
        (beta, expected_beta),
        (sb, expected_beta),
    ):
        np.testing.assert_allclose(actual, expected, atol=3e-7, rtol=1e-6)


def test_torch_coefficients_and_mesh_contract(tmp_path):
    torch = pytest.importorskip("torch")
    from tooling.paper.methods.lu.aero_coefficients import get_torch_coefficients
    from tooling.paper.methods.lu.gd_model_jax import compute_aerodynamic_coeffs_jax
    from tooling.paper.methods.lu.gd_model_torch import (
        compute_aerodynamic_coeffs_torch,
        generate_mesh_data_torch,
    )

    alpha, beta = np.array([[-0.4, 0.2, 0.7]]), np.array([[0.2, -0.3, 0.1]])
    expected = compute_aerodynamic_coeffs_jax(jnp.asarray(alpha), jnp.asarray(beta))
    actual = compute_aerodynamic_coeffs_torch(
        torch.tensor(alpha, dtype=torch.float32),
        torch.tensor(beta, dtype=torch.float32),
        *vars(get_torch_coefficients()).values(),
    )
    for tensor, reference in zip(actual, expected, strict=True):
        np.testing.assert_allclose(tensor.numpy(), np.asarray(reference), atol=2e-5, rtol=2e-4)
    data = generate_mesh_data_torch("torch-tiny", cache_dir=str(tmp_path), mesh_size=5)
    assert data["frame_convention"] == FRAME_CONVENTION
    load_mesh_data(tmp_path / "torch-tiny.npy")


def test_stale_cache_rejected_by_data_and_mesh_loaders(tmp_path):
    path = tmp_path / "unstamped.npy"
    np.save(path, {"CL": np.ones((2, 2))})
    with pytest.raises(ValueError, match="body-FLU"):
        load_aero_data("unstamped", cache_dir=str(tmp_path))
    with pytest.raises(ValueError, match="body-FLU"):
        load_mesh_data(path)
    save_aero_data({"value": np.array([1, 2])}, "native", cache_dir=str(tmp_path))
    assert load_aero_data("native", cache_dir=str(tmp_path))["frame_convention"] == FRAME_CONVENTION


def test_fresh_mesh_renders_native_moment_labels(tmp_path):
    data = generate_mesh_data_jax("tiny", cache_dir=str(tmp_path), mesh_size=5)
    assert data["frame_convention"] == FRAME_CONVENTION
    loaded = load_mesh_data(tmp_path / "tiny.npy")
    np.testing.assert_array_equal(data["CN"], loaded["CN"])
    output = tmp_path / "mesh.png"
    plot_aerodynamic_mesh_jax(data=data, show_plot=False, save_path=output)
    assert output.read_bytes().startswith(b"\x89PNG")
    labels = {key: title for _, key, title in PANELS}
    assert labels["CLL"] == "Body-Z Moment Coefficient"
    assert labels["CN"] == "Body-X Moment Coefficient"
    data["CL"][0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        plot_aerodynamic_mesh_jax(data=data, show_plot=False, save_path=output)
