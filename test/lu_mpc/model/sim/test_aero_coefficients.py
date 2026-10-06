import numpy as np
import pytest

from tooling.paper.methods.lu.aero_coefficients import (
    get_jax_coefficients,
    get_numpy_coefficients,
)
from tooling.paper.methods.lu.gd_model_casadi import SymbolicGdModel


def test_numpy_and_jax_coefficients_match_values():
    np_coeffs = get_numpy_coefficients()
    jax_coeffs = get_jax_coefficients()

    for name in ("kCL", "kCD", "kCY", "kCLL", "kCm", "kCn"):
        np_array = getattr(np_coeffs, name)
        jax_array = np.asarray(getattr(jax_coeffs, name))
        np.testing.assert_allclose(np_array, jax_array, atol=1e-7)


@pytest.mark.parametrize(
    "velocity",
    [
        np.array([5.0, 0.5, 1.0]),
        np.array([8.0, -0.2, -3.0]),
    ],
)
def test_symbolic_model_produces_body_moments(velocity: np.ndarray):
    model = SymbolicGdModel.create()
    _, moments, _, _ = model.evaluate(velocity)
    assert moments.shape == (3,)
    assert not np.allclose(moments[[0, 2]], 0.0)
