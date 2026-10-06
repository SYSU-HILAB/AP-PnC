import jax.numpy as jnp
import numpy as np

from tooling.paper.methods.lu.gd_model_jax import (
    compute_aerodynamic_coeffs_jax,
    compute_angles_batch,
    compute_angles_single,
    trans_stab_to_body_batch,
)


def test_compute_angles_batch_matches_single_evaluations():
    vectors = jnp.array(
        [
            [10.0, 0.0, 0.0],
            [5.0, 1.0, 2.0],
            [8.0, -3.0, 1.5],
        ],
        dtype=jnp.float32,
    )

    alpha_batch, beta_batch, norm_batch, squared_batch = compute_angles_batch(vectors)

    for idx, vector in enumerate(np.array(vectors)):
        alpha_single, beta_single, norm_single, squared_single = compute_angles_single(
            jnp.array(vector, dtype=jnp.float32)
        )

        assert np.isclose(np.asarray(alpha_batch[idx]), np.asarray(alpha_single), atol=1e-7)
        assert np.isclose(np.asarray(beta_batch[idx]), np.asarray(beta_single), atol=1e-7)
        assert np.isclose(np.asarray(norm_batch[idx]), np.asarray(norm_single), atol=1e-7)
        assert np.isclose(np.asarray(squared_batch[idx]), np.asarray(squared_single), atol=1e-7)


def test_trans_stab_to_body_batch_rotates_vectors_about_pitch_axis():
    alphas = jnp.array([0.0, jnp.pi / 4], dtype=jnp.float32)
    vectors = jnp.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=jnp.float32)

    transformed = trans_stab_to_body_batch(alphas, vectors)

    expected = np.array(
        [
            [1.0, 0.0, 0.0],
            [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4)],
        ]
    )
    assert np.allclose(np.asarray(transformed), expected, atol=1e-6)


def test_compute_aerodynamic_coeffs_jax_matches_scalar_for_meshgrid():
    alpha_scalar = jnp.array(0.0, dtype=jnp.float32)
    beta_scalar = jnp.array(0.0, dtype=jnp.float32)
    scalar_coeffs = compute_aerodynamic_coeffs_jax(alpha_scalar, beta_scalar)

    alpha_mesh = jnp.zeros((2, 3), dtype=jnp.float32)
    beta_mesh = jnp.zeros_like(alpha_mesh)
    mesh_coeffs = compute_aerodynamic_coeffs_jax(alpha_mesh, beta_mesh)

    for mesh, scalar in zip(mesh_coeffs, scalar_coeffs, strict=False):
        assert mesh.shape == alpha_mesh.shape
        assert np.all(np.isfinite(np.asarray(mesh)))
        assert np.allclose(
            np.asarray(mesh),
            np.asarray(scalar),
            atol=1e-6,
        )
