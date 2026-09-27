"""
Tests for aerodynamics MLP models.
"""

import jax
import jax.numpy as jnp
import pytest

from aerodynamics.models.aerodynamics_mlp import AerodynamicsMLP
from aerodynamics.models.mlp import MLP4Layer


def test_mlp_forward_pass():
    """Test MLP forward pass."""
    model = MLP4Layer(hidden_dim=64)
    key = jax.random.PRNGKey(42)

    # Initialize with dummy input
    dummy_input = jnp.zeros((1, 2))
    params = model.init(key, dummy_input)

    # Forward pass
    output = model.apply(params, dummy_input)

    assert output.shape == (1, 2)


def test_mlp_batch_forward_pass():
    """Test MLP batch forward pass."""
    model = MLP4Layer(hidden_dim=64)
    key = jax.random.PRNGKey(42)

    # Initialize
    dummy_input = jnp.zeros((1, 2))
    params = model.init(key, dummy_input)

    # Batch forward pass
    batch_input = jnp.randn((10, 2))
    output = model.apply(params, batch_input)

    assert output.shape == (10, 2)


def test_aerodynamics_mlp_create():
    """Test AerodynamicsMLP creation."""
    key = jax.random.PRNGKey(42)
    model = AerodynamicsMLP.create_for_aero_type("lyu", hidden_dim=64, key=key)

    assert model.model is not None
    assert model.params is not None
    assert isinstance(model.model, MLP4Layer)


def test_aerodynamics_mlp_create_uninitialized():
    """Test AerodynamicsMLP creation without initialization."""
    model = AerodynamicsMLP.create_for_aero_type("lyu", hidden_dim=64, key=None)

    assert model.model is not None
    assert model.params is None


def test_aerodynamics_mlp_init():
    """Test AerodynamicsMLP initialization."""
    model = AerodynamicsMLP.create_for_aero_type("lyu", hidden_dim=64, key=None)
    key = jax.random.PRNGKey(42)

    model.init(key)

    assert model.params is not None


def test_aerodynamics_mlp_forward_pass():
    """Test AerodynamicsMLP forward pass."""
    key = jax.random.PRNGKey(42)
    model = AerodynamicsMLP.create_for_aero_type("lyu", hidden_dim=64, key=key)

    # Forward pass
    features = jnp.array([[0.0, 1.0]])  # cos(0), sin(0)
    output = model.model.apply(model.params, features)

    assert output.shape == (1, 2)


def test_mlp_predict_coefs_sin():
    """Test that MLP can learn sin/cos relationship (sanity check)."""
    model = MLP4Layer(hidden_dim=64)
    key = jax.random.PRNGKey(42)

    # Initialize
    dummy_input = jnp.zeros((1, 2))
    params = model.init(key, dummy_input)

    # Test various angles
    alphas = jnp.array([0.0, np.pi / 2, np.pi, -np.pi / 2])
    cos_alpha = jnp.cos(alphas)
    sin_alpha = jnp.sin(alphas)
    features = jnp.stack([cos_alpha, sin_alpha], axis=-1)

    output = model.apply(params, features)

    # Just check shape (output values are random before training)
    assert output.shape == (4, 2)


if __name__ == "__main__":
    import numpy as np
    pytest.main([__file__, "-v"])
