"""
Aerodynamics MLP wrapper class.

Provides factory methods for creating MLPs for specific aerodynamic types.
"""

from typing import Literal

import jax
import jax.numpy as jnp

from aerodynamics.models.mlp import MLP4Layer


class AerodynamicsMLP:
    """
    Wrapper class for aerodynamics MLP models.

    Provides factory pattern for creating models per aerodynamic type.
    """

    def __init__(self, model: MLP4Layer, params: jax.Array | None = None):
        """
        Initialize aerodynamics MLP.

        Args:
            model: Flax MLP module
            params: Model parameters (None for uninitialized model)
        """
        self._model = model
        self._params = params

    @classmethod
    def create_for_aero_type(
        cls,
        aero_type: Literal["lyu", "bspline", "phi", "advanced"],
        hidden_dim: int = 64,
        key: jax.Array | None = None,
    ) -> "AerodynamicsMLP":
        """
        Create MLP for specific aerodynamic type.

        Args:
            aero_type: Aerodynamic type identifier
            hidden_dim: Hidden layer dimension
            key: JAX PRNGKey for parameter initialization (None for uninitialized)

        Returns:
            AerodynamicsMLP instance
        """
        model = MLP4Layer(hidden_dim=hidden_dim)
        params = None

        if key is not None:
            # Initialize with dummy input
            dummy_input = jnp.zeros((1, 2))
            params = model.init(key, dummy_input)

        return cls(model=model, params=params)

    def init(self, key: jax.Array) -> "AerodynamicsMLP":
        """Initialize model parameters."""
        dummy_input = jnp.zeros((1, 2))
        self._params = self._model.init(key, dummy_input)
        return self

    @property
    def model(self) -> MLP4Layer:
        return self._model

    @property
    def params(self) -> jax.Array | None:
        return self._params

    @params.setter
    def params(self, value: jax.Array) -> None:
        self._params = value

    def __repr__(self) -> str:
        return f"AerodynamicsMLP(model=MLP4Layer, params_initialized={self._params is not None})"
