"""
Derivative computation for aerodynamics MLP.

Provides JAX-based automatic differentiation for computing ∂cx/∂α and ∂cz/∂α.
"""

from pathlib import Path
from typing import Literal

import jax
import jax.numpy as jnp

from aerodynamics.inference.predictor import (
    AerodynamicsMLPPredictor,
    load_aerodynamics_mlp,
)
from aerodynamics.models.aerodynamics_mlp import AerodynamicsMLP


class AerodynamicsMLPDerivatives:
    """
    Compute ∂cx/∂α and ∂cz/∂α using JAX autodiff.

    The MLP model takes (cos_alpha, sin_alpha) as input, so the derivative
    requires applying the chain rule. JAX handles this automatically when
    we differentiate through the predict function.
    """

    def __init__(self, model: AerodynamicsMLP, params: jax.Array, aero_type: str):
        """
        Initialize derivatives instance with model and params.

        Args:
            model: MLP4Layer model
            params: Model parameters
            aero_type: Aerodynamic type identifier
        """
        self._model = model
        self._params = params
        self._aero_type = aero_type

    def _alpha_to_features(self, alpha: jnp.ndarray) -> jnp.ndarray:
        """Convert alpha to (cos_alpha, sin_alpha) features."""
        if alpha.ndim == 0:
            # Scalar input -> batch of 1
            return jnp.array([[jnp.cos(alpha), jnp.sin(alpha)]])
        else:
            # Array input -> batch
            return jnp.stack([jnp.cos(alpha), jnp.sin(alpha)], axis=-1)

    def _predict_from_features(self, features: jnp.ndarray) -> jnp.ndarray:
        """Direct model application."""
        return self._model.model.apply(self._params, features)

    def _predict_alpha(self, alpha: jnp.ndarray) -> tuple[jnp.ndarray, jnp.ndarray]:
        """Predict (cx, cz) from alpha."""
        features = self._alpha_to_features(alpha)
        predictions = self._predict_from_features(features)

        if alpha.ndim == 0:
            # Scalar: squeeze batch dimension
            return predictions[0, 0], predictions[0, 1]
        else:
            # Array: return full arrays
            return predictions[:, 0], predictions[:, 1]

    def _predict_scalar_cx(self, alpha: jnp.ndarray) -> jnp.ndarray:
        """Predict cx from scalar alpha (for gradient computation)."""
        cx, _cz = self._predict_alpha(alpha)
        # Ensure scalar output - use explicit indexing if needed
        return cx[0] if cx.ndim > 0 else cx

    def _predict_scalar_cz(self, alpha: jnp.ndarray) -> jnp.ndarray:
        """Predict cz from scalar alpha (for gradient computation)."""
        _cx, cz = self._predict_alpha(alpha)
        # Ensure scalar output - use explicit indexing if needed
        return cz[0] if cz.ndim > 0 else cz

    def _predict_scalar_both(self, alpha: jnp.ndarray) -> jnp.ndarray:
        """Predict (cx, cz) from scalar alpha (for jacobian computation)."""
        cx, cz = self._predict_alpha(alpha)
        cx_scalar = cx[0] if cx.ndim > 0 else cx
        cz_scalar = cz[0] if cz.ndim > 0 else cz
        return jnp.array([cx_scalar, cz_scalar])

    @classmethod
    def load_from_checkpoint(
        cls,
        checkpoint_path: str | Path,
        aero_type: str,
        hidden_dim: int = 256,
    ) -> "AerodynamicsMLPDerivatives":
        """
        Load predictor and create derivatives instance.

        Args:
            checkpoint_path: Path to safetensors checkpoint
            aero_type: Aerodynamic type identifier
            hidden_dim: Hidden layer dimension

        Returns:
            AerodynamicsMLPDerivatives instance
        """
        predictor = AerodynamicsMLPPredictor.load_from_checkpoint(
            checkpoint_path, aero_type, hidden_dim
        )
        return cls(model=predictor._model, params=predictor._model.params, aero_type=aero_type)

    @classmethod
    def load_by_type(
        cls,
        aero_type: Literal["lyu", "bspline", "phi", "advanced"],
    ) -> "AerodynamicsMLPDerivatives":
        """
        Load derivatives by aerodynamic type (auto-detect checkpoint).

        Args:
            aero_type: Aerodynamic type identifier

        Returns:
            AerodynamicsMLPDerivatives instance
        """
        predictor = load_aerodynamics_mlp(aero_type)
        return cls(model=predictor._model, params=predictor._model.params, aero_type=aero_type)

    @property
    def predictor(self) -> AerodynamicsMLPPredictor:
        """Get a predictor instance for inference."""
        return AerodynamicsMLPPredictor(
            model=self._model,
            aero_type=self._aero_type,
        )

    def _jacobian_scalar(self, alpha: jnp.ndarray) -> jnp.ndarray:
        """Compute jacobian for scalar alpha (internal, for vmap)."""
        jac_fn = jax.jacfwd(self._predict_scalar_both)
        return jac_fn(alpha)

    def compute_dcx_dalpha(self, alpha: float | jnp.ndarray) -> jnp.ndarray:
        """
        Compute ∂cx/∂α using JAX autodiff.

        The chain rule is automatically applied through the cos/sin transform:
            ∂cx/∂α = ∂cx/∂cos_α · (-sin_α) + ∂cx/∂sin_α · cos_α

        Args:
            alpha: Angle of attack in radians (scalar or array)

        Returns:
            ∂cx/∂α: Derivative of drag coefficient w.r.t alpha
        """
        if isinstance(alpha, float | int):
            alpha = jnp.array(alpha)

        if alpha.ndim == 0:
            jacobian = self._jacobian_scalar(alpha)
            return jacobian[0].flatten()[0]
        else:
            # Use vmap for vectorized computation
            vmap_jacobian = jax.vmap(self._jacobian_scalar)
            jacobians = vmap_jacobian(alpha)
            return jacobians[:, 0].flatten()

    def compute_dcz_dalpha(self, alpha: float | jnp.ndarray) -> jnp.ndarray:
        """
        Compute ∂cz/∂α using JAX autodiff.

        Args:
            alpha: Angle of attack in radians (scalar or array)

        Returns:
            ∂cz/∂α: Derivative of lift coefficient w.r.t alpha
        """
        if isinstance(alpha, float | int):
            alpha = jnp.array(alpha)

        if alpha.ndim == 0:
            jacobian = self._jacobian_scalar(alpha)
            return jacobian[1].flatten()[0]
        else:
            # Use vmap for vectorized computation
            vmap_jacobian = jax.vmap(self._jacobian_scalar)
            jacobians = vmap_jacobian(alpha)
            return jacobians[:, 1].flatten()

    def compute_jacobian(
        self, alpha: float | jnp.ndarray
    ) -> tuple[jnp.ndarray, jnp.ndarray]:
        """
        Compute both (∂cx/∂α, ∂cz/∂α).

        Args:
            alpha: Angle of attack in radians (scalar or array)

        Returns:
            (∂cx/∂α, ∂cz/∂α): Tuple of derivatives
        """
        if isinstance(alpha, float | int):
            alpha = jnp.array(alpha)

        if alpha.ndim == 0:
            jacobian = self._jacobian_scalar(alpha)
            return jacobian[0].flatten()[0], jacobian[1].flatten()[0]
        else:
            # Use vmap for vectorized computation
            vmap_jacobian = jax.vmap(self._jacobian_scalar)
            jacobians = vmap_jacobian(alpha)
            return jacobians[:, 0].flatten(), jacobians[:, 1].flatten()

    @property
    def aero_type(self) -> str:
        return self._aero_type

    def __repr__(self) -> str:
        return f"AerodynamicsMLPDerivatives(aero_type='{self.aero_type}')"


def load_aero_derivatives(
    aero_type: Literal["lyu", "bspline", "phi", "advanced"],
) -> AerodynamicsMLPDerivatives:
    """
    Load derivatives instance for aerodynamic type.

    Convenience function for loading derivatives.

    Args:
        aero_type: Aerodynamic type identifier

    Returns:
        AerodynamicsMLPDerivatives instance
    """
    return AerodynamicsMLPDerivatives.load_by_type(aero_type)
