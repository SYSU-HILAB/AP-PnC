"""
Inference API for trained aerodynamics MLP.
"""

from pathlib import Path
from typing import Literal

import jax
import jax.numpy as jnp

from aerodynamics.models.aerodynamics_mlp import AerodynamicsMLP
from aerodynamics.training.checkpoints import load_safetensors
from tooling.env import project_path


class AerodynamicsMLPPredictor:
    """
    Predictor for trained aerodynamics MLP.

    Provides inference API for (cx, cz) prediction from alpha.
    """

    def __init__(self, model: AerodynamicsMLP, aero_type: str):
        """
        Initialize predictor.

        Args:
            model: Trained AerodynamicsMLP model
            aero_type: Aerodynamic type identifier
        """
        self._model = model
        self._aero_type = aero_type

        # JIT-compiled predict function
        self._predict_fn = jax.jit(self._model.model.apply)

    @classmethod
    def load_from_checkpoint(
        cls,
        checkpoint_path: str | Path,
        aero_type: str,
        hidden_dim: int = 256,
    ) -> "AerodynamicsMLPPredictor":
        """
        Load trained model from safetensors checkpoint.

        Args:
            checkpoint_path: Path to safetensors checkpoint
            aero_type: Aerodynamic type identifier
            hidden_dim: Hidden layer dimension

        Returns:
            AerodynamicsMLPPredictor instance
        """
        checkpoint_path = project_path(checkpoint_path)
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

        # Create model
        model = AerodynamicsMLP.create_for_aero_type(aero_type=aero_type, hidden_dim=hidden_dim)

        # Load parameters
        params_dict = load_safetensors(checkpoint_path)

        # Reconstruct FrozenDict from flat dict
        params = _reconstruct_frozen_dict(params_dict)

        model.params = params

        return cls(model=model, aero_type=aero_type)

    def predict(self, alpha: float | jnp.ndarray) -> tuple[jnp.ndarray, jnp.ndarray]:
        """
        Predict aerodynamic coefficients from angle of attack.

        Args:
            alpha: Angle of attack in radians (scalar or array)

        Returns:
            (cx, cz): Force coefficients (same shape as alpha)
        """
        # Convert to input features
        cos_alpha = jnp.cos(alpha)
        sin_alpha = jnp.sin(alpha)

        # Stack features
        if jnp.ndim(alpha) == 0:
            features = jnp.array([[cos_alpha, sin_alpha]])
        else:
            features = jnp.stack([cos_alpha, sin_alpha], axis=-1)

        # Predict
        predictions = self._predict_fn(self._model.params, features)

        # Extract cx, cz
        cx = predictions[..., 0]
        cz = predictions[..., 1]

        return cx, cz

    @property
    def aero_type(self) -> str:
        return self._aero_type

    def __repr__(self) -> str:
        return f"AerodynamicsMLPPredictor(aero_type='{self._aero_type}')"


def _reconstruct_frozen_dict(flat_dict: dict) -> dict:
    """
    Reconstruct nested FrozenDict from flat safetensors dict.

    Args:
        flat_dict: Flat dictionary with dot-separated keys

    Returns:
        Nested dictionary structure
    """
    result = {}
    for key, value in flat_dict.items():
        parts = key.split(".")
        current = result
        for part in parts[:-1]:
            if part not in current:
                current[part] = {}
            current = current[part]
        current[parts[-1]] = value
    return result


def load_aerodynamics_mlp(
    aero_type: Literal["lyu", "bspline", "phi", "advanced"],
    checkpoint_path: str | Path | None = None,
) -> AerodynamicsMLPPredictor:
    """
    Load trained MLP predictor for aerodynamic type.

    Args:
        aero_type: Aerodynamic type identifier
        checkpoint_path: Path to checkpoint (default: auto-detect from parquet)

    Returns:
        AerodynamicsMLPPredictor instance
    """
    if checkpoint_path is None:
        # Auto-detect from parquet dataset
        from aerodynamics.data.dataset import AeroMLDataset

        dataset = AeroMLDataset.from_parquet(".artifacts/aerodynamics/aero_ml_dataset.parquet")
        filtered = dataset.filter_by_type(aero_type)
        weight_path = filtered.df["weight_path"].iloc[0]

        if weight_path is None:
            raise ValueError(f"No trained model found for aero_type '{aero_type}'. Train first.")

        checkpoint_path = weight_path

    return AerodynamicsMLPPredictor.load_from_checkpoint(checkpoint_path, aero_type)


def predict_coefficients(
    aero_type: Literal["lyu", "bspline", "phi", "advanced"],
    alpha: float,
) -> tuple[float, float]:
    """
    Predict aerodynamic coefficients from angle of attack.

    Convenience function for single prediction.

    Args:
        aero_type: Aerodynamic type identifier
        alpha: Angle of attack in radians

    Returns:
        (cx, cz): Force coefficients
    """
    predictor = load_aerodynamics_mlp(aero_type)
    cx, cz = predictor.predict(alpha)
    return float(cx), float(cz)
