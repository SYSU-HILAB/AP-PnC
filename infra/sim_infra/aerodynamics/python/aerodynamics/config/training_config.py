"""
Training configuration for aerodynamics MLP.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from tooling.env import artifact_path


@dataclass(frozen=True)
class TrainingConfig:
    """Configuration for MLP training."""

    aero_type: Literal["lyu", "bspline", "phi", "advanced"]

    # Data
    train_split: float = 0.999
    batch_size: int = 32
    shuffle_seed: int = 42

    # Model
    hidden_dim: int = 256
    learning_rate: float = 1e-4

    # Training
    num_epochs: int = 1000
    early_stopping_patience: int = 100
    val_frequency: int = 5

    # Outputs
    checkpoint_dir: str | Path = ".artifacts/aerodynamics/weights"

    def get_checkpoint_path(self) -> Path:
        """Get checkpoint path for this aerodynamic type."""
        return artifact_path(Path(self.checkpoint_dir) / f"{self.aero_type}_mlp_weights.safetensors")
