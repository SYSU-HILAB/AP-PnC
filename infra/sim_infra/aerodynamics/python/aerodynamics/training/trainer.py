"""
Training pipeline for aerodynamics MLP.
"""

from dataclasses import dataclass
from pathlib import Path

import jax
import jax.numpy as jnp
import optax
from flax.training import train_state

from aerodynamics.config.training_config import TrainingConfig
from aerodynamics.data.dataset import AeroMLDataset
from aerodynamics.models.aerodynamics_mlp import AerodynamicsMLP
from aerodynamics.training.checkpoints import save_safetensors


@dataclass
class TrainingResult:
    """Result of training."""

    checkpoint_path: Path
    train_metrics: dict
    epoch_count: int


class AeroMLPTrainer:
    """Trainer for aerodynamics MLP models."""

    def __init__(
        self,
        aero_type: str,
        config: TrainingConfig,
        dataset: AeroMLDataset,
    ):
        """
        Initialize trainer.

        Args:
            aero_type: Aerodynamic type identifier
            config: Training configuration
            dataset: Full dataset (will be filtered by aero_type)
        """
        self._aero_type = aero_type
        self._config = config

        # Filter dataset by aero type - use all data for training
        self._train_dataset = dataset.filter_by_type(aero_type)

        # Create model
        key = jax.random.PRNGKey(config.shuffle_seed)
        self._model = AerodynamicsMLP.create_for_aero_type(
            aero_type=aero_type,
            hidden_dim=config.hidden_dim,
            key=key,
        )

        # Setup optimizer
        self._tx = optax.adam(learning_rate=config.learning_rate)

        print(f"Initialized trainer for {aero_type}")
        print(f"  Train samples: {len(self._train_dataset)}")

    def train(self) -> TrainingResult:
        """
        Train model on all data.

        Returns:
            TrainingResult with checkpoint path and metrics
        """
        # Initialize training state
        key = jax.random.PRNGKey(self._config.shuffle_seed)
        state = self._create_train_state(key)

        # Training loop
        train_features = self._train_dataset.get_features()
        train_targets = self._train_dataset.get_targets()

        for epoch in range(self._config.num_epochs):
            # Training step
            state, train_loss = self._train_step(state, train_features, train_targets)

            # Print progress
            if epoch % self._config.val_frequency == 0:
                train_mae = self._compute_mae(state.params, train_features, train_targets)
                print(f"Epoch {epoch:3d}: train_loss={train_loss:.6f}, train_mae={train_mae:.6f}")

        # Save final checkpoint
        checkpoint_path = self._config.get_checkpoint_path()
        save_safetensors(
            state.params,
            checkpoint_path,
            metadata={"aero_type": self._aero_type, "hidden_dim": str(self._config.hidden_dim)},
        )

        # Final metrics
        train_loss = self._compute_loss(state.params, train_features, train_targets)
        train_mae = self._compute_mae(state.params, train_features, train_targets)

        return TrainingResult(
            checkpoint_path=checkpoint_path,
            train_metrics={"loss": float(train_loss), "mae": float(train_mae)},
            epoch_count=epoch + 1,
        )

    def _create_train_state(self, key: jax.Array):
        """Create initial training state."""
        params = self._model.params
        return train_state.TrainState.create(apply_fn=self._model.model.apply, params=params, tx=self._tx)

    def _train_step(self, state, features, targets):
        """Single training step."""

        def loss_fn(params):
            predictions = state.apply_fn(params, features)
            loss_value = jnp.mean((predictions - targets) ** 2)
            return loss_value

        loss, grads = jax.value_and_grad(loss_fn)(state.params)
        state = state.apply_gradients(grads=grads)
        return state, loss

    def _compute_loss(self, params, features, targets):
        """Compute MSE loss."""

        def loss_fn(params):
            predictions = self._model.model.apply(params, features)
            loss = jnp.mean((predictions - targets) ** 2) * 100.0
            return loss

        return loss_fn(params)

    def _compute_mae(self, params, features, targets):
        """Compute Mean Absolute Error."""
        predictions = self._model.model.apply(params, features)
        mae = jnp.mean(jnp.abs(predictions - targets))
        return mae


def train_single_aero_type(
    aero_type: str,
    dataset: AeroMLDataset | None = None,
    config: TrainingConfig | None = None,
) -> TrainingResult:
    """
    Train a single aerodynamic type.

    Args:
        aero_type: Aerodynamic type to train
        dataset: Dataset (will load from default if None)
        config: Training config (will use default if None)

    Returns:
        TrainingResult
    """
    if config is None:
        config = TrainingConfig(aero_type=aero_type)

    if dataset is None:
        dataset = AeroMLDataset.from_parquet(".artifacts/aerodynamics/aero_ml_dataset.parquet")

    trainer = AeroMLPTrainer(aero_type, config, dataset)
    result = trainer.train()

    print(f"\nTraining completed for {aero_type}")
    print(f"  Checkpoint: {result.checkpoint_path}")
    print(f"  Train MAE: {result.train_metrics['mae']:.6f}")

    # Update dataset with weight path
    dataset.update_weight_path(aero_type, result.checkpoint_path)
    dataset.to_parquet(".artifacts/aerodynamics/aero_ml_dataset.parquet")

    return result


def train_all_aero_types(
    dataset: AeroMLDataset | None = None,
) -> dict[str, TrainingResult]:
    """
    Train all 4 aerodynamic types (lyu, bspline, phi, advanced).

    Uses CSV data source to avoid pybind11 binding issues.

    Args:
        dataset: Dataset (will load from default if None)

    Returns:
        Dictionary mapping aero_type to TrainingResult
    """
    if dataset is None:
        dataset = AeroMLDataset.from_parquet(".artifacts/aerodynamics/aero_ml_dataset.parquet")

    aero_types = ["lyu", "bspline", "phi", "advanced"]
    results = {}

    for aero_type in aero_types:
        print(f"\n{'=' * 60}")
        print(f"Training {aero_type}")
        print(f"{'=' * 60}")

        result = train_single_aero_type(aero_type, dataset)
        results[aero_type] = result

    print(f"\n{'=' * 60}")
    print("All training completed!")
    print(f"{'=' * 60}")

    for aero_type, result in results.items():
        print(f"  {aero_type}: train_mae={result.train_metrics['mae']:.6f}")

    return results
