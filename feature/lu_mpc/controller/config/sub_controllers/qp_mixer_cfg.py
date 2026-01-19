"""QP Mixer Configuration for quadrotor control allocation."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class QpMixerConfig:
    """Configuration for QP control mixer."""

    # Physical parameters (must match simulator)
    dx: float = 0.32  # Arm length x (m)
    dy: float = 0.32  # Arm length y (m)
    c_tau: float = 0.032  # Torque coefficient

    # Motor constraints (from PX4 motor specs)
    u_min: float = 0.0  # Minimum motor thrust (N)
    u_max: float = 10.0  # Maximum motor thrust (N)
    du_min: float = -10.0  # Minimum motor rate (N/s)
    du_max: float = 10.0  # Maximum motor rate (N/s)

    # Control timing (should match control frequency)
    dt: float = 0.02  # Time step for motor dynamics (s)

    # QP weights (tunable for performance)
    W_tracking: np.ndarray = field(default_factory=lambda: np.eye(4))
    R_prev_cmd: float = 0.1  # Weight on command smoothness

    @classmethod
    def create_default(cls) -> QpMixerConfig:
        """Create default mixer configuration."""
        return cls()

    @classmethod
    def create_aggressive(cls) -> QpMixerConfig:
        """Create configuration for aggressive maneuvering."""
        return cls(
            W_tracking=np.diag([10.0, 5.0, 5.0, 1.0]),  # Prioritize thrust
            R_prev_cmd=0.01,  # Allow faster changes
        )

    @classmethod
    def create_smooth(cls) -> QpMixerConfig:
        """Create configuration for smooth, efficient flight."""
        return cls(
            W_tracking=np.diag([1.0, 1.0, 1.0, 1.0]),
            R_prev_cmd=1.0,  # Strong smoothness
        )


DEFAULT_QP_MIXER_CONFIG = QpMixerConfig()
