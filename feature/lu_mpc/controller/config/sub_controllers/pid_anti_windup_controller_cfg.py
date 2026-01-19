"""PID Controller with Anti-Windup Configuration for angular velocity tracking."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class PIDAntiWindupConfig:
    """Configuration for PID controller with anti-windup for angular velocity."""

    Kp: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.5]))  # Proportional gains
    Ki: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.1]))  # Integral gains
    Kd: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.05, 0.05]))  # Derivative gains

    # Anti-windup settings
    integral_max: float = 1.0  # Maximum integral error (anti-windup)
    integral_min: float = -1.0  # Minimum integral error (anti-windup)

    # Output limits (torque limits)
    output_min: np.ndarray = field(default_factory=lambda: np.array([-1.0, -1.0, -0.5]))
    output_max: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 0.5]))

    # Timing
    dt: float = 0.02  # Controller time step (s)

    @classmethod
    def create_default(cls) -> PIDAntiWindupConfig:
        """Create default PID configuration."""
        return cls()

    @classmethod
    def create_high_gain(cls) -> PIDAntiWindupConfig:
        """Create high-gain configuration for faster tracking."""
        return cls(
            Kp=np.array([1.0, 1.0, 1.0]),
            Ki=np.array([0.2, 0.2, 0.2]),
            Kd=np.array([0.1, 0.1, 0.1]),
        )


DEFAULT_PID_CONFIG = PIDAntiWindupConfig()
