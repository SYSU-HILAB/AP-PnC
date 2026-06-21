"""PID Controller with Anti-Windup Configuration for angular velocity tracking."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PIDAntiWindupConfig:
    """Configuration for PID controller with anti-windup for angular velocity."""

    Kp: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.4, 0.2]))  # Proportional gains
    Ki: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.2, 0.05]))  # Integral gains
    Kd: np.ndarray = field(default_factory=lambda: np.array([0.001, 0.01, 0.001]))  # Derivative gains

    # Anti-windup settings
    integral_max: float = 0.2  # Maximum integral error (anti-windup)
    integral_min: float = -0.2  # Minimum integral error (anti-windup)

    # Output limits (torque limits)
    output_min: np.ndarray = field(default_factory=lambda: np.array([-0.2, -0.4, -0.2]))
    output_max: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.4, 0.2]))

    # Timing
    dt: float = 0.02  # Controller time step (s)

    @classmethod
    def create_default(cls) -> PIDAntiWindupConfig:
        """Create default PID configuration."""
        return cls()


DEFAULT_PID_CONFIG = PIDAntiWindupConfig()
