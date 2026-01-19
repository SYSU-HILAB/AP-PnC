"""PD Controller Configuration for proper acceleration tracking."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PDControllerConfig:
    """Configuration for PD controller tracking proper acceleration."""

    Kp: float = 0.1  # Proportional gain
    Kd: float = 0.0  # Derivative gain

    # Output limits
    output_min: float = -5.0  # Minimum total thrust (N)
    output_max: float = 5.0  # Maximum total thrust (N)

    @classmethod
    def create_default(cls) -> PDControllerConfig:
        """Create default PD configuration."""
        return cls()

    @classmethod
    def create_aggressive(cls) -> PDControllerConfig:
        """Create configuration for aggressive tracking."""
        return cls(
            Kp=2.0,
            Kd=0.2,
        )


DEFAULT_PD_CONFIG = PDControllerConfig()
