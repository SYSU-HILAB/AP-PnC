"""PD Controller for proper acceleration tracking."""

from __future__ import annotations

import numpy as np

from ..config.sub_controllers import PDControllerConfig


class PDController:
    """PD controller for tracking proper acceleration."""

    def __init__(self, config: PDControllerConfig):
        self._config = config
        self._Kp = config.Kp
        self._Kd = config.Kd
        self._output_min = config.output_min
        self._output_max = config.output_max

        # State
        self._prev_error = 0.0
        self._dt = 0.02  # Default control period

    def compute(
        self,
        desired: float,
        measured: float,
    ) -> float:
        """
        Compute PD control output.

        Args:
            desired: Desired proper acceleration (scalar)
            measured: Measured proper acceleration (scalar)

        Returns:
            Total thrust command (scalar)
        """
        dt = self._dt

        error = desired - measured

        # PD control law
        p_term = self._Kp * error  # Sum to get total thrust
        d_term = self._Kd * (error - self._prev_error) / dt

        output = p_term + d_term

        # Clamp output
        output = np.clip(output, self._output_min, self._output_max)

        # Update state
        self._prev_error = error

        return output
