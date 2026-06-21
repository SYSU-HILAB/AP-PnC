"""PID Controller with Anti-Windup for angular velocity tracking."""

from __future__ import annotations

import numpy as np

from ..config.sub_controllers import PIDAntiWindupConfig


class PIDAntiWindupController:
    """PID controller with anti-windup for tracking angular velocity."""

    def __init__(self, config: PIDAntiWindupConfig):
        self._config = config
        self._Kp = config.Kp
        self._Ki = config.Ki
        self._Kd = config.Kd
        self._integral_max = config.integral_max
        self._integral_min = config.integral_min
        self._output_min = config.output_min
        self._output_max = config.output_max
        self._dt = config.dt

        # State
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)

    def compute(
        self,
        desired: np.ndarray,
        measured: np.ndarray,
        dt: float | None = None,
    ) -> np.ndarray:
        """
        Compute PID control output with anti-windup.

        Args:
            desired: Desired angular velocity (3D)
            measured: Measured angular velocity (3D)
            dt: Time step (uses config default if None)

        Returns:
            Torque command (3D)
        """
        if dt is None:
            dt = self._dt

        error = desired - measured

        # Proportional term
        p_term = self._Kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = np.clip(self._integral, self._integral_min, self._integral_max)
        i_term = self._Ki * self._integral

        # Derivative term
        d_term = self._Kd * (error - self._prev_error) / dt

        # Compute output
        output = p_term + i_term + d_term

        # Clamp output per axis
        output = np.clip(output, self._output_min, self._output_max)

        # Update state
        self._prev_error = error.copy()

        return output

    def reset(self):
        """Reset controller state."""
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)
