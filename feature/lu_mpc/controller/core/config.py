"""
Local copy of the MPC configuration objects used by the lu_mpc feature.

The production versions live under src/nmpc, but feature development should be
self-contained so we mirror the parameters here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ControlConfig:
    """Basic controller timing configuration."""

    ctrl_dt: float = 0.1
    nodes: int = 5

    @property
    def control_frequency(self) -> float:
        return 1.0 / self.ctrl_dt

    @property
    def T(self) -> float:
        return self.ctrl_dt * self.nodes


@dataclass
class MpcConfig:
    """Cleaned up MPC configuration."""

    control: ControlConfig = field(default_factory=ControlConfig)
    lb_r: np.ndarray = field(
        default_factory=lambda: np.array([-1.5, -2.5, -1.5], dtype=float)
    )
    ub_r: np.ndarray = field(
        default_factory=lambda: np.array([1.5, 2.5, 1.5], dtype=float)
    )

    lb_u: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    ub_u: np.ndarray = field(default_factory=lambda: np.array([5.5, 5.5, 5.5, 5.5]))

    qPos: np.ndarray = field(default_factory=lambda: np.array([15.0, 15.0, 100.0]))
    qYB: np.ndarray = field(default_factory=lambda: np.array([52.5, 52.5, 126.0]))
    qVel: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))

    cz_rate: float = 0.1

    qAngV: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    rU: np.ndarray = field(default_factory=lambda: np.array([0.001, 0.001, 0.001, 0.001]))

    @property
    def nodes(self) -> int:
        return self.control.nodes

    @property
    def T_horizon(self) -> float:
        return self.control.T

    @staticmethod
    def _yb_weights(qh: float, qv: float) -> np.ndarray:
        return np.array([qh, qh, qv], dtype=float) * 100.0 * 0.35

    @staticmethod
    def _vel_weights(qvelh: float, qvelv: float) -> np.ndarray:
        return np.array([qvelh, qvelh, qvelv], dtype=float)

    @classmethod
    def create_high_performance(cls) -> "MpcConfig":
        qh = 2.0
        qv = 5.0
        return cls(qYB=cls._yb_weights(qh, qv))



DEFAULT_CONTROL_CONFIG = ControlConfig()
DEFAULT_MPC_CONFIG = MpcConfig()
