"""Reference trajectory: the interface between the planner and a controller.

A :class:`Reference` is a plain numpy container (no ROS, no CSV) holding the
time-ordered samples produced by ``planner_bindings.plan`` and offering
linear interpolation so a controller can query it at any time.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _interp_column(t_query: np.ndarray, t: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Linear interpolation of a (N,) or (N, k) array over time."""
    values = np.asarray(values)
    if values.ndim == 1:
        return np.interp(t_query, t, values)
    return np.column_stack([np.interp(t_query, t, values[:, i]) for i in range(values.shape[1])])


@dataclass
class Reference:
    """Time-ordered reference trajectory (world z-up, body FLU)."""

    t: np.ndarray          # (N,)   time [s]
    p: np.ndarray          # (N, 3) position [m]
    v: np.ndarray          # (N, 3) velocity [m/s]
    a: np.ndarray          # (N, 3) acceleration [m/s^2]
    yb: np.ndarray         # (N, 3) desired body y-axis (world)
    omega: np.ndarray      # (N, 3) body rate [rad/s]
    thrust: np.ndarray     # (N,)   flatness collective thrust [N]

    def __post_init__(self) -> None:
        for name in ("t", "p", "v", "a", "yb", "omega"):
            setattr(self, name, np.asarray(getattr(self, name), dtype=float))
        self.thrust = np.asarray(self.thrust, dtype=float)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if self.t.size else 0.0

    def sample(self, t_query: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (p_d, v_d, a_d, yb_d) at ``t_query`` (clamped to range)."""
        tq = np.clip(np.atleast_1d(t_query), self.t[0], self.t[-1])
        p = _interp_column(tq, self.t, self.p)[0]
        v = _interp_column(tq, self.t, self.v)[0]
        a = _interp_column(tq, self.t, self.a)[0]
        yb = _interp_column(tq, self.t, self.yb)[0]
        n = np.linalg.norm(yb)
        if n > 1e-9:
            yb = yb / n
        return p, v, a, yb
