"""SE3 geometric controller (Lee et al., simplified).

Tracks a reference trajectory (p_d, v_d, a_d, heading from yb_d) and outputs a
collective thrust along body z plus a body moment. World frame is z-up, body is
FLU (x forward, y left, z up), matching the planning core.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..plant import VehicleState
from ..reference import Reference
from .base import Control

_E3 = np.array([0.0, 0.0, 1.0])


def _vee(S: np.ndarray) -> np.ndarray:
    """Inverse hat map."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


@dataclass
class SE3Gains:
    """Per-axis position/velocity/attitude gains."""

    kp: np.ndarray = field(default_factory=lambda: np.full(3, 10.0))
    kv: np.ndarray = field(default_factory=lambda: np.full(3, 5.0))
    kR: np.ndarray = field(default_factory=lambda: np.full(3, 80.0))
    kOmega: np.ndarray = field(default_factory=lambda: np.full(3, 10.0))


class SE3Controller:
    """Geometric SE(3) tracking controller."""

    def __init__(
        self,
        mass: float = 1.0,
        inertia: np.ndarray | None = None,
        gravity: float = 9.81,
        gains: SE3Gains | None = None,
        max_thrust: float = 40.0,
        max_moment: float = 2.0,
    ) -> None:
        self.mass = float(mass)
        self.gravity = float(gravity)
        self.J = (
            np.asarray(inertia, dtype=float)
            if inertia is not None
            else np.diag([0.01715066, 0.01818037, 0.01987597])
        )
        g = gains or SE3Gains()
        # scale gains by mass / inertia so the same numbers work across vehicles
        self.kp = np.asarray(g.kp, dtype=float) * self.mass
        self.kv = np.asarray(g.kv, dtype=float) * self.mass
        self.kR = np.asarray(g.kR, dtype=float) * np.diag(self.J)
        self.kOmega = np.asarray(g.kOmega, dtype=float) * np.diag(self.J)
        self.max_thrust = float(max_thrust)
        self.max_moment = float(max_moment)
        self._ref: Reference | None = None

    def reset(self, ref: Reference) -> None:
        self._ref = ref

    def act(self, t: float, state: VehicleState) -> Control:
        if self._ref is None:
            raise RuntimeError("SE3Controller.reset(reference) must be called first")

        p_d, v_d, a_d, yb_d = self._ref.sample(t)
        m = self.mass

        # Desired force (world) and thrust direction.
        e_p = state.p - p_d
        e_v = state.v - v_d
        F_d = -self.kp * e_p - self.kv * e_v + m * a_d + m * self.gravity * _E3
        F_norm = float(np.linalg.norm(F_d))
        if F_norm < 1e-6:
            F_d = m * self.gravity * _E3
            F_norm = float(np.linalg.norm(F_d))
        b3_d = F_d / F_norm

        # Heading: use desired body y (yb_d) orthogonalized against b3_d.
        b2_d = yb_d - float(yb_d @ b3_d) * b3_d
        n2 = float(np.linalg.norm(b2_d))
        if n2 < 1e-6:
            # fall back to world-y projection
            b2_d = np.array([0.0, 1.0, 0.0]) - b3_d[1] * b3_d
            n2 = float(np.linalg.norm(b2_d))
            if n2 < 1e-6:
                b2_d = np.array([1.0, 0.0, 0.0])
                n2 = 1.0
        b2_d = b2_d / n2
        b1_d = np.cross(b2_d, b3_d)
        R_d = np.column_stack([b1_d, b2_d, b3_d])

        # Collective thrust is the projection of the desired force on body z.
        thrust = float(F_d @ (state.R @ _E3))

        # Attitude error on SO(3) and body-rate error (no reference rate here).
        R = state.R
        e_R = 0.5 * _vee(R_d.T @ R - R.T @ R_d)
        e_omega = state.omega

        moment = (
            -self.kR * e_R
            - self.kOmega * e_omega
            + np.cross(state.omega, self.J @ state.omega)
        )

        thrust = float(np.clip(thrust, 0.0, self.max_thrust))
        moment = np.clip(moment, -self.max_moment, self.max_moment)
        return Control(thrust=thrust, moment=moment)
