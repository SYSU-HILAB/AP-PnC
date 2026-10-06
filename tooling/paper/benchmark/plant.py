"""Rigid-body plant for the benchmark pipeline (world z-up, body FLU)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class VehicleState:
    """Rigid-body state."""

    p: np.ndarray = field(default_factory=lambda: np.zeros(3))       # world position [m]
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))       # world velocity [m/s]
    R: np.ndarray = field(default_factory=lambda: np.eye(3))         # body->world rotation
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))   # body rate [rad/s]


@dataclass
class PlantConfig:
    """Quadrotor rigid-body parameters."""

    mass: float = 1.0
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([0.01715066, 0.01818037, 0.01987597])
    )
    gravity: float = 9.81


def _skew(w: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]
    )


def _rodrigues(rot_vec: np.ndarray) -> np.ndarray:
    """Exponential map of a rotation vector (3,) -> rotation matrix (3, 3)."""
    theta = float(np.linalg.norm(rot_vec))
    if theta < 1e-12:
        return np.eye(3) + _skew(rot_vec)
    axis = rot_vec / theta
    K = _skew(axis)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


class QuadPlant:
    """Simple rigid-body quadrotor with body-z thrust and body moments."""

    def __init__(self, config: PlantConfig | None = None) -> None:
        self.cfg = config or PlantConfig()
        self._J = np.asarray(self.cfg.inertia, dtype=float)
        self._J_inv = np.linalg.inv(self._J)
        self._g_world = np.array([0.0, 0.0, -self.cfg.gravity])

    def step(self, state: VehicleState, thrust: float, moment: np.ndarray, dt: float) -> VehicleState:
        """Advance the state by ``dt`` (semi-implicit, midpoint attitude)."""
        R = state.R
        omega = state.omega
        moment = np.asarray(moment, dtype=float)

        # Translational dynamics: thrust along body z, gravity in world.
        acc = self._g_world + R @ np.array([0.0, 0.0, thrust / self.cfg.mass])

        # Rotational dynamics: J w_dot = tau - w x (J w)
        omega_dot = self._J_inv @ (moment - np.cross(omega, self._J @ omega))

        p_next = state.p + state.v * dt + 0.5 * acc * dt * dt
        v_next = state.v + acc * dt
        omega_next = omega + omega_dot * dt
        R_next = R @ _rodrigues(0.5 * (omega + omega_next) * dt)

        return VehicleState(p=p_next, v=v_next, R=R_next, omega=omega_next)
