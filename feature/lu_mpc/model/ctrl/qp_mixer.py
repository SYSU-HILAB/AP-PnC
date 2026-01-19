"""
QP-based control mixer for quadrotor control allocation.

This module provides a quadratic programming (QP) based mixer that converts
high-level control commands (total thrust + body torques) into individual
motor commands while respecting motor saturation and rate constraints.

The mixer is aerodynamic model-agnostic - it only uses the geometric
control allocation matrix.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import casadi as cs
import numpy as np


@dataclass
class QpMixerConfig:
    """Configuration for QP-based control mixer."""

    # Physical parameters (must match simulator)
    dx: float = 0.32  # Arm length x (m)
    dy: float = 0.32  # Arm length y (m)
    c_tau: float = 0.032  # Torque coefficient

    # Motor constraints
    u_min: float = 0.0  # Minimum motor thrust (N)
    u_max: float = 10.0  # Maximum motor thrust (N)
    du_min: float = -10.0  # Minimum motor rate (N/s)
    du_max: float = 10.0  # Maximum motor rate (N/s)

    # Control timing
    dt: float = 0.02  # Time step for motor dynamics (s)

    # QP weights
    W_tracking: np.ndarray | None = None  # Weight on control allocation (4x4)
    R_prev_cmd: float = 0.1  # Weight on u deviation

    def __post_init__(self):
        if self.W_tracking is None:
            # Equal weighting on all 4 control channels
            self.W_tracking = np.eye(4) * 1.0


class QpMixer:
    """
    QP-based control mixer for quadrotor control allocation.

    Converts desired total thrust and torques into individual motor commands
    while respecting motor saturation and rate constraints.

    Aerodynamic model-agnostic: works with any aerodynamic model.
    """

    def __init__(self, config: QpMixerConfig):
        """
        Initialize the QP mixer.

        Args:
            config: Mixer configuration object
        """
        self._config = config
        self._build_allocation_matrix()
        self._setup_qp_solver()

    def _build_allocation_matrix(self):
        """Build the control allocation matrix Γ (4x4)."""
        dx, dy, c_tau = self._config.dx, self._config.dy, self._config.c_tau

        # Γ maps motor thrusts to [F_total, τ_roll, τ_pitch, τ_yaw]
        self._Gamma = cs.DM([
            [1, 1, 1, 1],  # F_total
            [-dy, -dy, dy, dy],  # τ_roll
            [-dx, dx, dx, -dx],  # τ_pitch
            [-c_tau, c_tau, -c_tau, c_tau],  # τ_yaw
        ])

    def _setup_qp_solver(self):
        """
        Setup the CasADi QP solver.

        Creates symbolic QP problem:
            minimize: 0.5*z'Hz + g'z
            subject to: lb ≤ z ≤ ub
                      A_eq*z = b_eq
        where z = [u; du]
        """
        # Decision variables: z = [u; du]
        u = cs.SX.sym("u", 4)  # Motor thrusts
        du = cs.SX.sym("du", 4)  # Motor rates
        z = cs.vertcat(u, du)

        # Parameters (vary at each solve)
        tau_des = cs.SX.sym("tau_des", 4)  # Desired [F, τ_roll, τ_pitch, τ_yaw]
        u_prev = cs.SX.sym("u_prev", 4)  # Previous motor commands

        # Cost function
        cfg = self._config
        W = cs.DM(cfg.W_tracking)
        R = cfg.R_prev_cmd

        # Tracking error: ||Γu - τ_des||²_W
        tau_achieved = self._Gamma @ u
        tau_error = tau_des - tau_achieved
        tracking_cost = cs.dot(tau_error, (W @ tau_error))

        # Command smoothness: ||u - u_prev||²_R
        smoothness_cost = R * cs.sum1((u - u_prev) ** 2)

        # Total cost
        cost = 0.5 * (tracking_cost + smoothness_cost)

        # Constraints: motor limits and rate limits
        lb = cs.vertcat(
            cs.DM(cfg.u_min * np.ones(4)),
            cs.DM(cfg.du_min * np.ones(4))
        )
        ub = cs.vertcat(
            cs.DM(cfg.u_max * np.ones(4)),
            cs.DM(cfg.du_max * np.ones(4))
        )

        # Create NLPSolver (more reliable than qpsol)
        # Note: We can use nlpsol for QP problems since QP is a special case of NLP
        p = cs.vertcat(tau_des, u_prev)  # Parameters
        nlp = {
            'x': z,
            'p': p,  # Parameters: [tau_desired, u_prev]
            'f': cost,
        }

        opts = {
            'print_time': False,
            'ipopt': {
                'print_level': 0,
                'tol': 1e-6,
                'max_iter': 100,
            }
        }

        # Create solver (using CasADi's nlpsol with ipopt)
        self._solver = cs.nlpsol(
            'mixer_solver',
            'ipopt',
            nlp,
            opts
        )

        # Store bounds for use during solve
        self._lb = lb
        self._ub = ub

    def mix(
        self,
        tau_desired: np.ndarray,
        u_prev: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        Compute motor commands from desired control inputs.

        Args:
            tau_desired: Desired [F_total, τ_roll, τ_pitch, τ_yaw]
            u_prev: Previous motor commands (defaults to equal distribution)

        Returns:
            u: Motor thrusts [m1, m2, m3, m4]
            du: Motor rates [dm1, dm2, dm3, dm4]
            info: Dictionary with solver status, statistics
        """
        if u_prev is None:
            # Initialize with equal thrust distribution
            u_prev = np.ones(4) * tau_desired[0] / 4.0

        # Initial guess: equal thrust distribution
        z_init = np.concatenate([
            np.ones(4) * tau_desired[0] / 4.0,  # u
            np.zeros(4),  # du
        ])

        # Solve NLP with parameters
        sol = self._solver(
            x0=z_init,
            lbx=self._lb,
            ubx=self._ub,
            p=cs.vertcat(cs.DM(tau_desired), cs.DM(u_prev)),
        )

        z_opt = sol['x'].full().flatten()
        u = z_opt[:4]
        du = z_opt[4:]

        info = {
            'solver_status': sol.get('return_status', 'unknown'),
            'cost': float(sol['f']),
        }

        return u, du, info


def create_qp_mixer(config: Optional[QpMixerConfig] = None) -> QpMixer:
    """
    Factory function to create a QP mixer with default configuration.

    Args:
        config: Optional mixer configuration (uses defaults if None)

    Returns:
        Configured QpMixer instance
    """
    if config is None:
        config = QpMixerConfig()
    return QpMixer(config)
