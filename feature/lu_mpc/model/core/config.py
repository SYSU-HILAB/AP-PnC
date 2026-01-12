"""
Local copy of the MPC configuration objects used by the lu_mpc feature.

The production versions live under src/nmpc, but feature development should be
self-contained so we mirror the parameters here.
"""

from __future__ import annotations

import numpy as np


class ControlConfig:
    control_time = 0.1
    control_frequency = 1 / control_time
    nodes = 10
    T = control_time * nodes


class MpcConfig:
    """Simple MPC configuration container."""

    mpc_timeout = 3  # seconds
    nodes = ControlConfig.nodes
    T_horizon = ControlConfig.T

    cz = 0.0 * np.zeros((1,))
    lb_r = np.array([-1.5, -2.5, -1.5])
    ub_r = np.array([1.5, 2.5, 1.5])

    lb_u = np.array([0.05, 0.05, 0.05, 0.05]) * 0
    ub_u = np.array([10.0, 10.0, 10.0, 10.0]) * 0.55
    drag_est = 0.00

    qPos = np.array([15, 15, 10])
    qh = 1.5
    qv = 1.8 * 2.0
    qYB = np.array([qh * 1e2, qh * 1e2, qv * 1e2]) * 0.35

    qvelh = 1.0
    qvelv = 1.0
    qVel = np.array([qvelh, qvelh, qvelv])

    wx = 0.0
    wy = 0.0
    wz = 0.0
    qAngV = np.array([wx, wy, wz])
    qDynamicVel = np.zeros((1,))
    rU = np.array([1e1, 1e1, 1e1, 1e1]) * 1e-3 * 3

    Metrics = False
