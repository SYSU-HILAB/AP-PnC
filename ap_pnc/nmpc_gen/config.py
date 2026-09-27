# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.


import os
from pathlib import Path

import numpy as np
import yaml


def _nmpc_namespace() -> dict:
    """Read the `nmpc:` section of the nmpc config (single source of truth)."""
    root = os.environ.get("AP_PNC_DIR")
    if not root:
        return {}
    for rel in ("bringup/config/nmpc.yaml", "core/bringup/config/nmpc.yaml"):
        path = Path(root) / rel
        if path.exists():
            return (yaml.safe_load(path.read_text()) or {}).get("nmpc", {}) or {}
    return {}


_NMPC = _nmpc_namespace()


class ControlConfig:
    """NMPC horizon convention (see nmpc.yaml)."""

    horizon_s = float(_NMPC.get("horizon_s", 1.0))
    traj_res_s = float(_NMPC.get("traj_res_s", 0.1))
    ctrl_frq = float(_NMPC.get("ctrl_frq", 50.0))
    # acados discretization: N intervals over the horizon
    nodes = int(round(horizon_s / traj_res_s))
    T = horizon_s


class MpcConfig:
    """
    simple mpc configuration
    """
    mpc_timeout = 3  # [s  second]
    nodes = ControlConfig.nodes
    T_horizon = ControlConfig.T

    cz = 0.0 * np.zeros((1,))
    # TODO keep sync with MpcConfig
    lb_r = np.array([-1.5, -2.5, -1.5])
    ub_r = np.array([1.5, 2.5, 1.5])

    lb_u = np.array([0.05, 0.05, 0.05, 0.05]) * 0
    ub_u = np.array([10., 10., 10., 10.]) * 0.55
    drag_est = 0.00
    # 1e1 15 15 20 OK
    qPos = np.array([ 15,  15,  10]) * 1 * 1.0

    qh = 1.5
    qv = 1.8 * 1.0 * 2.0
    qYB = np.array([qh*1e2, qh*1e2, qv*1e2]) * 1.0 * 0.1 * 3.5

    qvelh = 1.0 # 5.0
    qvelv = 1.0

    qVel = np.array([qvelh, qvelh, qvelv]) * 1

    wx=.1 * 0.0
    wy=0.5 * 0.0
    wz=.1 * 0.0
    qAngV = np.array([wx, wy, wz]) * 1
    qDynamicVel = np.zeros((1,))
    rU = np.array([1e1, 1e1, 1e1, 1e1]) * 1e-3 * 3

    Metrics = False

def main():
    pass

if __name__ == "__main__":
    main()
