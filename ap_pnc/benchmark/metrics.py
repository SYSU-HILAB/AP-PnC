"""Tracking metrics for a benchmark run."""

from __future__ import annotations

import numpy as np


def tracking_metrics(records: dict[str, np.ndarray], settle_time: float = 0.0) -> dict[str, float]:
    """Position/velocity tracking error and control effort after ``settle_time``."""
    mask = records["t"] >= settle_time
    p = records["p"][mask]
    v = records["v"][mask]
    p_ref = records["p_ref"][mask]
    v_ref = records["v_ref"][mask]
    moment = records["moment"][mask]
    thrust = records["thrust"][mask]

    e_p = p - p_ref
    e_v = v - v_ref
    pos_err = np.linalg.norm(e_p, axis=1)
    vel_err = np.linalg.norm(e_v, axis=1)
    n = max(len(records["t"][mask]) - 1, 1)
    duration = float(records["t"][mask][-1] - records["t"][mask][0]) if pos_err.size else 0.0

    return {
        "pos_rmse": float(np.sqrt(np.mean(pos_err**2))),
        "pos_max": float(np.max(pos_err)),
        "pos_final": float(pos_err[-1]) if pos_err.size else float("nan"),
        "vel_rmse": float(np.sqrt(np.mean(vel_err**2))),
        "vel_max": float(np.max(vel_err)),
        "thrust_mean": float(np.mean(thrust)),
        "moment_rms": float(np.sqrt(np.mean(np.sum(moment**2, axis=1)))),
        "steps": int(n),
        "duration": duration,
    }
