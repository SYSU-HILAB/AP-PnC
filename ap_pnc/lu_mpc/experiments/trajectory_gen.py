"""
Minimal-jerk multi-segment trajectory generator.

Solves a piecewise quintic (minimum-jerk) spline through user-specified
waypoints with fixed segment durations.  Outputs a CSV compatible with
``TrajectoryReference`` so that it can be consumed by the wind-test
orchestrator.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import casadi as cs
import numpy as np

# Inlined from ap_pnc.utils.math_utils to avoid ROS2 import chain
_WING_TO_QUAD_QUAT = np.array([0.0, 0.707, 0.0, 0.707], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

# ────────────────────────────────────────────────────────────────────
# Config
# ────────────────────────────────────────────────────────────────────


@dataclasses.dataclass
class TrajectoryGenConfig:
    dt: float = 0.02
    waypoints: list[list[float]] | None = None
    segment_durations: list[float] | None = None
    a_max: float = 15.0
    output_dir: str = "artifacts/simulations/temp_data"
    filename: str = "planning_line_vel12.0_thrust-2.5-2.5.csv"

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> TrajectoryGenConfig:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    @classmethod
    def from_toml(cls, path: str | Path) -> TrajectoryGenConfig:
        import tomllib

        with open(path, "rb") as f:
            d = tomllib.load(f)
        return cls.from_dict(d)


# ────────────────────────────────────────────────────────────────────
# 1-D minimal-jerk spline through M waypoints
# ────────────────────────────────────────────────────────────────────


def _segment_jerk_cost_matrix(T: float) -> cs.DM:
    M_mat = cs.DM(
        [
            [T**3, T**4, T**5],
            [3 * T**2, 4 * T**3, 5 * T**4],
            [6 * T, 12 * T**2, 20 * T**3],
        ]
    )
    K_mat = cs.DM(
        [
            [36 * T, 72 * T**2, 120 * T**3],
            [72 * T**2, 192 * T**3, 360 * T**4],
            [120 * T**3, 360 * T**4, 720 * T**5],
        ]
    )
    D_mat = cs.DM(
        [
            [-1, -T, -(T**2) / 2, 1, 0, 0],
            [0, -1, -T, 0, 1, 0],
            [0, 0, -1, 0, 0, 1],
        ]
    )
    Minv = cs.solve(M_mat, cs.DM.eye(3))
    Q = D_mat.T @ Minv.T @ K_mat @ Minv @ D_mat
    return cs.DM(Q)


def _solve_minimal_jerk_1d(p: np.ndarray, T: np.ndarray) -> np.ndarray:
    """p : (M,)  waypoint positions along one axis
       T : (M-1,) segment durations

    Returns x_opt : (2*(M-2),)  [v1 a1 v2 a2 … v_{M-2} a_{M-2}]
    """
    M = len(p)
    N = M - 2  # internal waypoints
    if N < 1:
        return np.array([])

    # ---- symbolic cost ---- #
    v = cs.MX.sym("v", N)
    a = cs.MX.sym("a", N)
    x = cs.vertcat(v, a)

    cost = 0
    for i in range(M - 1):
        Ti = T[i]
        Q = _segment_jerk_cost_matrix(Ti)

        p_s = p[i]
        p_e = p[i + 1]

        if i == 0:
            v_s = 0.0
            a_s = 0.0
        else:
            v_s = v[i - 1]
            a_s = a[i - 1]

        if i == M - 2:
            v_e = 0.0
            a_e = 0.0
        else:
            v_e = v[i]
            a_e = a[i]

        b = cs.vertcat(p_s, v_s, a_s, p_e, v_e, a_e)
        cost += b.T @ Q @ b

    # ---- Hessian / gradient (constant for quadratic cost) ---- #
    g_func = cs.Function("g", [x], [cs.jacobian(cost, x).T])
    H_func = cs.Function("H", [x], [cs.jacobian(cs.jacobian(cost, x).T, x)])
    x0 = cs.DM.zeros(2 * N)
    g0 = g_func(x0).full().flatten()
    H0 = H_func(x0).full()

    # Solve H0 · x = -g0
    x_opt = np.linalg.solve(H0, -g0)
    return x_opt


def _quintic_coeffs(
    p_s: float, v_s: float, a_s: float, p_e: float, v_e: float, a_e: float, T: float
) -> np.ndarray:
    """Return (6,) array of quintic coefficients c0 … c5 for one segment."""
    M_mat = np.array(
        [
            [T**3, T**4, T**5],
            [3 * T**2, 4 * T**3, 5 * T**4],
            [6 * T, 12 * T**2, 20 * T**3],
        ]
    )
    dp = p_e - p_s - v_s * T - a_s * T**2 / 2
    dv = v_e - v_s - a_s * T
    da = a_e - a_s
    c345 = np.linalg.solve(M_mat, np.array([dp, dv, da]))
    return np.array([p_s, v_s, a_s / 2, c345[0], c345[1], c345[2]])


# ────────────────────────────────────────────────────────────────────
# Trajectory sampling
# ────────────────────────────────────────────────────────────────────


def _eval_quintic_poly(coeffs: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Evaluate quintic at times t (vectorised)."""
    # coeffs: (6,)  →  s(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    return (
        coeffs[0]
        + coeffs[1] * t
        + coeffs[2] * t**2
        + coeffs[3] * t**3
        + coeffs[4] * t**4
        + coeffs[5] * t**5
    )


def _eval_quintic_deriv(coeffs: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (
        coeffs[1]
        + 2 * coeffs[2] * t
        + 3 * coeffs[3] * t**2
        + 4 * coeffs[4] * t**3
        + 5 * coeffs[5] * t**4
    )


def _eval_quintic_accel(coeffs: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (
        2 * coeffs[2]
        + 6 * coeffs[3] * t
        + 12 * coeffs[4] * t**2
        + 20 * coeffs[5] * t**3
    )


def _build_coeffs_3d(
    wp: np.ndarray, T: np.ndarray, x_opt: dict[int, np.ndarray]
) -> dict[int, list[np.ndarray]]:
    """Return {coord_idx: list of (6,) coeffs per segment}."""
    M = wp.shape[0]
    coeffs = {d: [] for d in range(3)}

    for seg_i in range(M - 1):
        Ti = T[seg_i]
        for d in range(3):
            p_s = wp[seg_i, d]
            p_e = wp[seg_i + 1, d]

            if seg_i == 0:
                v_s = 0.0
                a_s = 0.0
            else:
                idx = seg_i - 1  # internal index
                v_s = x_opt[d][2 * idx]
                a_s = x_opt[d][2 * idx + 1]

            if seg_i == M - 2:
                v_e = 0.0
                a_e = 0.0
            else:
                idx = seg_i
                v_e = x_opt[d][2 * idx]
                a_e = x_opt[d][2 * idx + 1]

            coeffs[d].append(_quintic_coeffs(p_s, v_s, a_s, p_e, v_e, a_e, Ti))

    return coeffs


# ────────────────────────────────────────────────────────────────────
# Quaternion / orientation helpers
# ────────────────────────────────────────────────────────────────────


def _heading_from_vel(vel: np.ndarray) -> np.ndarray:
    """Return heading angle [rad] from velocity array (N,3)."""
    return np.arctan2(vel[:, 1], vel[:, 0])


def _quat_quad_flu_from_heading(heading: np.ndarray) -> np.ndarray:
    """Return (N,4) quaternion [w,x,y,z] for pure z-rotation by heading."""
    h2 = heading / 2.0
    q = np.zeros((len(heading), 4))
    q[:, 0] = np.cos(h2)  # w
    q[:, 3] = np.sin(h2)  # z
    return q


def _quat_wing_from_quad_flu(q_flu: np.ndarray) -> np.ndarray:
    """Convert quad-FLU quaternion → wing-FRD quaternion.

    wing_frd_to_quad_flu(q_wing) = q_wing * q_transform = q_flu
    → q_wing = q_flu * conj(q_transform)
    """
    q_t_conj = _quat_conjugate(_WING_TO_QUAD_QUAT)  # (4,)
    out = np.zeros_like(q_flu)
    for i in range(len(q_flu)):
        out[i] = _quat_multiply(q_flu[i], q_t_conj)
    return out


def _yb_from_heading(heading: np.ndarray) -> np.ndarray:
    """Body Y-axis in world frame for pure z-rotation."""
    yb = np.zeros((len(heading), 3))
    yb[:, 0] = -np.sin(heading)
    yb[:, 1] = np.cos(heading)
    return yb


# ────────────────────────────────────────────────────────────────────
# Main generator
# ────────────────────────────────────────────────────────────────────


def generate_trajectory(config: TrajectoryGenConfig) -> dict[str, np.ndarray]:
    """
    Generate a minimal-jerk multi-segment trajectory and return
    a dict in the format expected by ``TrajectoryReference.__init__``.
    """
    wp = np.asarray(config.waypoints, dtype=float)  # (M, 3)
    T = np.asarray(config.segment_durations, dtype=float)  # (M-1,)

    M = wp.shape[0]
    if M != len(T) + 1:
        raise ValueError(f"Expected {len(T)+1} waypoints for {len(T)} segments, got {M}")

    # ── 1. Solve minimal-jerk per coordinate ── #
    x_opt: dict[int, np.ndarray] = {}
    for d in range(3):
        x_opt[d] = _solve_minimal_jerk_1d(wp[:, d], T)

    # ── 2. Quintic coefficients per segment, per coordinate ── #
    coeffs = _build_coeffs_3d(wp, T, x_opt)

    # ── 3. Sample ── #
    # Build cumulative time grid per segment
    seg_starts = np.concatenate([[0.0], np.cumsum(T)])
    total_dur = seg_starts[-1]
    t_global = np.arange(0, total_dur, config.dt)
    N = len(t_global)

    pos = np.zeros((N, 3))
    vel = np.zeros((N, 3))
    acc = np.zeros((N, 3))

    for i, _ts in enumerate(T):
        start = seg_starts[i]
        end = seg_starts[i + 1]
        mask = (t_global >= start) & (t_global < end - 1e-10)
        # Handle last point inclusively
        if i == len(T) - 1:
            mask = (t_global >= start) & (t_global <= end + 1e-10)
        t_loc = t_global[mask] - start

        for d in range(3):
            c = coeffs[d][i]
            pos[mask, d] = _eval_quintic_poly(c, t_loc)
            vel[mask, d] = _eval_quintic_deriv(c, t_loc)
            acc[mask, d] = _eval_quintic_accel(c, t_loc)

    # ── 4. Orientation ── #
    heading = _heading_from_vel(vel)
    q_flu = _quat_quad_flu_from_heading(heading)
    q_wing = _quat_wing_from_quad_flu(q_flu)
    yb = _yb_from_heading(heading)
    alpha = np.zeros(N)  # level flight → zero angle of attack

    # ── 5. Pack into trajectory dict ── #
    trajectory: dict[str, np.ndarray] = {}
    trajectory["time"] = t_global
    trajectory["position"] = pos
    trajectory["velocity"] = vel
    trajectory["yb"] = yb
    trajectory["alpha"] = alpha
    trajectory["df_phi_alpha"] = alpha.copy()
    trajectory["df_lyu_alpha"] = alpha.copy()
    trajectory["df_phi_quat"] = q_wing.copy()
    trajectory["df_lyu_quat"] = q_wing.copy()
    # Aero derivatives – online-estimated by cz_state, set to zero
    trajectory["S_ap"] = np.zeros((N, 3))
    trajectory["S_ap_dot"] = np.zeros((N, 3))
    trajectory["S_Omega"] = np.zeros((N, 3))
    trajectory["B_Omega_df_lyu"] = np.zeros((N, 3))
    trajectory["B_fa_z_df_lyu"] = np.zeros(N)

    return trajectory


# ────────────────────────────────────────────────────────────────────
# CSV output
# ────────────────────────────────────────────────────────────────────


def trajectory_to_csv(trajectory: dict[str, np.ndarray], path: str | Path) -> None:
    """
    Write trajectory to CSV in the format expected by
    ``load_trajectory_csv``.
    """
    data = np.column_stack(
        [
            trajectory["time"],
            trajectory["position"][:, 0],
            trajectory["position"][:, 1],
            trajectory["position"][:, 2],
            trajectory["velocity"][:, 0],
            trajectory["velocity"][:, 1],
            trajectory["velocity"][:, 2],
            trajectory["yb"][:, 0],
            trajectory["yb"][:, 1],
            trajectory["yb"][:, 2],
            trajectory["df_phi_alpha"],
            trajectory["df_lyu_alpha"],
            trajectory["df_phi_quat"][:, 0],
            trajectory["df_phi_quat"][:, 1],
            trajectory["df_phi_quat"][:, 2],
            trajectory["df_phi_quat"][:, 3],
            trajectory["df_lyu_quat"][:, 0],
            trajectory["df_lyu_quat"][:, 1],
            trajectory["df_lyu_quat"][:, 2],
            trajectory["df_lyu_quat"][:, 3],
            trajectory["S_ap"][:, 0],
            trajectory["S_ap"][:, 2],
            trajectory["S_ap_dot"][:, 0],
            trajectory["S_ap_dot"][:, 2],
            trajectory["S_Omega"][:, 0],
            trajectory["S_Omega"][:, 1],
            trajectory["S_Omega"][:, 2],
            trajectory["B_Omega_df_lyu"][:, 0],
            trajectory["B_Omega_df_lyu"][:, 1],
            trajectory["B_Omega_df_lyu"][:, 2],
            trajectory["B_fa_z_df_lyu"],
        ]
    )

    header = (
        "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,"
        "yb_x,yb_y,yb_z,"
        "df_phi_alpha,df_lyu_alpha,"
        "df_phi_quat_w,df_phi_quat_x,df_phi_quat_y,df_phi_quat_z,"
        "df_lyu_quat_w,df_lyu_quat_x,df_lyu_quat_y,df_lyu_quat_z,"
        "S_ap_x,S_ap_z,"
        "S_ap_x_dot,S_ap_z_dot,"
        "S_Omega_x,S_Omega_y,S_Omega_z,"
        "B_Omega_x_df_lyu,B_Omega_y_df_lyu,B_Omega_z_df_lyu,"
        "B_fa_z_df_lyu"
    )

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        str(path),
        data,
        delimiter=",",
        header=header,
        comments="",
        fmt="%.10f",
    )
    print(f"[TrajectoryGen] Saved {len(trajectory['time'])} samples to {path}")


# ────────────────────────────────────────────────────────────────────
# Helper for quick circle waypoints
# ────────────────────────────────────────────────────────────────────


def make_circle_waypoints(
    num_segments: int,
    radius: float = 30.0,
    z: float = 2.0,
) -> list[list[float]]:
    """Create waypoints forming a horizontal circle, closed loop."""
    theta = np.linspace(0, 2 * np.pi, num_segments + 1)
    wp = np.column_stack(
        [radius * np.cos(theta), radius * np.sin(theta), np.full(num_segments + 1, z)]
    )
    return wp.tolist()


def make_default_config() -> TrajectoryGenConfig:
    """Return a default config: 8-segment circle at 30m radius."""
    num_seg = 8
    dur = 2.0
    return TrajectoryGenConfig(
        dt=0.02,
        waypoints=make_circle_waypoints(num_seg, radius=30.0, z=2.0),
        segment_durations=[dur] * num_seg,
        a_max=15.0,
        output_dir="artifacts/simulations/temp_data",
        filename="minimal_jerk_trajectory.csv",
    )
