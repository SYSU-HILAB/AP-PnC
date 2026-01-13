# -*- coding: future_fstrings -*-
#
# Trajectory Loader Module
# Created on 2025-01-10
#
# This module provides utilities for loading and parsing trajectory CSV files
# for use with the nMPC controller.
#

import os
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import interp1d


TEMP_DATA_SUBDIR = Path("artifacts") / "simulations" / "temp_data"


def _repo_root() -> Path:
    """Best-effort repository root detection."""
    return Path(os.environ.get("AP_PNC_DIR", Path(__file__).resolve().parents[3]))


def _temp_data_dir() -> Path:
    return _repo_root() / TEMP_DATA_SUBDIR


def _find_planning_csv() -> Optional[Path]:
    """Locate the latest planning CSV emitted by the global planner."""
    temp_dir = _temp_data_dir()
    if not temp_dir.exists():
        return None

    planning_files = sorted(
        (
            path
            for path in temp_dir.glob("planning_*.csv")
            if path.is_file()
        ),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if planning_files:
        return planning_files[0]

    fallback = temp_dir / "planning.csv"
    if fallback.is_file():
        return fallback
    return None


def _stack_columns(data: np.ndarray, columns: Sequence[str]) -> np.ndarray:
    """Stack named columns into a matrix."""
    names = data.dtype.names or []
    missing = [name for name in columns if name not in names]
    if missing:
        raise ValueError(f"Missing required columns in trajectory CSV: {missing}")
    stacked = np.column_stack([np.atleast_1d(data[name]) for name in columns])
    return stacked


def load_trajectory_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """
    Load trajectory data from a planner-generated CSV.

    Supported columns include those emitted by global_planning.cpp:
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,thrust,df_alpha,real_alpha,
    aero_force_z,yb_x,yb_y,yb_z,quat_w,quat_x,quat_y,quat_z,ap_x,ap_y,ap_z,
    S_ap_x,S_ap_z,B_Omega_b_y,alpha_dot,B_aero_force_z

    Older v14-alpha.csv files (without quaternion and df_alpha) remain supported.
    """
    csv_path_obj = Path(csv_path)
    if not csv_path_obj.exists():
        raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")

    data = np.genfromtxt(csv_path_obj, delimiter=",", names=True)
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    trajectory: Dict[str, np.ndarray] = {}
    trajectory["time"] = np.atleast_1d(data["time"])
    trajectory["position"] = _stack_columns(data, ["pos_x", "pos_y", "pos_z"])
    trajectory["velocity"] = _stack_columns(data, ["vel_x", "vel_y", "vel_z"])
    trajectory["yb"] = _stack_columns(data, ["yb_x", "yb_y", "yb_z"])

    names = data.dtype.names or []
    thrust_col = "thrust" if "thrust" in names else None
    trajectory["thrust"] = np.atleast_1d(data[thrust_col]) if thrust_col else np.zeros_like(trajectory["time"])

    alpha_col = None
    if "df_alpha" in names:
        alpha_col = "df_alpha"
    elif "alpha" in names:
        alpha_col = "alpha"
    trajectory["alpha"] = np.atleast_1d(data[alpha_col]) if alpha_col else np.zeros_like(trajectory["time"])

    quat_columns = ["quat_w", "quat_x", "quat_y", "quat_z"]
    if all(col in names for col in quat_columns):
        trajectory["quaternion"] = _stack_columns(data, quat_columns)

    print(f"Loaded trajectory with {len(trajectory['time'])} points")
    print(f"  Time range: {trajectory['time'][0]:.2f} to {trajectory['time'][-1]:.2f}s")

    return trajectory


class TrajectoryReference:
    """
    Trajectory reference provider with interpolation.
    Provides reference states for the nMPC controller.
    """

    def __init__(self, trajectory_data: Dict[str, np.ndarray]):
        """
        Initialize trajectory reference.

        Args:
            trajectory_data: Dictionary from load_trajectory_csv()
        """
        self.time = trajectory_data['time']
        self.position = trajectory_data['position']
        self.velocity = trajectory_data['velocity']
        self.yb = trajectory_data['yb']
        self.thrust = trajectory_data.get('thrust', np.zeros(len(self.time)))
        self.alpha = trajectory_data.get('alpha', np.zeros(len(self.time)))
        self.quaternion = trajectory_data.get('quaternion')

        # Create interpolators
        self._pos_interp = [interp1d(self.time, self.position[:, i],
                                      kind='linear', fill_value='extrapolate')
                            for i in range(3)]
        self._vel_interp = [interp1d(self.time, self.velocity[:, i],
                                      kind='linear', fill_value='extrapolate')
                            for i in range(3)]
        self._yb_interp = [interp1d(self.time, self.yb[:, i],
                                     kind='linear', fill_value='extrapolate')
                           for i in range(3)]
        self._alpha_interp = interp1d(self.time, self.alpha,
                                      kind='linear', fill_value='extrapolate')
        self._quat_interp = None
        if self.quaternion is not None:
            self._quat_interp = [
                interp1d(self.time, self.quaternion[:, i],
                         kind='linear', fill_value='extrapolate')
                for i in range(4)
            ]

    def get_reference_at_time(self, t: float) -> Dict[str, np.ndarray]:
        """
        Get interpolated reference state at time t.

        Args:
            t: Time in seconds

        Returns:
            Dictionary with reference state:
            - position: (3,) array
            - velocity: (3,) array
            - yb: (3,) array
            - alpha: scalar
        """
        ref = {
            'position': np.array([f(t) for f in self._pos_interp]),
            'velocity': np.array([f(t) for f in self._vel_interp]),
            'yb': np.array([f(t) for f in self._yb_interp]),
            'alpha': float(self._alpha_interp(t)),
        }
        if self._quat_interp:
            quat = np.array([f(t) for f in self._quat_interp])
            norm = np.linalg.norm(quat)
            if norm > 1e-6:
                quat = quat / norm
            ref['quaternion'] = quat
        return ref

    def get_reference_window(self, t: float, horizon: float, dt: float) -> Dict[str, np.ndarray]:
        """
        Get reference trajectory for MPC prediction horizon.

        Args:
            t: Current time
            horizon: Prediction horizon length (seconds)
            dt: Time step within horizon

        Returns:
            Dictionary with reference arrays:
            - position: (N, 3) array
            - velocity: (N, 3) array
            - yb: (N, 3) array
        """
        N = int(horizon / dt)
        times = t + np.arange(N + 1) * dt

        position_ref = np.array([[f(tau) for f in self._pos_interp] for tau in times])
        velocity_ref = np.array([[f(tau) for f in self._vel_interp] for tau in times])
        yb_ref = np.array([[f(tau) for f in self._yb_interp] for tau in times])

        ref = {
            'position': position_ref,
            'velocity': velocity_ref,
            'yb': yb_ref,
        }
        if self._quat_interp:
            quat_ref = []
            for tau in times:
                quat = np.array([f(tau) for f in self._quat_interp])
                norm = np.linalg.norm(quat)
                if norm > 1e-6:
                    quat = quat / norm
                quat_ref.append(quat)
            ref['quaternion'] = np.vstack(quat_ref)

        return ref


def get_trajectory_path() -> str:
    """
    Get the default trajectory file path (v14-alpha.csv).

    Returns:
        Path to the latest planner output (fallback to v14-alpha.csv)
    """
    planning_csv = _find_planning_csv()
    if planning_csv:
        return str(planning_csv)
    return str(Path(__file__).parent / 'refs' / 'v14-alpha.csv')
