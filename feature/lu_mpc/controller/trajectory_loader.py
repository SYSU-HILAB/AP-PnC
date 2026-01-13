# -*- coding: future_fstrings -*-
#
# Trajectory Loader Module
# Created on 2025-01-10
#
# This module provides utilities for loading and parsing trajectory CSV files
# for use with the nMPC controller.
#

import numpy as np
from pathlib import Path
from typing import Dict, Optional, Tuple
from scipy.interpolate import interp1d


def load_trajectory_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """
    Load trajectory from v14-alpha.csv format.

    Expected CSV columns:
    time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,thrust,alpha,aero_force_z,
    yb_x,yb_y,yb_z,ap_x,ap_y,ap_z,S_ap_x,S_ap_z,B_Omega_b_y,alpha_dot,B_aero_force_z

    Args:
        csv_path: Path to the CSV file

    Returns:
        Dictionary with trajectory data:
        - time: (N,) array
        - position: (N, 3) array [pos_x, pos_y, pos_z]
        - velocity: (N, 3) array [vel_x, vel_y, vel_z]
        - yb: (N, 3) array [yb_x, yb_y, yb_z] (Y-axis body frame)
        - thrust: (N,) array
        - alpha: (N,) array
    """
    # Load CSV
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)

    # Extract columns (0-indexed)
    # 0:time, 1:pos_x, 2:pos_y, 3:pos_z, 4:vel_x, 5:vel_y, 6:vel_z,
    # 7:thrust, 8:alpha, ..., 11:yb_x, 12:yb_y, 13:yb_z
    trajectory = {
        'time': data[:, 0],
        'position': data[:, 1:4],   # pos_x, pos_y, pos_z
        'velocity': data[:, 4:7],   # vel_x, vel_y, vel_z
        'yb': data[:, 11:14],       # yb_x, yb_y, yb_z
        'thrust': data[:, 7],
        'alpha': data[:, 8],
    }

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
        return {
            'position': np.array([f(t) for f in self._pos_interp]),
            'velocity': np.array([f(t) for f in self._vel_interp]),
            'yb': np.array([f(t) for f in self._yb_interp]),
            'alpha': float(self._alpha_interp(t)),
        }

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

        position_ref = np.array([[f(t) for f in self._pos_interp] for t in times])
        velocity_ref = np.array([[f(t) for f in self._vel_interp] for t in times])
        yb_ref = np.array([[f(t) for f in self._yb_interp] for t in times])

        return {
            'position': position_ref,
            'velocity': velocity_ref,
            'yb': yb_ref,
        }


def get_trajectory_path() -> str:
    """
    Get the default trajectory file path (v14-alpha.csv).

    Returns:
        Path to v14-alpha.csv
    """
    return str(Path(__file__).parent / 'refs' / 'v14-alpha.csv')
