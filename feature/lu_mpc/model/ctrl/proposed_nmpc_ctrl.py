# -*- coding: future_fstrings -*-
#
# Proposed nMPC Controller Module
# Created on 2025-01-10
#
# This module implements a trajectory tracking controller using the nMPC
# from src/nmpc/model.py and src/nmpc/config.py.
#

import numpy as np
from pathlib import Path
from typing import Dict, Optional

from feature.lu_mpc.model.core import ControlConfig, MpcConfig, ocp_nls
from acados_template import AcadosOcpSolver

# Add trajectory loader to path
try:
    from .trajectory_loader import TrajectoryReference, load_trajectory_csv
except ImportError:
    from trajectory_loader import TrajectoryReference, load_trajectory_csv


class ProposedNmpcController:
    """
    nMPC-based trajectory tracking controller for tailsitter UAV.

    Uses the nMPC formulation from src/nmpc with:
    - 18D state: [p(3), v(3), r(3), q(4), u(4), cz(1)]
    - 4D control: motor thrust rates (du)
    - Reference tracking for position, velocity, and YB orientation
    """

    def __init__(
        self,
        trajectory: TrajectoryReference,
        config: Optional[MpcConfig] = None,
        control_dt: float = None,
    ):
        """
        Initialize the nMPC controller.

        Args:
            trajectory: TrajectoryReference object
            config: MpcConfig instance (uses default if None)
            control_dt: Control time step (uses ControlConfig.control_time if None)
        """
        self.trajectory = trajectory
        self.config = config if config is not None else MpcConfig()
        self.control_dt = control_dt if control_dt is not None else ControlConfig.control_time

        # Create OCP
        print("Creating nMPC OCP...")
        ocp = ocp_nls()

        # Create solver
        json_path = Path(__file__).parent.parent / '.cache' / 'acados_ocp_proposed_nmpc.json'
        json_path.parent.mkdir(exist_ok=True)
        self.solver = AcadosOcpSolver(ocp, json_file=str(json_path))

        # Solver dimensions
        self.nx = ocp.model.x.size()[0]  # 18 (including cz_state)
        self.nu = ocp.model.u.size()[0]  # 4

        # Timing
        self.N = self.config.nodes
        self.horizon = self.config.T_horizon

        # State storage
        self._cz_state = 0.0

        print(f"nMPC Controller initialized:")
        print(f"  State dim: {self.nx}")
        print(f"  Control dim: {self.nu}")
        print(f"  Horizon: {self.N} nodes, {self.horizon}s")

    def compute_control(self, sim_state: np.ndarray, current_time: float) -> np.ndarray:
        """
        Compute control input for current state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]
            current_time: Current simulation time

        Returns:
            Motor thrust rates (du) as (4,) array
        """
        # Map sim state to MPC state (with cz_state)
        mpc_state = self._map_sim_to_mpc_state(sim_state, current_time)

        # Set parameters [cz, cur_alpha, cz_rate]
        params = self._compute_parameters(sim_state)
        for i in range(self.N + 1):
            self.solver.set(i, "p", params)

        # Initialize with current state as guess
        for i in range(self.N + 1):
            self.solver.set(i, "x", mpc_state)
            if i < self.N:
                self.solver.set(i, "u", np.zeros(self.nu))

        # Set reference trajectory
        self._set_reference(current_time)

        # Set initial state
        self.solver.set(0, "lbx", mpc_state)
        self.solver.set(0, "ubx", mpc_state)

        # Solve OCP
        status = self.solver.solve()

        if status != 0:
            print(f"Warning: nMPC solver failed with status {status}")
            return np.zeros(self.nu)

        # Get control input
        u = self.solver.get(0, "u")

        # Update cz_state estimate
        x_next = self.solver.get(1, "x")
        self._cz_state = x_next[-1]

        return u

    def _map_sim_to_mpc_state(self, sim_state: np.ndarray, current_time: float) -> np.ndarray:
        """
        Map GD simulator state (17D) to nMPC state (18D).

        Args:
            sim_state: [p(3), v(3), r(3), q(4), u(4)] = 17D
            current_time: Current simulation time

        Returns:
            mpc_state: [p(3), v(3), r(3), q(4), u(4), cz(1)] = 18D
        """
        # Extract sim state components
        p = sim_state[0:3]    # Position
        v = sim_state[3:6]    # Velocity (world frame)
        r = sim_state[6:9]    # Angular rates
        q = sim_state[9:13]   # Quaternion
        u = sim_state[13:17]  # Motor thrusts

        # Estimate cz from current state: cz = -acc_b_z / V^2
        # For now, use the current cz_state estimate
        V_norm = np.linalg.norm(v)
        if V_norm > 0.1:
            # Use previous estimate or compute from dynamics
            cz = self._cz_state
        else:
            cz = 0.0

        # Assemble MPC state
        mpc_state = np.concatenate([p, v, r, q, u, [cz]])

        return mpc_state

    def _set_reference(self, current_time: float):
        """
        Set reference trajectory for prediction horizon.

        Args:
            current_time: Current simulation time
        """
        # Get reference window
        dt = self.horizon / self.N
        ref = self.trajectory.get_reference_window(current_time, self.horizon, dt)

        # Set reference for each stage
        for i in range(self.N):
            # Extract reference
            p_ref = ref['position'][i]
            v_ref = ref['velocity'][i]
            yb_ref = ref['yb'][i]

            # nMPC expects y_ref = [p, v, r, yb, u]
            # r (angular rate) reference is zeros
            # u (control) reference is zeros
            y_ref = np.concatenate([
                p_ref,   # 3
                v_ref,   # 3
                np.zeros(3),  # r_ref
                yb_ref,  # 3
                np.zeros(4),  # u_ref
            ])

            self.solver.set(i, "yref", y_ref)

        # Terminal reference
        p_ref_term = ref['position'][-1]
        v_ref_term = ref['velocity'][-1]
        yb_ref_term = ref['yb'][-1]
        y_ref_term = np.concatenate([
            p_ref_term,
            v_ref_term,
            np.zeros(3),
            yb_ref_term,
        ])
        self.solver.set(self.N, "yref", y_ref_term)

    def _compute_parameters(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Compute nMPC parameters from current state.

        Args:
            sim_state: GD simulator state (17D)

        Returns:
            Parameters [cz, cur_alpha, cz_rate] as (3,) array
        """
        # Extract state
        v = sim_state[3:6]    # Velocity (world frame)
        q = sim_state[9:13]   # Quaternion

        # Compute body frame velocity
        # vb = q_inv_oproduct_vec(v, q)
        # Using numpy equivalent:
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        v_body = R.T @ v

        # Compute angle of attack: alpha = atan2(vx, vz)
        alpha = np.arctan2(v_body[0], v_body[2] + 0.1)

        # Parameters from config
        cz = self.config.cz[0] if hasattr(self.config, 'cz') else 0.0
        cz_rate = 1.0  # Default rate from model dynamics

        return np.array([cz, alpha, cz_rate])

    def reset(self):
        """Reset controller state."""
        self._cz_state = 0.0
        self.solver.reset()


def create_proposed_controller(
    trajectory_csv_path: str,
    config: Optional[MpcConfig] = None,
) -> ProposedNmpcController:
    """
    Factory function to create a proposed nMPC controller.

    Args:
        trajectory_csv_path: Path to trajectory CSV file
        config: Optional MpcConfig instance

    Returns:
        ProposedNmpcController instance
    """
    # Load trajectory
    trajectory_data = load_trajectory_csv(trajectory_csv_path)
    trajectory = TrajectoryReference(trajectory_data)

    # Create controller
    controller = ProposedNmpcController(trajectory, config)

    return controller


if __name__ == "__main__":
    # Test controller creation
    print("Testing proposed nMPC controller...")

    # Get trajectory path
    traj_path = Path(__file__).parent.parent / 'refs' / 'v14-alpha.csv'

    # Create controller
    controller = create_proposed_controller(str(traj_path))

    print("✅ Controller created successfully!")
