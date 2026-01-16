import numpy as np
from pathlib import Path

from acados_template import AcadosOcpSolver

from feature.lu_mpc.controller.core.config import MpcConfig
from feature.lu_mpc.core.ocp_setup import ocp_nls_cfg
from .trajectory_loader import TrajectoryReference, load_trajectory_csv

VELOCITY_EPSILON = 0.1


class ProposedNmpcController:
    """
    NMPC-based trajectory tracking controller for tailsitter UAV.

    Uses the NMPC formulation from src/nmpc with:
    - 18D state: [p(3), v(3), r(3), q(4), u(4), cz(1)]
    - 4D control: motor thrust rates (du)
    - Reference tracking for position, velocity, and YB orientation
    """

    def __init__(
        self,
        trajectory: TrajectoryReference,
        ctrl_dt: float | None = None,
    ):
        """
        Initialize the NMPC controller.

        Args:
            trajectory: TrajectoryReference object
            config: MpcConfig instance (uses default if None)
            ctrl_dt: Control time step (uses config.control.ctrl_dt if None)
        """
        self.trajectory = trajectory
        self.config = MpcConfig.create_high_performance()
        self.ctrl_dt = ctrl_dt if ctrl_dt is not None else self.config.control.ctrl_dt

        # Create OCP
        print("Creating NMPC OCP...")
        ocp_cfg = ocp_nls_cfg(self.config)

        # Create solver
        json_path = Path(__file__).parent.parent / ".cache" / "acados_ocp_proposed_nmpc.json"
        json_path.parent.mkdir(exist_ok=True)
        self.solver = AcadosOcpSolver(ocp_cfg, json_file=str(json_path))

        # Solver dimensions
        self.nx = ocp_cfg.model.x.size()[0]  # 18 (including cz_state) pvrq(13) thrust_forces(4)
        self.nu = ocp_cfg.model.u.size()[0]  # 4

        # Timing
        self.N = self.config.nodes
        self.horizon = self.config.T_horizon

        # State storage
        self._cz_state = 0.0
        self._cur_idx_ref = 0
        self._sim_dt = 0.02  # Trajectory sampling rate (const, 50Hz)

        print(f"NMPC Controller initialized:")
        print(f"  State dim: {self.nx}")
        print(f"  Control dim: {self.nu}")
        print(f"  Horizon: {self.N} nodes, {self.horizon}s")

    def compute_control(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Compute control input for current state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            Motor thrust rates (du) as (4,) array
        """
        # Map sim state to MPC state (with cz_state)
        mpc_state = self._est_mpc_state(sim_state)

        # Set parameter [cz_rate]
        params = np.array([self.config.cz_rate])
        for i in range(self.N + 1):
            self.solver.set(i, "p", params)

        # Initialize with current state as guess
        self.solver.set(0, "x", mpc_state)

        # Set reference trajectory
        # self._add_refecence_hover()
        self._set_reference()

        # Set initial state
        self.solver.set(0, "lbx", mpc_state)
        self.solver.set(0, "ubx", mpc_state)

        # Solve OCP
        status = self.solver.solve()

        if status != 0:
            print(f"Warning: NMPC solver failed with status {status}")
            return np.zeros(self.nu)

        # Get control input
        u = self.solver.get(0, "u")

        # Update cz_state estimate
        x_next = self.solver.get(1, "x")
        self._cz_state = x_next[-1]

        # Increment reference index after solving
        total_steps = len(self.trajectory.time)
        self._cur_idx_ref = (self._cur_idx_ref + 1) % total_steps

        return u

    def _est_mpc_state(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Map GD simulator state (17D) to NMPC state (18D).

        Args:
            sim_state: [p(3), v(3), r(3), q(4), u(4)] = 17D

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
        if V_norm > VELOCITY_EPSILON:
            # Use previous estimate or compute from dynamics
            cz = self._cz_state
        else:
            cz = 0.0
            self._cz_state = 0.0

        # Assemble MPC state
        # -3.00000e+01  3.67394e-15  0.00000e+00  p
        # 0.00000e+00  0.00000e+00   0.00000e+00 v
        # 0.00000e+00  0.00000e+00  0.00000e+00  r
        # 1.00000e+00  0.00000e+00  0.00000e+00  0.00000e+00  q
        # 2.45000e+00  2.45000e+00  2.45000e+00  2.45000e+00  0.00000e+00  f_thr+cz
        mpc_state = np.concatenate([p, v, r, q, u, [cz]])

        return mpc_state

    def _set_reference(self):
        """Set reference trajectory for prediction horizon using current index."""
        # Get reference window using current index
        ref = self.trajectory.get_reference_window(
            cur_idx_ref=self._cur_idx_ref,
            sample_dt=self._sim_dt,
            ctrl_dt=self.ctrl_dt,
            N=self.N
        )

        # Set reference for each stage
        for i in range(self.N):
            # Extract reference
            p_ref = ref['position'][i]
            v_ref = ref['velocity'][i]
            yb_ref = ref['yb'][i]

            # NMPC expects y_ref = [p, v, r, yb, u]
            # r (angular rate) reference is zeros
            # u (control) reference is zeros
            y_ref = np.concatenate([
                p_ref,   # 3
                v_ref,   # 3
                yb_ref,  # 3
                np.zeros(4),  # u_ref
            ])

            self.solver.set(i, "yref", y_ref)
        p_ref_N = ref['position'][-1]
        v_ref_N = ref['velocity'][-1]
        yb_ref_N = ref['yb'][-1]

        y_ref_N = np.concatenate([
            p_ref_N,
            v_ref_N,
            yb_ref_N,
        ])
        self.solver.set(self.N, "yref", y_ref_N)


    def _add_refecence_hover(self):
        """Set a constant hover reference using the first trajectory sample."""
        p_ref = self.trajectory.position[0]
        v_ref = self.trajectory.velocity[0]
        yb_ref = self.trajectory.yb[0]

        y_ref = np.concatenate([
            p_ref,
            v_ref,
            np.zeros(3),
            yb_ref,
            np.zeros(4),
        ])

        for i in range(self.N):
            self.solver.set(i, "yref", y_ref)

        y_ref_term = np.concatenate([
            p_ref,
            v_ref,
            np.zeros(3),
            yb_ref,
        ])
        self.solver.set(self.N, "yref", y_ref_term)

    def reset(self):
        """Reset controller state."""
        self._cz_state = 0.0
        self._cur_idx_ref = 0
        self.solver.reset()