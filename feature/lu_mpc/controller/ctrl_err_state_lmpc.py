"""Error-state MPC controller for trajectory tracking."""

from __future__ import annotations

import numpy as np
from pathlib import Path

from acados_template import AcadosOcpSolver

from .config.ocp_err_state_cfg import ocp_err_state_cfg, ErrorStateLinearMpcConfig
from .config.ctrl_err_state_lmpc_cfg import ErrStateLmpcAngVelActuatedCfg
from .sub_controllers import (
    PDController,
    PIDAntiWindupController,
    QpMixer,
)
from .trajectory_loader import TrajectoryReference
from ...utils.math_utils import (
    quat_multiply,
    quat_to_rotmat,
)


VELOCITY_EPSILON = 0.1


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate of a quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_err_theta(q_curr: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    """
    Compute angular error between two quaternions using small-angle approximation.

    Args:
        q_curr: Current quaternion [w, x, y, z]
        q_ref: Reference quaternion [w, x, y, z]

    Returns:
        Angular error vector [theta_x, theta_y, theta_z]
    """
    # q_err = q_ref^* * q_curr
    q_err = quat_multiply(quat_conjugate(q_ref), q_curr)

    # Small angle approximation: theta ≈ 2 * q_xyz
    return 2.0 * q_err[1:4]


class ErrStateLmpcController:
    """
    Error-state MPC controller for trajectory tracking.

    Uses linearized error dynamics with:
    - 9D error state: [err_p(3), err_v(3), err_theta(3)]
    - 4D control increments: [increment_aT(1), increment_omega(3)]
    - 28D time-varying parameters from reference trajectory

    The controller minimizes the error between current state and reference,
    making it well-suited for precise trajectory tracking.
    """

    def __init__(self, config: ErrStateLmpcAngVelActuatedCfg):
        """
        Initialize the error-state MPC controller.

        Args:
            config: Unified configuration containing all controller settings.
        """
        # Extract from config
        self.trajectory = config.trajectory
        self._is_hover_task = config.is_hover_task

        # MPC configuration
        mpc_config = config.mpc_config or ErrorStateLinearMpcConfig.create_default()
        self.config = mpc_config
        self.ctrl_dt = mpc_config.ctrl_dt

        # Create OCP with error state model
        ocp_cfg = ocp_err_state_cfg(mpc_config)

        # Create solver
        json_path = config.json_path
        json_path.parent.mkdir(exist_ok=True)
        self.solver = AcadosOcpSolver(ocp_cfg, json_file=str(json_path))

        # Solver dimensions
        self.nx = ocp_cfg.model.x.size()[0]  # 9 (error state)
        self.nu = ocp_cfg.model.u.size()[0]  # 4 (control increments)
        self.np = ocp_cfg.model.p.size()[0]  # 28 (time-varying parameters)

        # Timing
        self.N = mpc_config.nodes
        self.horizon = mpc_config.T_horizon

        # Create sub-controllers
        self._pd_thrust = PDController(config.pd_config)
        self._pid_ang_vel = PIDAntiWindupController(config.pid_config)
        self._qp_mixer = QpMixer(config.qp_mixer_config)

        # State storage
        self._cur_idx_ref = 0
        self._sim_dt = 0.02  # Trajectory sampling rate (const, 50Hz)
        self._u_prev = np.zeros(4)  # Previous motor commands for QP mixer

        print(f"Error-State MPC Controller initialized:")
        print(f"  State dim: {self.nx} (error state)")
        print(f"  Control dim: {self.nu} (control increments)")
        print(f"  Param dim: {self.np} (time-varying)")
        print(f"  Horizon: {self.N} nodes, {self.horizon}s")
        print(f"  Mode: {'Hover (fixed reference)' if self._is_hover_task else 'Tracking (advancing reference)'}")

    def reset(self):
        """Reset controller state."""
        self._cur_idx_ref = 0
        self._u_prev = np.zeros(4)
        self._pid_ang_vel.reset()
        self.solver.reset()

    def _compute_error_state(
        self,
        sim_state: np.ndarray,
        ref_state: dict[str, np.ndarray]
    ) -> np.ndarray:
        """
        Compute error state from simulation state and reference state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]
            ref_state: Reference state dictionary with 'position', 'velocity', 'quaternion'

        Returns:
            err_state: 9D error state [err_p(3), err_v(3), err_theta(3)]
        """
        # Extract simulation state components
        p_sim = sim_state[0:3]    # Position (world frame)
        v_sim = sim_state[3:6]    # Velocity (world frame)
        q_sim = sim_state[9:13]   # Quaternion [w, x, y, z]

        # Extract reference state components
        p_ref = ref_state["position"]
        v_ref = ref_state["velocity"]
        q_ref = ref_state.get("quaternion", np.array([1.0, 0.0, 0.0, 0.0]))  # Identity if not available

        # Position error (world frame)
        err_p = p_sim - p_ref

        # Velocity error - transform to body frame for more intuitive control
        # Get rotation matrix from reference quaternion
        R_ref = quat_to_rotmat(q_ref)
        v_sim_body = R_ref.T @ v_sim
        v_ref_body = R_ref.T @ v_ref
        err_v = v_sim_body - v_ref_body

        # Angular error using Log mapping (small-angle approximation)
        err_theta = quat_to_err_theta(q_sim, q_ref)

        # Assemble error state
        err_state = np.concatenate([err_p, err_v, err_theta])

        return err_state

    def _compute_reference_parameters(
        self,
        ref_state: dict[str, np.ndarray]
    ) -> np.ndarray:
        """
        Compute the 28 time-varying parameters from reference state.

        Args:
            ref_state: Reference state dictionary

        Returns:
            params: 28D parameter vector [v_b_d(3), R_d(9), aT_d(1), omega_d(3), f_a_d(3), par_f_a_par_v_b_d(9)]
        """
        # Extract reference state
        p_ref = ref_state["position"]
        v_ref = ref_state["velocity"]
        q_ref = ref_state.get("quaternion", np.array([1.0, 0.0, 0.0, 0.0]))

        # Get rotation matrix from reference quaternion
        R_ref = quat_to_rotmat(q_ref)

        # 1. v_b_d: Reference velocity in body frame (3)
        v_b_d = R_ref.T @ v_ref

        # 2. R_d: Reference rotation matrix flattened (9)
        R_d = R_ref.flatten()

        # For now, use a nominal value - this should be computed from trajectory dynamics
        aT_d = np.array([9.81])  

        # 4. omega_d: Reference angular velocity (3)
        # For now, use zeros - ideally this would come from the trajectory
        omega_d = np.zeros(3)

        # 5. f_a_d: Reference aerodynamic force (3)
        # For now, use zeros - this requires GD model evaluation
        f_a_d = np.zeros(3)

        # 6. par_f_a_par_v_b_d: Jacobian of aero force w.r.t. velocity (9)
        # For now, use zeros - this requires analytical or numerical derivatives
        par_f_a_par_v_b_d = np.zeros(9)

        # Assemble parameters
        params = np.concatenate([
            v_b_d,           # 3
            R_d,             # 9
            aT_d,            # 1
            omega_d,         # 3
            f_a_d,           # 3
            par_f_a_par_v_b_d,  # 9
        ])  # Total: 28

        return params

    def _set_reference(self):
        """Set reference trajectory for prediction horizon using current index."""
        # For error state MPC, the reference is always zero (goal is zero error)
        # Path stages (0 to N-1): cost output includes state and control (nx + nu = 13)
        # Terminal stage (N): cost output includes only state (nx = 9)

        ny_path = self.nx + self.nu  # 13 for path stages
        ny_terminal = self.nx        # 9 for terminal stage

        y_ref_path = np.zeros(ny_path)
        y_ref_terminal = np.zeros(ny_terminal)

        for i in range(self.N):
            self.solver.set(i, "yref", y_ref_path)

        self.solver.set(self.N, "yref", y_ref_terminal)

    def _set_parameters_for_stage(self, stage_idx: int, ref_state: dict[str, np.ndarray]):
        """
        Set time-varying parameters for a specific MPC stage.

        Args:
            stage_idx: Stage index (0 to N)
            ref_state: Reference state dictionary
        """
        params = self._compute_reference_parameters(ref_state)
        self.solver.set(stage_idx, "p", params)

    def _get_quaternion_at_stage(self, stage_idx: int) -> np.ndarray:
        """Get quaternion for a given MPC stage index."""
        if self.trajectory.quad_df_phi_quat is None:
            return np.array([1.0, 0.0, 0.0, 0.0])
        idx = (self._cur_idx_ref + stage_idx * int(self.ctrl_dt / self._sim_dt)) % len(
            self.trajectory.time
        )
        return self.trajectory.quad_df_phi_quat[idx]

    def _set_parameters_for_horizon(self):
        """Set time-varying parameters for all stages in the MPC horizon."""
        if self._is_hover_task:
            # Hover mode: all stages use same parameters (initial reference)
            ref = self.trajectory.get_reference_at_time(0.0)
            params = self._compute_reference_parameters(ref)
            for i in range(self.N + 1):
                self.solver.set(i, "p", params)
        else:
            # Tracking mode: each stage uses different reference parameters
            ref = self.trajectory.get_reference_window(
                cur_idx_ref=self._cur_idx_ref,
                ctrl_dt=self.ctrl_dt,
                N=self.N
            )

            # Set parameters for each stage based on reference window
            for i in range(self.N + 1):
                ref_state_i = {
                    "position": ref["position"][i],
                    "velocity": ref["velocity"][i],
                    "quaternion": self._get_quaternion_at_stage(i),
                }
                self._set_parameters_for_stage(i, ref_state_i)

    def compute_control(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Compute motor commands from current state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            Motor thrusts (4D)
        """
        # Get reference at current index
        if self._is_hover_task:
            ref_state = self.trajectory.get_reference_at_time(0.0)
        else:
            t_current = self._cur_idx_ref * self._sim_dt
            ref_state = self.trajectory.get_reference_at_time(t_current)

        # Compute error state
        err_state = self._compute_error_state(sim_state, ref_state)

        # Set time-varying parameters for all stages
        self._set_parameters_for_horizon()

        # Set initial error state
        self.solver.set(0, "x", err_state)
        self.solver.set(0, "lbx", err_state)
        self.solver.set(0, "ubx", err_state)

        # Set reference (zero for error state)
        self._set_reference()

        # Solve OCP
        status = self.solver.solve()

        if status != 0:
            print(f"Warning: Error-state MPC solver failed with status {status}")
            return np.zeros(4)

        # Get control increments from MPC
        u = self.solver.get(0, "u")  # [increment_aT, increment_omega_x, increment_omega_y, increment_omega_z]
        increment_aT = u[0]
        increment_omega = u[1:4]

        # Extract reference values from parameters
        # params = [v_b_d(3), R_d(9), aT_d(1), omega_d(3), f_a_d(3), par_f_a_par_v_b_d(9)]
        params = self._compute_reference_parameters(ref_state)
        aT_ref = params[12]      # Index 12 (after 3+9=12)
        omega_ref = params[13:16] # Indices 13, 14, 15

        # Convert to absolute commands
        aT_cmd = aT_ref + increment_aT
        omega_cmd = omega_ref + increment_omega

        # Get current measurements from sim state
        ang_vel_meas = sim_state[6:9]  # Angular rates

        # PD controller for thrust tracking
        # Note: Current aT is estimated from motor thrusts
        current_motor_thrusts = sim_state[13:17]
        current_aT = np.sum(current_motor_thrusts) / 2.0  # Approximate
        thrust_correction = self._pd_thrust.compute(aT_cmd, current_aT)
        tot_thrust_cmd = aT_cmd * 2.0 + thrust_correction  # Scale to total thrust

        # PID controller for angular velocity tracking
        torque_cmd = self._pid_ang_vel.compute(omega_cmd, ang_vel_meas)

        # QP mixer: convert [tot_thrust, 3d_torque] to motor thrusts (4D)
        tau_desired = np.concatenate([[tot_thrust_cmd], torque_cmd])
        motor_thrusts, motor_rates, info = self._qp_mixer.mix(tau_desired, self._u_prev)

        # Update previous command
        self._u_prev = motor_thrusts.copy()

        # Increment reference index after solving (only in tracking mode)
        if not self._is_hover_task:
            total_steps = len(self.trajectory.time)
            self._cur_idx_ref = (self._cur_idx_ref + 1) % total_steps

        return motor_thrusts
