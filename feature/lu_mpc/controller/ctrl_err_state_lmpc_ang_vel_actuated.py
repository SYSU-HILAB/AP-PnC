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
    quat_conjugate,
)


VELOCITY_EPSILON = 0.1


def quat_to_err_theta(q_ref: np.ndarray, q_curr: np.ndarray) -> np.ndarray:
    """
    Compute angular error between two quaternions using small-angle approximation.

    Args:
        q_ref: Reference quaternion [w, x, y, z]
        q_curr: Current quaternion [w, x, y, z]


    Returns:
        Angular error vector [theta_x, theta_y, theta_z]
    """
    # q_err = q_curr_conj * q_ref
    q_err = quat_multiply(quat_conjugate(q_curr), q_ref)

    # Small angle approximation: theta ≈ 2 * q_xyz
    if q_err[0] < 0:
        q_err = -q_err
    
    delta_w = q_err[0]
    delta_v = q_err[1:4]

    norm_delta_v = np.linalg.norm(delta_v)
    if norm_delta_v < 1e-1:
        return 2.0 * delta_v
    else:
        return 2.0 * np.arctan2(norm_delta_v, delta_w) * (delta_v / norm_delta_v)





class ErrStateLmpcAngVelActuatedController:
    """
    Angular-velocity-actuated error-state linear MPC controller for trajectory tracking.

    Uses linearized error dynamics with:
    - 9D error state: [err_p(3), err_v(3), err_theta(3)]
    - 4D control increments: [increment_aT(1), increment_omega(3)]
    - 28D time-varying parameters from reference trajectory

    The controller minimizes the error between current state and reference,
    making it well-suited for precise trajectory tracking.
    """

    def __init__(self, config: ErrStateLmpcAngVelActuatedCfg):
        """
        Initialize the ErrStateLmpcAngVelActuatedController.

        Args:
            config: Unified configuration containing all controller settings.
        """
        # Extract from config
        self.trajectory = config.trajectory
        self._is_hover_task = config.is_hover_task


        self.mpc_config = config.mpc_config
        self.ctrl_dt = self.mpc_config.ctrl_dt

        # Create OCP with error state model
        ocp_cfg = ocp_err_state_cfg(self.mpc_config)
        # Create solver
        json_path = config.json_path
        json_path.parent.mkdir(exist_ok=True)
        self.solver = AcadosOcpSolver(ocp_cfg, json_file=str(json_path))

        # Solver dimensions
        self.nx = ocp_cfg.model.x.size()[0]  # 9 (error state)
        self.nu = ocp_cfg.model.u.size()[0]  # 4 (control increments)
        self.np = ocp_cfg.model.p.size()[0]  # 28 (time-varying parameters)

        # Timing
        self.N = self.mpc_config.nodes
        self.horizon = self.mpc_config.T

        # Create sub-controllers
        self._pd_thrust = PDController(config.pd_config)
        self._pid_ang_vel = PIDAntiWindupController(config.pid_config)
        self._qp_mixer = QpMixer(config.qp_mixer_config)

        # State storage
        self._cur_idx_ref = 0
        self._sim_dt = 0.02  # Trajectory sampling rate (const, 50Hz)
        self._u_prev = np.full(4, 9.81 * 2.0 / 4.0)  # Previous motor commands for QP mixer

        # Set reference (zero for error-state MPC) - only needs to be set once
        self._set_reference_only_once()

        print(f"Error-State MPC Controller initialized:")
        print(f"  State dim: {self.nx} (error state)")
        print(f"  Control dim: {self.nu} (control increments)")
        print(f"  Param dim: {self.np} (28 time-varying)")
        print(f"  Horizon: {self.N} nodes, {self.horizon}s")
        print(f"  Mode: {'Hover (fixed reference)' if self._is_hover_task else 'Tracking (advancing reference)'}")

    def reset(self):
        """Reset controller state."""
        self._cur_idx_ref = 0
        self._u_prev = np.full(4, 9.81 * 2.0 / 4.0)  # Hover thrust
        self._pid_ang_vel.reset()
        self.solver.reset()

    def _compute_error_state(
        self,
        ref_state: dict[str, np.ndarray],
        sim_state: np.ndarray
    ) -> np.ndarray:
        """
        Compute error state from simulation state and reference state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]
            ref_state: Reference state dictionary with 'position', 'velocity', 'quaternion'

        Returns:
            err_state: 9D error state [err_p(3), err_v(3), err_theta(3)]
        """
        # Extract reference state components
        p_ref = ref_state["position"]
        v_ref = ref_state["velocity"]
        q_ref = ref_state["quat_quad_flu"]
        # Position error (world frame)

        p_sim = sim_state[0:3]
        v_sim = sim_state[3:6]
        q_sim = sim_state[9:13]

        err_p = p_ref - p_sim
        err_v = v_ref - v_sim
        err_p = p_sim - p_ref
        err_v = v_sim - v_ref
        err_theta = quat_to_err_theta(q_ref=q_ref, q_curr=q_sim)

        # Assemble error state
        err_state = np.concatenate([err_p, err_v, err_theta])

        return err_state

    def _compute_reference_parameters(
        self,
        ref_state: dict[str, np.ndarray],
    ) -> np.ndarray:
        """
        Compute the 28 time-varying parameters from reference state.

        Order matches export_error_state_model():
        [R_d(9), mass_normalized_par_f_a_w_par_v_w_d(9), aT_d(1), mass_normalized_f_a_w_d(3), v_d(3), omega_d(3)]

        Args:
            ref_state: Reference state dictionary

        Returns:
            params: 28D parameter vector
        """
        # Extract reference state
        p_ref = ref_state["position"]
        v_ref = ref_state["velocity"]
        q_ref = ref_state['quat_quad_flu']
        # Get rotation matrix from reference quaternion
        R_ref = quat_to_rotmat(q_ref)

        # 1. R_d: Reference rotation matrix flattened (9)
        R_d = R_ref.flatten()

        # 2. mass_normalized_par_f_a_w_par_v_w_d: Jacobian of aero force w.r.t. velocity in world frame (9)
        # For now, use zeros - this requires analytical or numerical derivatives
        mass_normalized_par_f_a_w_par_v_w_d = np.zeros(9)

        # 3. aT_d: Reference thrust acceleration magnitude (1) - SCALAR
        # For now, use a nominal value - this should be computed from trajectory dynamics
        aT_d = 9.81  # Scalar thrust acceleration

        # 4. mass_normalized_f_a_w_d: Reference aerodynamic force in world frame (3)
        # For now, use zeros - this requires GD model evaluation
        mass_normalized_f_a_w_d = np.zeros(3)

        # 5. v_d: Reference velocity in world frame (3)
        v_d = v_ref

        # 6. omega_d: Reference angular velocity (3)
        # For now, use zeros - ideally this would come from the trajectory
        omega_d = np.zeros(3)
        # omega_d = ang_vel_meas

        # Assemble parameters in order matching export_error_state_model()
        params = np.concatenate([
            R_d,                                   # 0-8:   9 elements
            mass_normalized_par_f_a_w_par_v_w_d,  # 9-17:  9 elements
            [aT_d],                               # 18:    1 element (scalar as array)
            mass_normalized_f_a_w_d,              # 19-21: 3 elements
            v_d,                                  # 22-24: 3 elements
            omega_d,                              # 25-27: 3 elements
        ])  # Total: 28

        return params

    def _set_reference_only_once(self):
        """Set reference trajectory for prediction horizon (zero for error-state MPC)."""
        ny_path = self.nx + self.nu  # 13 for path stages
        ny_terminal = self.nx        # 9 for terminal stage

        y_ref_path = np.zeros(ny_path)
        y_ref_terminal = np.zeros(ny_terminal)

        for i in range(self.N):
            self.solver.set(i, "yref", y_ref_path)

        self.solver.set(self.N, "yref", y_ref_terminal)

    def _get_error_state(self, sim_state: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        """
        Get reference state and compute error state from simulation state.

        Combines reference extraction and error state computation into a single step.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            err_state: 9D error state [err_p(3), err_v(3), err_theta(3)]
            ref_state: Reference state dictionary
        """
        # Get reference at current index (hover uses index 0, tracking advances)
        cur_idx_ref = 0 if self._is_hover_task else self._cur_idx_ref
        ref_window = self.trajectory.get_reference_window(
            cur_idx_ref=cur_idx_ref,
            ctrl_dt=self.ctrl_dt,
            N=0  # Single step
        )

        # Extract first (only) element from window
        ref_state = {
            "position": ref_window["position"][0],
            "velocity": ref_window["velocity"][0],
            "quat_quad_flu": ref_window["quat_quad_flu"][0],
        }

        # Compute error state
        err_state = self._compute_error_state(ref_state=ref_state, sim_state=sim_state)

        return err_state, ref_state

    def _set_shooting_0_x0(self, state: np.ndarray) -> None:
        """Set shooting node 0 initial state, lower bound, and upper bound."""
        self.solver.set(0, "x", state)
        self.solver.set(0, "lbx", state)
        self.solver.set(0, "ubx", state)

    def _set_parameters_for_horizon(self):
        """Set time-varying parameters for all stages in the MPC horizon."""
        # Get reference window - hover mode uses index 0, tracking mode advances
        cur_idx_ref = 0 if self._is_hover_task else self._cur_idx_ref
        N = 0 if self._is_hover_task else self.N

        ref_window = self.trajectory.get_reference_window(
            cur_idx_ref=cur_idx_ref,
            ctrl_dt=self.ctrl_dt,
            N=N
        )
        if self._is_hover_task:
            # Repeat the single reference for all stages
            for key in ref_window:
                ref_window[key] = np.tile(
                    ref_window[key],
                    (self.N + 1, 1),
                )

        # Set parameters for each stage based on reference window
        for i in range(self.N + 1):
            ref_state_i = {
                "position": ref_window["position"][i],
                "velocity": ref_window["velocity"][i],
                "quat_quad_flu": ref_window["quat_quad_flu"][i],
            }
            params = self._compute_reference_parameters(ref_state_i)
            self.solver.set(i, "p", params)

    def _low_level_control(
        self,
        u: np.ndarray,
        ref_state: dict[str, np.ndarray],
        sim_state: np.ndarray,
    ) -> np.ndarray:
        """
        Low-level control: extract increments, compute reference values, and track with sub-controllers.

        Steps:
        - Extract control increments from MPC solution
        - Compute reference values from parameters
        - Convert increments to absolute commands
        - PD controller for thrust tracking
        - PID controller for angular velocity tracking
        - QP mixer: convert [tot_thrust, torque] to motor thrusts (4D)

        Args:
            u: MPC solution control vector [increment_aT, increment_omega_x, increment_omega_y, increment_omega_z]
            ref_state: Reference state dictionary
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            motor_thrusts: Motor thrusts (4D)
        """
        # Extract control increments from MPC solution
        increment_aT = u[0]
        increment_omega = u[1:4]

        # Compute reference values from parameters
        params = self._compute_reference_parameters(ref_state)
        aT_ref = params[18]          # Index 18 (scalar thrust acceleration)
        omega_ref = params[25:28]    # Indices 25, 26, 27 (angular velocity)

        # Convert increments to absolute commands
        aT_cmd = aT_ref - increment_aT  # Scalar thrust acceleration
        omega_cmd = omega_ref - increment_omega

        # Get measurements
        ang_vel_meas = sim_state[6:9]  # Angular rates
        current_motor_thrusts = sim_state[13:17]

        # PD controller for thrust tracking
        current_aT = np.sum(current_motor_thrusts) / 2.0  # Approximate current thrust
        thrust_correction = self._pd_thrust.compute(aT_cmd, current_aT)
        tot_thrust_cmd = aT_cmd * 2.0 + thrust_correction  # Scale to total thrust

        # PID controller for angular velocity tracking
        torque_cmd = self._pid_ang_vel.compute(omega_cmd, ang_vel_meas)

        # QP mixer: convert [tot_thrust, 3d_torque] to motor thrusts (4D)
        tau_desired = np.concatenate([[tot_thrust_cmd], torque_cmd])
        motor_thrusts, motor_rates, info = self._qp_mixer.mix(tau_desired, self._u_prev)

        # Update previous command for QP mixer
        self._u_prev = motor_thrusts.copy()

        return motor_thrusts

    def compute_control(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Compute motor commands from current state.

        Error-state MPC compute procedure:
        1. Initialize solver: get reference, compute error state, set parameters and initial condition
        2. Solve OCP
        3. Low-level control: extract increments, compute reference values, and track with sub-controllers
        4. Update reference index for tracking mode

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            Motor thrusts (4D)
        """
        # 1. Initialize solver: get reference (9D), compute error state, set parameters and initial condition
        sim_state[0] = sim_state[0] - 0.2  # Altitude adjustment
        err_state, ref_state = self._get_error_state(sim_state)
        self._set_parameters_for_horizon()
        self._set_shooting_0_x0(err_state)

        # 2. Solve OCP
        status = self.solver.solve()

        if status != 0:
            raise ValueError(f"Error: Error-state MPC solver failed with status {status}")

        # 3. Low-level control: extract increments, compute reference values, and track with sub-controllers
        u = self.solver.get(0, "u")  # [increment_aT, increment_omega_x, increment_omega_y, increment_omega_z]
        motor_thrusts = self._low_level_control(u, ref_state, sim_state)

        #<HARD
        #Task. this mpc cares about pos, vel, attitude error only
        #Task. plot err_state use 3 figures, attitude use euler angles to represent. 200 steps
        #Task. plot motor thrusts as well 4x1 subplot.
        #Task. plot body_rates trakcing as well 3x1 subplot.
        import matplotlib.pyplot as plt
        from scipy.spatial.transform import Rotation as R
        if not hasattr(self, '_hard_step_counter'):
            self._hard_step_counter = 0
            self._hard_err_history_pos = []
            self._hard_err_history_vel = []
            self._hard_err_history_att = []
            self._hard_motor_history = []
        self._hard_step_counter += 1
        self._hard_err_history_pos.append(err_state[0:3])
        self._hard_err_history_vel.append(err_state[3:6])
        euler_err = err_state[6:9]
        self._hard_err_history_att.append(euler_err)
        self._hard_motor_history.append(motor_thrusts.copy())
        if self._hard_step_counter >= 500:
            steps = np.arange(500)
            pos_hist = np.array(self._hard_err_history_pos)
            vel_hist = np.array(self._hard_err_history_vel)
            att_hist = np.array(self._hard_err_history_att)
            motor_hist = np.array(self._hard_motor_history)
            fig1, axes1 = plt.subplots(3, 1, figsize=(10, 12))
            axes1[0].plot(steps, pos_hist[:, 0], label='x error')
            axes1[0].plot(steps, pos_hist[:, 1], label='y error')
            axes1[0].plot(steps, pos_hist[:, 2], label='z error')
            axes1[0].set_title('Position Error')
            axes1[0].set_xlabel('Step')
            axes1[0].set_ylabel('Error (m)')
            axes1[0].legend()
            axes1[0].grid(True)
            axes1[1].plot(steps, vel_hist[:, 0], label='vx error')
            axes1[1].plot(steps, vel_hist[:, 1], label='vy error')
            axes1[1].plot(steps, vel_hist[:, 2], label='vz error')
            axes1[1].set_title('Velocity Error')
            axes1[1].set_xlabel('Step')
            axes1[1].set_ylabel('Error (m/s)')
            axes1[1].legend()
            axes1[1].grid(True)
            axes1[2].plot(steps, att_hist[:, 0], label='roll error (phi)')
            axes1[2].plot(steps, att_hist[:, 1], label='pitch error (theta)')
            axes1[2].plot(steps, att_hist[:, 2], label='yaw error (psi)')
            axes1[2].set_title('Attitude Error (Euler Angles)')
            axes1[2].set_xlabel('Step')
            axes1[2].set_ylabel('Error (rad)')
            axes1[2].legend()
            axes1[2].grid(True)
            plt.tight_layout()
            plt.show()
            fig2, axes2 = plt.subplots(4, 1, figsize=(10, 12))
            axes2[0].plot(steps, motor_hist[:, 0], label='motor 1')
            axes2[0].set_title('Motor 1 Thrust')
            axes2[0].set_xlabel('Step')
            axes2[0].set_ylabel('Thrust (N)')
            axes2[0].legend()
            axes2[0].set_ylim([0,8])
            axes2[0].grid(True)
            axes2[1].plot(steps, motor_hist[:, 1], label='motor 2')
            axes2[1].set_title('Motor 2 Thrust')
            axes2[1].set_xlabel('Step')
            axes2[1].set_ylabel('Thrust (N)')
            axes2[1].legend()
            axes2[1].set_ylim([0,8])
            axes2[1].grid(True)
            axes2[2].plot(steps, motor_hist[:, 2], label='motor 3')
            axes2[2].set_title('Motor 3 Thrust')
            axes2[2].set_xlabel('Step')
            axes2[2].set_ylabel('Thrust (N)')
            axes2[2].legend()
            axes2[2].set_ylim([0,8])
            axes2[2].grid(True)
            axes2[3].plot(steps, motor_hist[:, 3], label='motor 4')
            axes2[3].set_title('Motor 4 Thrust')
            axes2[3].set_xlabel('Step')
            axes2[3].set_ylabel('Thrust (N)')
            axes2[3].set_ylim([0,8])
            axes2[3].legend()
            axes2[3].grid(True)
            plt.tight_layout()
            plt.show()
            import sys
            sys.exit(0)
        #>

        # 4. Update reference index for tracking mode
        total_steps = len(self.trajectory.time)
        self._cur_idx_ref = (self._cur_idx_ref + 1) % total_steps

        return motor_thrusts

    @property
    def is_trajectory_complete(self) -> bool:
        """Check if trajectory tracking is complete."""
        if self._is_hover_task:
            return False
        total_steps = len(self.trajectory.time)
        return self._cur_idx_ref >= total_steps - 1
