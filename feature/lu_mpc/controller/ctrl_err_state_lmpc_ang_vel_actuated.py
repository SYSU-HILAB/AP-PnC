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

        print(f"Error-State MPC Controller initialized:")
        print(f"  State dim: {self.nx} (error state)")
        print(f"  Control dim: {self.nu} (control increments)")
        print(f"  Param dim: {self.np} (time-varying)")
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
        q_ref = ref_state["quat_quad_flu"]
        # Position error (world frame)
        err_p = p_ref - p_sim

        err_v = v_ref - v_sim

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
        q_ref = ref_state['quat_quad_flu']
        # Get rotation matrix from reference quaternion
        R_ref = quat_to_rotmat(q_ref)

        # 1. v_b_d: Reference velocity in body frame (3)
        v_b_d = R_ref.T @ v_ref

        # 2. R_d: Reference rotation matrix flattened (9)
        R_d = R_ref.flatten()

        # For now, use a nominal value - this should be computed from trajectory dynamics
        aT_d = np.array([9.81])  

        # 3. omega_d: Reference angular velocity (3)
        # For now, use zeros - ideally this would come from the trajectory
        omega_d = np.zeros(3)

        # 4. f_a_d: Reference aerodynamic force (3)
        # For now, use zeros - this requires GD model evaluation
        f_a_d = np.zeros(3)

        # 5. par_f_a_par_v_b_d: Jacobian of aero force w.r.t. velocity (9)
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
                    "quat_quad_flu": ref["quat_quad_flu"][i],
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
        euler_state = err_state[6:9]
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

        #<HARD
        #Task. this mpc cares about pos, vel, attitude error only
        #Task. plot err_state use 3 figures, attitude use euler angles to represent. 200 steps
        #Task. plot motor thrusts as well 4x1 subplot.
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
            import time
            time.sleep(20.0)
            print("HARD_CODE: Saved error state plots to /tmp/hard_err_state_plots.png")
            plt.close('all')
            self._hard_step_counter = 0
            self._hard_err_history_pos = []
            self._hard_err_history_vel = []
            self._hard_err_history_att = []
            self._hard_motor_history = []
        #>


        # Increment reference index after solving (only in tracking mode)
        if not self._is_hover_task:
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
