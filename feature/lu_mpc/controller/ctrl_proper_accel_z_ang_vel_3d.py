"""
Proper Acceleration Z and Angular Velocity 3D Controller for quadrotor trajectory tracking.

This module provides a controller that:
1. Wraps the ACADOS NMPC solver as a low-level handler
2. Extracts ang_vel_des and accel_des from x[1] (predicted state)
3. Computes proper_accel_des = accel_des - gravity
4. Runs PD controller for proper acceleration tracking → tot_thrust_cmd
5. Runs PID controller with anti-windup for angular velocity tracking → 3d_torque_cmd
6. Runs QP mixer for [tot_thrust, 3d_torque] tracking → motor_thrusts_cmd (4D)
"""

from __future__ import annotations

import casadi as cs
import numpy as np
from pathlib import Path

from acados_template import AcadosOcpSolver

from .config.ctrl_proper_accel_z_ang_vel_3d_cfg import (
    CtrlProperAccelZAngVel3DConfig,
    PDControllerConfig,
    PIDAntiWindupConfig,
    QpMixerConfig,
    DEFAULT_PD_CONFIG,
    DEFAULT_PID_CONFIG,
    DEFAULT_QP_MIXER_CONFIG,
)
from .config.ocp_proposed_mpc_cfg import ocp_proposed_mpc_cfg, MpcConfig
from .trajectory_loader import TrajectoryReference

VELOCITY_EPSILON = 0.1


# =============================================================================
# PD Controller (for proper acceleration tracking)
# =============================================================================

class PDController:
    """PD controller for tracking proper acceleration."""

    def __init__(self, config: PDControllerConfig):
        self._config = config
        self._Kp = config.Kp
        self._Kd = config.Kd
        self._output_min = config.output_min
        self._output_max = config.output_max

        # State
        self._prev_error = np.zeros(3)
        self._dt = 0.02  # Default control period

    def compute(
        self,
        desired: np.ndarray,
        measured: np.ndarray,
        dt: float | None = None,
    ) -> float:
        """
        Compute PD control output.

        Args:
            desired: Desired proper acceleration (3D)
            measured: Measured proper acceleration (3D)
            dt: Time step (uses default if None)

        Returns:
            Total thrust command (scalar)
        """
        if dt is None:
            dt = self._dt

        error = desired - measured

        # PD control law
        p_term = self._Kp * np.sum(error)  # Sum to get total thrust
        d_term = self._Kd * np.sum(error - self._prev_error) / dt

        output = p_term + d_term

        # Clamp output
        output = np.clip(output, self._output_min, self._output_max)

        # Update state
        self._prev_error = error.copy()
        self._dt = dt

        return output


# =============================================================================
# PID Controller with Anti-Windup (for angular velocity tracking)
# =============================================================================

class PIDAntiWindupController:
    """PID controller with anti-windup for tracking angular velocity."""

    def __init__(self, config: PIDAntiWindupConfig):
        self._config = config
        self._Kp = config.Kp
        self._Ki = config.Ki
        self._Kd = config.Kd
        self._integral_max = config.integral_max
        self._integral_min = config.integral_min
        self._output_min = config.output_min
        self._output_max = config.output_max
        self._dt = config.dt

        # State
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)

    def compute(
        self,
        desired: np.ndarray,
        measured: np.ndarray,
        dt: float | None = None,
    ) -> np.ndarray:
        """
        Compute PID control output with anti-windup.

        Args:
            desired: Desired angular velocity (3D)
            measured: Measured angular velocity (3D)
            dt: Time step (uses config default if None)

        Returns:
            Torque command (3D)
        """
        if dt is None:
            dt = self._dt

        error = desired - measured

        # Proportional term
        p_term = self._Kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = np.clip(self._integral, self._integral_min, self._integral_max)
        i_term = self._Ki * self._integral

        # Derivative term
        d_term = self._Kd * (error - self._prev_error) / dt

        # Compute output
        output = p_term + i_term + d_term

        # Clamp output per axis
        output = np.clip(output, self._output_min, self._output_max)

        # Update state
        self._prev_error = error.copy()

        return output

    def reset(self):
        """Reset controller state."""
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)


# =============================================================================
# QP Mixer (moved from model/ctrl/qp_mixer.py)
# =============================================================================

class QpMixer:
    """
    QP-based control mixer for quadrotor control allocation.

    Converts desired total thrust and torques into individual motor commands
    while respecting motor saturation and rate constraints.

    Aerodynamic model-agnostic: works with any aerodynamic model.
    """

    def __init__(self, config: QpMixerConfig):
        """
        Initialize the QP mixer.

        Args:
            config: Mixer configuration object
        """
        self._config = config
        self._build_allocation_matrix()
        self._setup_qp_solver()

    def _build_allocation_matrix(self):
        """Build the control allocation matrix Γ (4x4)."""
        dx, dy, c_tau = self._config.dx, self._config.dy, self._config.c_tau

        # Γ maps motor thrusts to [F_total, τ_roll, τ_pitch, τ_yaw]
        self._Gamma = cs.DM([
            [1, 1, 1, 1],  # F_total
            [-dy, -dy, dy, dy],  # τ_roll
            [-dx, dx, dx, -dx],  # τ_pitch
            [-c_tau, c_tau, -c_tau, c_tau],  # τ_yaw
        ])

    def _setup_qp_solver(self):
        """
        Setup the CasADi QP solver.

        Creates symbolic QP problem:
            minimize: 0.5*z'Hz + g'z
            subject to: lb ≤ z ≤ ub
                      A_eq*z = b_eq
        where z = [u; du]
        """
        # Decision variables: z = [u; du]
        u = cs.SX.sym("u", 4)  # Motor thrusts
        du = cs.SX.sym("du", 4)  # Motor rates
        z = cs.vertcat(u, du)

        # Parameters (vary at each solve)
        tau_des = cs.SX.sym("tau_des", 4)  # Desired [F, τ_roll, τ_pitch, τ_yaw]
        u_prev = cs.SX.sym("u_prev", 4)  # Previous motor commands

        # Cost function
        cfg = self._config
        W = cs.DM(cfg.W_tracking)
        R = cfg.R_prev_cmd

        # Tracking error: ||Γu - τ_des||²_W
        tau_achieved = self._Gamma @ u
        tau_error = tau_des - tau_achieved
        tracking_cost = cs.dot(tau_error, (W @ tau_error))

        # Command smoothness: ||u - u_prev||²_R
        smoothness_cost = R * cs.sum1((u - u_prev) ** 2)

        # Total cost
        cost = 0.5 * (tracking_cost + smoothness_cost)

        # Constraints: motor limits and rate limits
        lb = cs.vertcat(
            cs.DM(cfg.u_min * np.ones(4)),
            cs.DM(cfg.du_min * np.ones(4))
        )
        ub = cs.vertcat(
            cs.DM(cfg.u_max * np.ones(4)),
            cs.DM(cfg.du_max * np.ones(4))
        )

        # Create NLPSolver (more reliable than qpsol)
        p = cs.vertcat(tau_des, u_prev)  # Parameters
        nlp = {
            'x': z,
            'p': p,  # Parameters: [tau_desired, u_prev]
            'f': cost,
        }

        opts = {
            'print_time': False,
            'ipopt': {
                'print_level': 0,
                'tol': 1e-6,
                'max_iter': 100,
            }
        }

        # Create solver (using CasADi's nlpsol with ipopt)
        self._solver = cs.nlpsol(
            'mixer_solver',
            'ipopt',
            nlp,
            opts
        )

        # Store bounds for use during solve
        self._lb = lb
        self._ub = ub

    def mix(
        self,
        tau_desired: np.ndarray,
        u_prev: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray, dict]:
        """
        Compute motor commands from desired control inputs.

        Args:
            tau_desired: Desired [F_total, τ_roll, τ_pitch, τ_yaw]
            u_prev: Previous motor commands (defaults to equal distribution)

        Returns:
            u: Motor thrusts [m1, m2, m3, m4]
            du: Motor rates [dm1, dm2, dm3, dm4]
            info: Dictionary with solver status, statistics
        """
        if u_prev is None:
            # Initialize with equal thrust distribution
            u_prev = np.ones(4) * tau_desired[0] / 4.0

        # Initial guess: equal thrust distribution
        z_init = np.concatenate([
            np.ones(4) * tau_desired[0] / 4.0,  # u
            np.zeros(4),  # du
        ])

        # Solve NLP with parameters
        sol = self._solver(
            x0=z_init,
            lbx=self._lb,
            ubx=self._ub,
            p=cs.vertcat(cs.DM(tau_desired), cs.DM(u_prev)),
        )

        z_opt = sol['x'].full().flatten()
        u = z_opt[:4]
        du = z_opt[4:]

        info = {
            'solver_status': sol.get('return_status', 'unknown'),
            'cost': float(sol['f']),
        }

        return u, du, info


# =============================================================================
# Angular Velocity Controller (main controller combining all components)
# =============================================================================

class CtrlProperAccelZAngVel3D:
    """
    Wrapper around ACADOS NMPC that tracks proper acceleration and angular velocity.

    This controller encapsulates the low-level ACADOS solver and provides
    a clean interface for trajectory tracking with proper control allocation.

    Architecture:
    1. Wraps ACADOS NMPC solver (encapsulates low-level handler)
    2. Extracts ang_vel_des, accel_des from x[1] predicted state
    3. Computes proper_accel_des = accel_des - gravity
    4. PD controller: proper_accel_des, proper_accel_meas → tot_thrust_cmd
    5. PID controller: ang_vel_des, ang_vel_meas → 3d_torque_cmd
    6. QP mixer: [tot_thrust, 3d_torque] → motor_thrusts (4D)
    """

    def __init__(
        self,
        trajectory: TrajectoryReference,
        is_hover_task: bool = False,
        pd_config: PDControllerConfig | None = None,
        pid_config: PIDAntiWindupConfig | None = None,
        mixer_config: QpMixerConfig | None = None,
        mpc_config: MpcConfig | None = None,
    ):
        """
        Initialize the CtrlProperAccelZAngVel3D.

        Args:
            trajectory: TrajectoryReference object
            is_hover_task: If True, hover at initial reference
            pd_config: PD controller configuration (uses default if None)
            pid_config: PID controller configuration (uses default if None)
            mixer_config: QP mixer configuration (uses default if None)
            mpc_config: MPC configuration (uses high_performance default if None)
        """
        self.trajectory = trajectory
        self._is_hover_task = is_hover_task

        # MPC configuration
        if mpc_config is None:
            mpc_config = MpcConfig.create_high_performance()
        self._mpc_config = mpc_config
        self.ctrl_dt = mpc_config.control.ctrl_dt

        # Create OCP and solver
        ocp_cfg = ocp_proposed_mpc_cfg(mpc_config)
        json_path = Path(__file__).parent.parent / ".cache" / "acados_ocp_ctrl_proper_accel_z_ang_vel_3d.json"
        json_path.parent.mkdir(exist_ok=True)
        self._solver = AcadosOcpSolver(ocp_cfg, json_file=str(json_path))

        # Solver dimensions
        self._nx = ocp_cfg.model.x.size()[0]  # 18
        self._nu = ocp_cfg.model.u.size()[0]  # 4

        # Timing
        self._N = mpc_config.nodes
        self._horizon = mpc_config.T_horizon

        # Create sub-controllers
        self._pd_accel = PDController(pd_config or DEFAULT_PD_CONFIG)
        self._pid_ang_vel = PIDAntiWindupController(pid_config or DEFAULT_PID_CONFIG)
        self._qp_mixer = QpMixer(mixer_config or DEFAULT_QP_MIXER_CONFIG)

        # State storage
        self._cz_state = 0.0
        self._cur_idx_ref = 0
        self._sim_dt = 0.02  # Trajectory sampling rate (const, 50Hz)
        self._u_prev = np.zeros(4)  # Track previous motor commands

        # Gravity vector
        self._gravity = np.array([0, 0, 9.81])

    def compute_control(
        self,
        sim_state: np.ndarray,
    ) -> np.ndarray:
        """
        Compute motor commands from current state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]

        Returns:
            Motor thrust rates (du) as (4,) array
        """
        # 1. Run NMPC solver to get predicted state
        mpc_state = self._est_mpc_state(sim_state)

        # Set parameter [cz_rate]
        params = np.array([self._mpc_config.cz_rate])
        for i in range(self._N + 1):
            self._solver.set(i, "p", params)

        # Initialize with current state as guess
        self._solver.set(0, "x", mpc_state)
        self._solver.set(0, "lbx", mpc_state)
        self._solver.set(0, "ubx", mpc_state)

        # Set reference trajectory
        if self._is_hover_task:
            self._set_hover_reference()
        else:
            self._set_tracking_reference()

        # Solve OCP
        status = self._solver.solve()

        if status != 0:
            print(f"Warning: NMPC solver failed with status {status}")
            return np.zeros(self._nu)

        # 2. Extract desired values from predicted state x[1]
        x_next = self._solver.get(1, "x")

        # Extract components
        p_next = x_next[0:3]  # Position
        v_next = x_next[3:6]  # Velocity
        ang_vel_des = x_next[6:9]  # Angular rates (r)
        # q_next = x_next[9:13]  # Quaternion
        u_nmpc = x_next[13:17]  # Motor thrusts

        # Update cz_state estimate
        self._cz_state = x_next[17]

        # 3. Compute accel_des from velocity derivative
        # accel_des = (v_next - v_current) / dt
        v_current = sim_state[3:6]
        accel_des = (v_next - v_current) / self.ctrl_dt

        # Compute proper acceleration (proper_accel = accel - gravity)
        proper_accel_des = accel_des - self._gravity

        # Compute measured proper acceleration
        # For sim state, we don't have acceleration directly
        # Estimate as zero for now (tracking error will be handled by PD)
        proper_accel_meas = np.zeros(3)

        # 4. PD controller for proper acceleration → tot_thrust_cmd
        tot_thrust_cmd = self._pd_accel.compute(proper_accel_des, proper_accel_meas, self.ctrl_dt)

        # 5. PID controller for angular velocity → 3d_torque_cmd
        ang_vel_meas = sim_state[6:9]  # Angular rates from sim state
        torque_cmd = self._pid_ang_vel.compute(ang_vel_des, ang_vel_meas, self.ctrl_dt)

        # 6. QP mixer → motor_thrusts_cmd (4D)
        tau_desired = np.concatenate([[tot_thrust_cmd], torque_cmd])
        motor_thrusts, motor_rates, info = self._qp_mixer.mix(tau_desired, self._u_prev)

        # Update previous command
        self._u_prev = motor_thrusts.copy()

        # Increment reference index after solving (only in tracking mode)
        if not self._is_hover_task:
            total_steps = len(self.trajectory.time)
            self._cur_idx_ref = (self._cur_idx_ref + 1) % total_steps

        return motor_rates

    def _est_mpc_state(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Map GD simulator state (17D) to NMPC state (18D).

        Args:
            sim_state: [p(3), v(3), r(3), q(4), motor_thrusts(4)] = 17D

        Returns:
            mpc_state: [p(3), v(3), r(3), q(4), motor_thrusts(4), cz(1)] = 18D
        """
        # Extract sim state components
        p = sim_state[0:3]    # Position
        v = sim_state[3:6]    # Velocity (world frame)
        r = sim_state[6:9]    # Angular rates
        q = sim_state[9:13]   # Quaternion
        motor_thrusts = sim_state[13:17]  # Motor thrusts

        # Estimate cz from current state
        V_norm = np.linalg.norm(v)
        if V_norm > VELOCITY_EPSILON:
            cz = self._cz_state
        else:
            cz = 0.0
            self._cz_state = 0.0

        # Assemble MPC state
        mpc_state = np.concatenate([p, v, r, q, motor_thrusts, [cz]])

        return mpc_state

    def _set_hover_reference(self):
        """Set constant reference (hover mode)."""
        p_ref = self.trajectory.position[0]
        v_ref = self.trajectory.velocity[0]
        yb_ref = self.trajectory.yb[0]

        y_ref = np.concatenate([
            p_ref,
            v_ref,
            yb_ref,
            np.zeros(4),  # motor_thrusts_rate_ref
        ])

        for i in range(self._N):
            self._solver.set(i, "yref", y_ref)

        # Terminal reference (no motor thrusts)
        y_ref_N = np.concatenate([
            p_ref,
            v_ref,
            yb_ref,
        ])
        self._solver.set(self._N, "yref", y_ref_N)

    def _set_tracking_reference(self):
        """Set reference trajectory for prediction horizon (tracking mode)."""
        ref = self.trajectory.get_reference_window(
            cur_idx_ref=self._cur_idx_ref,
            ctrl_dt=self.ctrl_dt,
            N=self._N
        )

        # Set reference for each stage
        for i in range(self._N):
            p_ref = ref['position'][i]
            v_ref = ref['velocity'][i]
            yb_ref = ref['yb'][i]

            y_ref = np.concatenate([
                p_ref,   # 3
                v_ref,   # 3
                yb_ref,  # 3
                np.zeros(4),  # motor_thrusts_rate_ref
            ])

            self._solver.set(i, "yref", y_ref)

        # Terminal reference
        p_ref_N = ref['position'][-1]
        v_ref_N = ref['velocity'][-1]
        yb_ref_N = ref['yb'][-1]

        y_ref_N = np.concatenate([
            p_ref_N,
            v_ref_N,
            yb_ref_N,
        ])
        self._solver.set(self._N, "yref", y_ref_N)

    def reset(self):
        """Reset controller state."""
        self._cz_state = 0.0
        self._cur_idx_ref = 0
        self._u_prev = np.zeros(4)
        self._pid_ang_vel.reset()
        self._solver.reset()

    # =========================================================================
    # Factory Classmethod
    # =========================================================================

    @classmethod
    def create(
        cls,
        trajectory: TrajectoryReference,
        is_hover_task: bool = False,
        pd_config: PDControllerConfig | None = None,
        pid_config: PIDAntiWindupConfig | None = None,
        mixer_config: QpMixerConfig | None = None,
        mpc_config: MpcConfig | None = None,
    ) -> "CtrlProperAccelZAngVel3D":
        """
        Factory classmethod to create a CtrlProperAccelZAngVel3D.

        Args:
            trajectory: TrajectoryReference object
            is_hover_task: If True, hover at initial reference
            pd_config: PD controller configuration (uses default if None)
            pid_config: PID controller configuration (uses default if None)
            mixer_config: QP mixer configuration (uses default if None)
            mpc_config: MPC configuration (uses high_performance default if None)

        Returns:
            Configured CtrlProperAccelZAngVel3D instance
        """
        return cls(
            trajectory=trajectory,
            is_hover_task=is_hover_task,
            pd_config=pd_config,
            pid_config=pid_config,
            mixer_config=mixer_config,
            mpc_config=mpc_config,
        )