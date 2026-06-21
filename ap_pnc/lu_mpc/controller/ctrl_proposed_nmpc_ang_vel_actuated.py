"""
Proper Acceleration Z and Angular Velocity 3D Controller for quadrotor trajectory tracking.

This module provides a controller that:
1. Wraps the ACADOS NMPC solver as a low-level handler
2. Extracts ang_vel_des and accel_des from x[1] (predicted state)
3. Computes proper_accel_des = accel_des - gravity
4. Runs PD controller for proper acceleration tracking → tot_thrust_cmd
5. Runs PID controller with anti-windup for angular velocity tracking → 3d_torque_cmd
6. Runs QP mixer for [tot_thrust, 3d_torque] tracking → motor_thrusts_cmd (4D)

Sub-controllers are imported from controller.sub_controllers.
"""

from __future__ import annotations

import sys

import numpy as np
from acados_template import AcadosOcpSolver

from .config.ctrl_proposed_nmpc_cfg import ProposedMpcAngVelActuatedCfg
from .config.ocp_proposed_mpc_cfg import ProposedMpcConfig, ocp_proposed_mpc_cfg
from .sub_controllers import (
    PDController,
    PIDAntiWindupController,
    QpMixer,
)

VELOCITY_EPSILON = 0.1


# =============================================================================
# MPC with Angular Velocity Controller (main controller combining all components)
# =============================================================================

class ProposedNmpcAngVelActuatedController:
    """
    Angular-velocity-actuated proposed NMPC controller for trajectory tracking.

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

    def __init__(self, config: ProposedMpcAngVelActuatedCfg):
        """
        Initialize the ProposedNmpcAngVelActuatedController.

        Args:
            config: Unified configuration containing all controller settings.
        """
        # Extract from config
        self.trajectory = config.trajectory
        self._is_hover_task = config.is_hover_task
        self._gravity = config.gravity

        # MPC configuration
        mpc_config = config.mpc_config or ProposedMpcConfig.create_high_performance()
        self._mpc_config = mpc_config
        self.ctrl_dt = mpc_config.ctrl_dt

        # Create OCP and solver
        ocp_cfg = ocp_proposed_mpc_cfg(mpc_config)
        json_path = config.json_path
        json_path.parent.mkdir(exist_ok=True)
        self._solver = AcadosOcpSolver(ocp_cfg, json_file=str(json_path))

        # Solver dimensions
        self._nx = ocp_cfg.model.x.size()[0]  # 18
        self._nu = ocp_cfg.model.u.size()[0]  # 4

        # Timing
        self._N = mpc_config.nodes
        self._horizon = mpc_config.T

        # Create sub-controllers
        self._pd_accel = PDController(config.pd_config)
        self._pid_ang_vel = PIDAntiWindupController(config.pid_config)
        self._qp_mixer = QpMixer(config.qp_mixer_config)

        # State storage
        self._cz_state = 0.0
        self._cur_idx_ref = 0
        self._sim_dt = 0.02  # Trajectory sampling rate (const, 50Hz)
        self._u_prev = np.zeros(4)  # Track previous motor commands

    @property
    def solver(self) -> AcadosOcpSolver:
        """Get the internal ACADOS solver instance."""
        return self._solver

    def _set_shooting_0_x0(self, state: np.ndarray) -> None:
        """Set shooting node 0 initial state, lower bound, and upper bound."""
        self._solver.set(0, "x", state)
        self._solver.set(0, "lbx", state)
        self._solver.set(0, "ubx", state)

    def _initialize_solver(self, mpc_state: np.ndarray) -> None:
        """
        Initialize ACADOS solver with parameters and initial state.

        Args:
            mpc_state: Initial MPC state (18D) for warm-starting the solver.
        """
        # Set parameter [cz_rate] across all shooting nodes
        params = np.array([self._mpc_config.cz_rate])
        for i in range(self._N + 1):
            self._solver.set(i, "p", params)

        # Initialize with current state as guess and bounds
        self._set_shooting_0_x0(mpc_state)

    def compute_control(
        self,
        sim_state: np.ndarray,
        acc_p_meas: np.ndarray,
    ) -> np.ndarray:
        """
        Compute motor commands from current state.

        Args:
            sim_state: GD simulator state (17D) [p(3), v(3), r(3), q(4), u(4)]
            acc_p_meas: Proper acceleration measurement (3D) in body FLU frame

        Returns:
            Motor thrust rates (du) as (4,) array
        """
        # 1. Update cz_state estimate from acceleration measurement
        v = sim_state[3:6]  # Velocity (world frame)
        V_norm = np.linalg.norm(v)
        if V_norm > VELOCITY_EPSILON:
            self._cz_state = -acc_p_meas[0] / (V_norm ** 2)
        else:
            self._cz_state = 0.0

        # 2. Run NMPC solver to get predicted state
        mpc_state = self._est_mpc_state(sim_state)

        # 2. Initialize solver with parameters and initial state
        self._initialize_solver(mpc_state)

        # 3. Set reference trajectory
        self._set_reference()

        # 4. Solve OCP
        status = self._solver.solve()

        # debug infor
        proposed_solver: AcadosOcpSolver = self.solver

        if status != 0:
            print(f"Warning: NMPC solver failed with status {status}")
            sys.exit(1)
            return np.zeros(self._nu)

        # 5. Extract desired values from predicted state x[1]
        x_next = self._solver.get(1, "x")

        ang_vel_des = x_next[6:9]  # Angular rates (r)
        u_nmpc = x_next[13:17]  # Motor thrusts

        accel_pz_des = (np.sum(u_nmpc)) / 2.0

        # 6. PD controller for proper acceleration → tot_thrust_cmd
        incre_thrust = self._pd_accel.compute(accel_pz_des, acc_p_meas[2])
        tot_thrust_cmd = accel_pz_des * 2.0 + incre_thrust
        # 7. PID controller for angular velocity → 3d_torque_cmd
        ang_vel_meas = sim_state[6:9]  # Angular rates from sim state
        torque_cmd = self._pid_ang_vel.compute(ang_vel_des, ang_vel_meas)

        # 8. QP mixer → motor_thrusts_cmd (4D)
        tau_desired = np.concatenate([[tot_thrust_cmd], torque_cmd])
        motor_thrusts, motor_rates, info = self._qp_mixer.mix(tau_desired, self._u_prev)

        # Update previous command
        self._u_prev = motor_thrusts.copy()


        # Increment reference index after solving (only in tracking mode)
        if not self._is_hover_task:
            self._cur_idx_ref += 1

        return motor_thrusts

    @property
    def is_trajectory_complete(self) -> bool:
        """Check if the trajectory has been fully traversed."""
        if self._is_hover_task:
            return False
        total_steps = len(self.trajectory.time)
        return self._cur_idx_ref >= total_steps

    def _est_mpc_state(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Map GD simulator state (17D) to NMPC state (18D).

        The cz state is estimated from the acceleration measurement in compute_control()
        (acc_p_meas[0] / V^2) and stored in self._cz_state.

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

    def _set_reference(self):
        """Set reference trajectory for prediction horizon."""
        # Get reference window - hover uses N=0 (single ref), tracking uses N=N
        cur_idx_ref = 0 if self._is_hover_task else self._cur_idx_ref
        N = 0 if self._is_hover_task else self._N

        ref_window = self.trajectory.get_reference_window(
            cur_idx_ref=cur_idx_ref,
            ctrl_dt=self.ctrl_dt,
            N=N
        )

        # Tile the single reference for all stages if in hover mode
        if self._is_hover_task:
            for key in ref_window:
                if isinstance(ref_window[key], np.ndarray):
                    if ref_window[key].ndim == 1:  # Scalar field
                        ref_window[key] = np.tile(ref_window[key], self._N + 1)
                    else:  # Vector field
                        ref_window[key] = np.tile(ref_window[key], (self._N + 1, 1))

        # Set reference for each stage
        for i in range(self._N):
            p_ref = ref_window['position'][i]
            v_ref = ref_window['velocity'][i]
            yb_ref = ref_window['yb'][i]

            y_ref = np.concatenate([
                p_ref,   # 3
                v_ref,   # 3
                yb_ref,  # 3
                np.zeros(4),  # motor_thrusts_rate_ref
            ])
            self._solver.set(i, "yref", y_ref)

        # Terminal reference
        p_ref_N = ref_window['position'][-1]
        v_ref_N = ref_window['velocity'][-1]
        yb_ref_N = ref_window['yb'][-1]

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
