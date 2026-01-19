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

import numpy as np
from pathlib import Path

from acados_template import AcadosOcpSolver

from .config.ocp_proposed_mpc_cfg import ocp_proposed_mpc_cfg, MpcConfig
from .config.sub_controllers import (
    PDControllerConfig,
    PIDAntiWindupConfig,
    QpMixerConfig,
    DEFAULT_PD_CONFIG,
    DEFAULT_PID_CONFIG,
    DEFAULT_QP_MIXER_CONFIG,
)
from .sub_controllers import (
    PDController,
    PIDAntiWindupController,
    QpMixer,
)
from .trajectory_loader import TrajectoryReference
import sys
VELOCITY_EPSILON = 0.1


# =============================================================================
# MPC with Angular Velocity Controller (main controller combining all components)
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
        self._solver.set(0, "x", mpc_state)
        self._solver.set(0, "lbx", mpc_state)
        self._solver.set(0, "ubx", mpc_state)

    def compute_control(
        self,
        sim_state: np.ndarray,
        acc_p_meas: np.ndarray,
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

        # 2. Initialize solver with parameters and initial state
        self._initialize_solver(mpc_state)

        # 3. Set reference trajectory
        if self._is_hover_task:
            self._set_hover_reference()
        else:
            self._set_tracking_reference()

        # 4. Solve OCP
        status = self._solver.solve()

        if status != 0:
            print(f"Warning: NMPC solver failed with status {status}")
            sys.exit(1)
            return np.zeros(self._nu)

        # 5. Extract desired values from predicted state x[1]
        x_next = self._solver.get(1, "x")

        ang_vel_des = x_next[6:9]  # Angular rates (r)
        u_nmpc = x_next[13:17]  # Motor thrusts

        accel_p_des = (np.sum(u_nmpc)) / 2.0
        #<HARD
        #1. add counter, everytime the compute_control is called, counter +=1
        #2. print the counter value 
        
        #>

        # 6. PD controller for proper acceleration → tot_thrust_cmd
        incre_thrust = self._pd_accel.compute(accel_p_des, acc_p_meas[2])
        tot_thrust_cmd = accel_p_des * 2.0 + incre_thrust
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
            total_steps = len(self.trajectory.time)
            self._cur_idx_ref = (self._cur_idx_ref + 1) % total_steps

        return motor_thrusts

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