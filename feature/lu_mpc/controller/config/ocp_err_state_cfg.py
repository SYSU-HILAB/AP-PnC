"""Error-state OCP builder for the err_state_lmpc controller."""

from __future__ import annotations

import casadi as cs
import numpy as np
from acados_template import AcadosOcp
from dataclasses import dataclass, field

from ...model.ctrl.err_state_model import export_error_state_model


# =============================================================================
# Configuration Classes
# =============================================================================

@dataclass
class ErrorStateLinearMpcConfig:
    """
    Configuration for error-state linear MPC controller.

    Cost weights - tune these for performance:
    - Heavier weights on position error for tracking
    - q_pos: Position error weights [x, y, z]
    - q_vel: Velocity error weights [vx, vy, vz]
    - q_theta: Angular error weights [phi, theta, psi]
    - r_u: Control effort weights [increment_aT, increment_omega_x, increment_omega_y, increment_omega_z]
    """

    # Cost weights
    q_pos: np.ndarray = field(
        default_factory=lambda: np.array([15.0, 15.0, 100.0])
    )  # Position error weights
    q_vel: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 1.0, 1.0])
    )  # Velocity error weights
    q_theta: np.ndarray = field(
        default_factory=lambda: np.array([52.5, 52.5, 126.0])
    )  # Angular error weights
    r_u: np.ndarray = field(
        default_factory=lambda: np.array([0.01, 0.01, 0.01, 0.01])
    )  # Control effort weights

    # Constraints on error angular states
    lb_r: np.ndarray = field(
        default_factory=lambda: np.array([-1.5, -2.5, -1.5], dtype=float)
    )  # Angular error lower bounds
    ub_r: np.ndarray = field(
        default_factory=lambda: np.array([1.5, 2.5, 1.5], dtype=float)
    )  # Angular error upper bounds

    # Constraints on control increments
    lb_u: np.ndarray = field(
        default_factory=lambda: np.zeros(4, dtype=float)
    )  # Lower bounds on control increments
    ub_u: np.ndarray = field(
        default_factory=lambda: np.array([5.5, 5.5, 5.5, 5.5])
    )  # Upper bounds on control increments

    # Control timing
    ctrl_dt: float = 0.1
    nodes: int = 5

    @property
    def T_horizon(self) -> float:
        """Prediction horizon duration."""
        return self.ctrl_dt * self.nodes

    @classmethod
    def create_default(cls) -> "ErrorStateLinearMpcConfig":
        """Create default error-state linear MPC configuration."""
        return cls()

    @classmethod
    def create_high_gain(cls) -> "ErrorStateLinearMpcConfig":
        """Create high-gain configuration for tighter tracking."""
        return cls(
            q_pos=np.array([20.0, 20.0, 150.0]),
            q_vel=np.array([2.0, 2.0, 2.0]),
            q_theta=np.array([70.0, 70.0, 180.0]),
        )


DEFAULT_ERROR_STATE_MPC_CONFIG = ErrorStateLinearMpcConfig()


# =============================================================================
# OCP Builder Function
# =============================================================================


def ocp_err_state_cfg(cfg: ErrorStateLinearMpcConfig) -> AcadosOcp:
    """
    Create the AcadosOcp instance for the error state controller.

    The error state model uses linearized dynamics around the reference trajectory.
    State: 9D error state [err_p(3), err_v(3), err_theta(3)]
    Input: 4D control increments [increment_aT(1), increment_omega(3)]
    Parameters: 28D time-varying reference parameters

    Args:
        cfg: ErrorStateLinearMpcConfig configuration

    Returns:
        AcadosOcp configured for error state MPC
    """
    ocp = AcadosOcp()
    model = export_error_state_model()

    ocp.model = model
    nx = model.x.size()[0]  # 9 (error state: p, v, theta)
    nu = model.u.size()[0]  # 4 (increment_aT, increment_omega)
    nparam = model.p.size()[0]  # 28 (v_b_d, R_d, aT_d, omega_d, f_a_d, Jacobian)

    # Cost function: LINEAR_LS is appropriate for linearized error dynamics
    # Goal is to drive all error states to zero
    ocp.cost.cost_type_0 = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.cost_type = "LINEAR_LS"

    # For LINEAR_LS, we need to define Vx and Vu matrices that map state/control to cost output
    # Cost output = [Cx @ x + Cu @ u], we want to minimize state error and control effort
    # State dimension: 9, Control dimension: 4

    # For initial and path stages: cost includes both state and control
    ny_path = nx + nu  # 9 state errors + 4 control increments

    # Vx maps state to cost output (state errors are included directly)
    ocp.cost.Vx_0 = np.zeros((ny_path, nx))
    ocp.cost.Vx_0[:nx, :nx] = np.eye(nx)  # State part

    ocp.cost.Vx = np.zeros((ny_path, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)  # State part

    # Vu maps control to cost output (control effort is included directly)
    ocp.cost.Vu_0 = np.zeros((ny_path, nu))
    ocp.cost.Vu_0[nx:nx+nu, :nu] = np.eye(nu)  # Control part

    ocp.cost.Vu = np.zeros((ny_path, nu))
    ocp.cost.Vu[nx:nx+nu, :nu] = np.eye(nu)  # Control part

    # For terminal stage: only state (no control at terminal)
    ny_terminal = nx
    ocp.cost.Vx_e = np.eye(nx)  # Only state at terminal

    # Reference is zero for error state (goal is zero error)
    ocp.cost.yref_0 = np.zeros((ny_path,))
    ocp.cost.yref = np.zeros((ny_path,))
    ocp.cost.yref_e = np.zeros((ny_terminal,))

    # Initial parameter values (will be set at runtime)
    ocp.parameter_values = np.zeros((nparam))

    # Cost weights from config - tune these for performance
    # Heavier weights on position error for tracking
    q_pos = cfg.q_pos
    q_vel = cfg.q_vel
    q_theta = cfg.q_theta
    r_u = cfg.r_u

    Q_list = np.hstack((q_pos, q_vel, q_theta))

    # Path and initial weights: include both state and control
    W_mat = np.diag(np.hstack((Q_list, r_u)))
    ocp.cost.W_0 = W_mat
    ocp.cost.W = W_mat

    # Terminal weights: only state
    W_mat_terminal = np.diag(Q_list)
    ocp.cost.W_e = W_mat_terminal

    # Dimensions and horizon
    ocp.dims.N = cfg.nodes
    ocp.solver_options.tf = cfg.T_horizon

    # Initial state constraint (default is zero)
    ocp.constraints.x0 = np.zeros((nx))

    # Constraints on error angular states (to limit attitude deviation)
    # Using indices for err_theta: 6, 7, 8 (last 3 of 9 states)
    ocp.constraints.idxbx = np.array([6, 7, 8])
    ocp.constraints.lbx = cfg.lb_r  # Angular error lower bounds
    ocp.constraints.ubx = cfg.ub_r  # Angular error upper bounds

    # Constraints on control increments
    ocp.constraints.Jbu = np.eye(nu)
    ocp.constraints.ubu = cfg.ub_u  # Upper bounds on control increments
    ocp.constraints.lbu = cfg.lb_u  # Lower bounds on control increments

    # Solver options - SQP_RTI for real-time performance
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    ocp.solver_options.sim_method_num_stages = 4

    return ocp
