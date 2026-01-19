"""
MPC configuration and OCP builder for the proposed lu_mpc controller.

This module contains:
- ControlConfig: Basic controller timing configuration
- MpcConfig: MPC parameters (cost weights, constraints, etc.)
- ocp_proposed_mpc_cfg: Function that builds the ACADOS OCP
"""

from __future__ import annotations

import casadi as cs
import numpy as np
from acados_template import AcadosOcp
from dataclasses import dataclass, field


# =============================================================================
# Configuration Classes
# =============================================================================

@dataclass
class ControlConfig:
    """Basic controller timing configuration."""

    ctrl_dt: float = 0.1
    nodes: int = 5 # tuned for tracking in acados simulator.

    @property
    def control_frequency(self) -> float:
        return 1.0 / self.ctrl_dt

    @property
    def T(self) -> float:
        return self.ctrl_dt * self.nodes


@dataclass
class MpcConfig:
    """Cleaned up MPC configuration."""

    control: ControlConfig = field(default_factory=ControlConfig)
    lb_r: np.ndarray = field(
        default_factory=lambda: np.array([-1.5, -2.5, -1.5], dtype=float)
    )
    ub_r: np.ndarray = field(
        default_factory=lambda: np.array([1.5, 2.5, 1.5], dtype=float)
    )

    lb_u: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    ub_u: np.ndarray = field(default_factory=lambda: np.array([5.5, 5.5, 5.5, 5.5]))

    qPos: np.ndarray = field(default_factory=lambda: np.array([15.0, 15.0, 100.0]))
    qYB: np.ndarray = field(default_factory=lambda: np.array([52.5, 52.5, 126.0]))
    qVel: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))

    cz_rate: float = 0.1

    qAngV: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    rU: np.ndarray = field(default_factory=lambda: np.array([0.001, 0.001, 0.001, 0.001]))

    @property
    def nodes(self) -> int:
        return self.control.nodes

    @property
    def T_horizon(self) -> float:
        return self.control.T

    @staticmethod
    def _yb_weights(qh: float, qv: float) -> np.ndarray:
        return np.array([qh, qh, qv], dtype=float) * 100.0 * 0.35

    @classmethod
    def create_high_performance(cls) -> "MpcConfig":
        qh = 2.0
        qv = 5.0
        return cls(qYB=cls._yb_weights(qh, qv))


DEFAULT_CONTROL_CONFIG = ControlConfig()
DEFAULT_MPC_CONFIG = MpcConfig()


# =============================================================================
# OCP Builder Function
# =============================================================================

def ocp_proposed_mpc_cfg(cfg: MpcConfig) -> AcadosOcp:
    """
    Create the AcadosOcp instance for the proposed controller.

    Args:
        cfg: MPC configuration object with cost weights, constraints, and timing.
    """
    from ...model.ctrl.proposed_model import tailsitter_flu

    ocp = AcadosOcp()
    model = tailsitter_flu()

    ocp.model = model
    nx_aug = model.x.size()[0]
    nu = model.u.size()[0]
    nparam = model.p.size()[0]

    ocp.cost.cost_type_0 = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.cost_type = "NONLINEAR_LS"
    sym_p = model.x[:3]
    sym_v = model.x[3:6]
    # sym_r = model.x[6:9]
    sym_q = model.x[9:13]

    sym_u = model.u

    qw, qx, qy, qz = sym_q[0], sym_q[1], sym_q[2], sym_q[3]
    sym_yb = cs.vertcat(
        2 * (qx * qy - qw * qz),
        1 - 2 * (qx**2 + qz**2),
        2 * (qy * qz + qw * qx),
    )
    cost_x = cs.vertcat(sym_p, sym_v, sym_yb)
    cost_x_size = cost_x.shape[0]

    model.cost_y_expr_0 = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr_e = cost_x

    ny = cost_x_size + nu

    ocp.cost.yref_0 = np.zeros((ny))
    ocp.cost.yref = np.zeros((ny))
    ocp.cost.yref_e = np.zeros((cost_x_size))
    ocp.parameter_values = np.zeros((nparam))

    Q_list = np.hstack((cfg.qPos, cfg.qVel, cfg.qYB))
    W_mat_terminal = np.diag(Q_list)
    W_mat = np.diag(np.hstack((Q_list, cfg.rU)))

    ocp.cost.W_0 = np.zeros((ny, ny))
    ocp.cost.W = W_mat
    ocp.cost.W_e = W_mat_terminal

    ocp.translate_nls_cost_to_conl()

    ocp.dims.N = cfg.nodes
    ocp.solver_options.tf = cfg.T_horizon
    ocp.solver_options.exact_hess_cost = False

    ocp.constraints.x0 = np.zeros((nx_aug))
    ocp.constraints.idxbx = np.array([6, 7, 8, 13, 14, 15, 16])
    ocp.constraints.lbx = np.concatenate((cfg.lb_r, cfg.lb_u))
    ocp.constraints.ubx = np.concatenate((cfg.ub_r, cfg.ub_u))

    ocp.constraints.Jbu = np.eye(nu)
    ocp.constraints.ubu = np.array([10.0, 10.0, 10.0, 10.0]) * 0.8
    ocp.constraints.lbu = -ocp.constraints.ubu

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    ocp.solver_options.sim_method_num_stages = 4

    return ocp
