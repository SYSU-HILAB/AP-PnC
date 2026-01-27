"""
MPC configuration and OCP builder for the proposed lu_mpc controller.

This module contains:
- ProposedMpcConfig: MPC parameters (cost weights, constraints, timing, etc.)
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
class ProposedMpcConfig:
    """Cleaned up MPC configuration."""

    # Timing configuration
    ctrl_dt: float = 0.1
    nodes: int = 10  # tuned for tracking in acados simulator.
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
    def control_frequency(self) -> float:
        return 1.0 / self.ctrl_dt

    @property
    def T(self) -> float:
        return self.ctrl_dt * self.nodes

    @staticmethod
    def _yb_weights(qh: float, qv: float) -> np.ndarray:
        return np.array([qh, qh, qv], dtype=float) * 100.0 * 0.35

    @classmethod
    def create_high_performance(cls) -> "ProposedMpcConfig":
        qh = 2.0
        qv = 5.0
        return cls(qYB=cls._yb_weights(qh, qv))


DEFAULT_MPC_CONFIG = ProposedMpcConfig()


# =============================================================================
# OCP Builder Function
# =============================================================================

def ocp_proposed_mpc_cfg(cfg: ProposedMpcConfig) -> AcadosOcp:
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

    ocp.cost.cost_type_0 = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.cost.cost_type = "EXTERNAL"
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



    cost_x = cs.vertcat(sym_p, sym_v, sym_yb)
    
    # --- 3. 定义权重矩阵 (转化为 CasADi 常数或参数) ---
    Q_list = np.hstack((cfg.qPos, cfg.qVel, cfg.qYB))
    W_x = cs.diag(Q_list)
    W_u = cs.diag(cfg.rU)

    # --- 4. 构建标量 Cost 表达式 ---
    # 注意：External cost 必须是标量。这里假设 yref 为 0。
    # 如果 yref 不为 0，建议将其放入 model.p 中，然后在这里减去：(cost_x - p_ref).T @ W_x @ (cost_x - p_ref)
    expr_ext_cost_x = cost_x.T @ W_x @ cost_x
    expr_ext_cost_u = sym_u.T @ W_u @ sym_u

    model.cost_expr_ext_cost_0 = expr_ext_cost_x + expr_ext_cost_u
    model.cost_expr_ext_cost = expr_ext_cost_x + expr_ext_cost_u
    model.cost_expr_ext_cost_e = expr_ext_cost_x

    ocp.parameter_values = np.zeros((nparam))
    ocp.dims.N = cfg.nodes
    ocp.solver_options.tf = cfg.T

    # --- 6. 关键设置：开启 Exact Hessian ---
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.nlp_solver_exact_hessian = "RECURSIVE" # 或 "FULL"
    ocp.solver_options.exact_hess_cost = True 
    ocp.solver_options.exact_hess_dyn = True  # 动力学也使用 Exact Hessian

    ocp.dims.N = cfg.nodes
    ocp.solver_options.tf = cfg.T

    ocp.constraints.x0 = np.zeros((nx_aug))
    ocp.constraints.idxbx = np.array([6, 7, 8, 13, 14, 15, 16])
    ocp.constraints.lbx = np.concatenate((cfg.lb_r, cfg.lb_u))
    ocp.constraints.ubx = np.concatenate((cfg.ub_r, cfg.ub_u))

    ocp.constraints.Jbu = np.eye(nu)
    ocp.constraints.ubu = np.array([10.0, 10.0, 10.0, 10.0]) * 0.8
    ocp.constraints.lbu = -ocp.constraints.ubu

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"

    return ocp
