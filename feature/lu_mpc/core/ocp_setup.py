"""Nonlinear least-squares OCP builder for the lu_mpc controller."""

from __future__ import annotations

import casadi as cs
import numpy as np
from acados_template import AcadosOcp

from feature.lu_mpc.controller.core.config import MpcConfig
from feature.lu_mpc.model.ctrl.proposed_model import tailsitter_flu


def ocp_nls() -> AcadosOcp:
    """Create the AcadosOcp instance for the proposed controller."""
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
    sym_r = model.x[6:9]
    sym_q = model.x[9:13]

    sym_u = model.u

    qw, qx, qy, qz = sym_q[0], sym_q[1], sym_q[2], sym_q[3]
    sym_yb = cs.vertcat(
        2 * (qx * qy - qw * qz),
        1 - 2 * (qx**2 + qz**2),
        2 * (qy * qz + qw * qx),
    )
    cost_x = cs.vertcat(sym_p, sym_v, sym_r, sym_yb)
    cost_x_size = cost_x.shape[0]

    model.cost_y_expr_0 = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr_e = cost_x

    ny = cost_x_size + nu

    ocp.cost.yref_0 = np.zeros((ny))
    ocp.cost.yref = np.zeros((ny))
    ocp.cost.yref_e = np.zeros((cost_x_size))
    ocp.parameter_values = np.zeros((nparam))

    Q_list = np.hstack((MpcConfig.qPos, MpcConfig.qVel, MpcConfig.qAngV, MpcConfig.qYB))
    W_mat_terminal = np.diag(Q_list)
    W_mat = np.diag(np.hstack((Q_list, MpcConfig.rU)))

    ocp.cost.W_0 = np.zeros((ny, ny))
    ocp.cost.W = W_mat
    ocp.cost.W_e = W_mat_terminal

    ocp.translate_nls_cost_to_conl()

    ocp.dims.N = MpcConfig.nodes
    ocp.solver_options.tf = MpcConfig.T_horizon
    ocp.solver_options.exact_hess_cost = False

    ocp.constraints.x0 = np.zeros((nx_aug))
    ocp.constraints.idxbx = np.array([6, 7, 8, 13, 14, 15, 16])
    ocp.constraints.lbx = np.concatenate((MpcConfig.lb_r, MpcConfig.lb_u))
    ocp.constraints.ubx = np.concatenate((MpcConfig.ub_r, MpcConfig.ub_u))

    ocp.constraints.Jbu = np.eye(nu)
    ocp.constraints.ubu = np.array([10.0, 10.0, 10.0, 10.0]) * 0.8
    ocp.constraints.lbu = -ocp.constraints.ubu

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    ocp.solver_options.sim_method_num_stages = 4

    return ocp
