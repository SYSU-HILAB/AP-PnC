# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
# Tail-sitter UAV.

"""Generate the acados OCP solver code for the standalone NMPC solver.

Run via `uv run ap-pnc gen-nmpc-lib` with cwd .artifacts/c_generated_code:
the json and C sources land there, then the solver wrapper builds and merges
everything into libnmpc_bundle.a.
"""

import casadi as cs
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver

from ap_pnc.nmpc_gen.config import MpcConfig
from ap_pnc.nmpc_gen.model import tailsitter_flu


def ocp_nls() -> AcadosOcp:
    """Formulate the prior-free NMPC OCP (convex-over-nonlinear least squares)."""
    ocp = AcadosOcp()
    model = tailsitter_flu()

    ocp.model = model
    nx_aug = model.x.size()[0]
    nx = nx_aug - model.u.size()[0]
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

    qw = sym_q[0]
    qx = sym_q[1]
    qy = sym_q[2]
    qz = sym_q[3]
    sym_yb = cs.vertcat(
        2 * (qx * qy - qw * qz),
        1 - 2 * (qx ** 2 + qz ** 2),
        2 * (qy * qz + qw * qx),
    )
    cost_x = cs.vertcat(
        sym_p,
        sym_v,
        sym_r,
        sym_yb
    )
    cost_x_size = cost_x.shape[0]

    model.cost_y_expr_0 = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr = cs.vertcat(cost_x, sym_u)
    model.cost_y_expr_e = cost_x

    ny = cost_x_size + nu
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(cost_x_size)
    ocp.parameter_values = np.zeros(nparam)
    assert nx == 3 + 3 + 3 + 4 + 1
    assert nu == 4

    Q_list = np.hstack((MpcConfig.qPos, MpcConfig.qVel, MpcConfig.qAngV, MpcConfig.qYB))
    Q_mat = np.diag(Q_list)  # [x,y,x_d,y_d,th,th_d]
    W_mat = np.zeros((ny, ny))
    We_mat = Q_mat
    ## weight
    ocp.cost.W_0 = W_mat
    W_mat = np.diag(np.hstack((Q_list, MpcConfig.rU)))

    ocp.cost.W = W_mat
    ocp.cost.W_e = We_mat

    ocp.translate_nls_cost_to_conl()

    ocp.dims.N = MpcConfig.nodes
    ocp.solver_options.tf = MpcConfig.T_horizon
    ocp.solver_options.exact_hess_cost = False

    ocp.constraints.x0 = np.zeros(nx_aug)
    # bound part of x
    ocp.constraints.idxbx = np.array([6, 7, 8, 13, 14, 15, 16])
    ocp.constraints.lbx = np.concatenate((MpcConfig.lb_r, MpcConfig.lb_u))
    ocp.constraints.ubx = np.concatenate((MpcConfig.ub_r, MpcConfig.ub_u))

    ocp.constraints.Jbu = np.eye(nu)
    ocp.constraints.ubu = np.array([10.0, 10.0, 10.0, 10.0]) * 1.0 * 0.8
    ocp.constraints.lbu = -np.array([10.0, 10.0, 10.0, 10.0]) * 1.0 * 0.8

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    # 4-order ERK
    ocp.solver_options.sim_method_num_stages = 4
    return ocp


def main() -> None:
    ocp: AcadosOcp = ocp_nls()
    # Emit code next to the json file (.artifacts/c_generated_code, set by gen-nmpc-lib cwd)
    ocp.code_gen_opts.code_export_directory = "."
    AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )


if __name__ == "__main__":
    main()
