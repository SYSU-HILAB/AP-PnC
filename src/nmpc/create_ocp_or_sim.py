from acados_template import AcadosOcp, AcadosSim, AcadosSimSolver, AcadosOcpSolver
from config.config import MpcConfig
import numpy as np
from model import *
from utils.quaternions import *
import time
from utils.terminal_color import *


def ocp_nls() -> AcadosOcp:
    # recent acados support directly model constraint to cost
    # cost type must be Convex Over Nonlinear / Nonlinear Square
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    model = tailsitter_flu()

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    nparam = model.p.size()[0]
    print(model.x)
    print(nx)
    print(nu)
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.cost_type = 'NONLINEAR_LS'

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
    print(sym_yb)
    cost_x = cs.vertcat(
        sym_p,
        sym_v,
        sym_r,
        sym_yb
    )
    cost_x_size = cost_x.shape[0]

    model.cost_y_expr_0 = cs.vertcat(
        cost_x,
        sym_u
    )
    model.cost_y_expr = cs.vertcat(
        cost_x,
        sym_u
    )
    model.cost_y_expr_e = cost_x

    ny = cost_x_size + nu
    print(ny)
    # set dimensions
    ocp.dims.nx = nx
    ocp.dims.nu = nu
    ocp.dims.np = nparam

    ocp.cost.yref_0 = np.zeros((ny))
    ocp.cost.yref = np.zeros((ny))
    ocp.cost.yref_e = np.zeros((cost_x_size))
    ocp.parameter_values = np.zeros((nparam))
    assert nx == 3 + 3 + 3 + 4
    assert nu == 4

    Q_list = np.hstack((
        MpcConfig.qPos,
        MpcConfig.qVel,
        MpcConfig.qAngV,
        MpcConfig.qYB
    ))
    Q_mat = np.diag(Q_list)  # [x,y,x_d,y_d,th,th_d]
    W_mat = np.zeros((ny, ny))
    We_mat = np.zeros((cost_x_size, cost_x_size))

    We_mat = Q_mat
    ## weight
    ocp.cost.W_0 = W_mat
    # Q_list = np.hstack((
    #     0.05 * MpcConfig.qPos,
    #     0.05 * MpcConfig.qVel,
    #     0.05 * MpcConfig.qAngV,
    #     MpcConfig.qYB
    # ))
    W_mat = np.diag(np.hstack((Q_list, MpcConfig.rU)))

    ocp.cost.W = W_mat
    ocp.cost.W_e = We_mat

    ocp.translate_nls_cost_to_conl()

    ocp.dims.N = MpcConfig.nodes
    ocp.solver_options.tf = MpcConfig.T_horizon
    ocp.solver_options.exact_hess_cost = False
    ocp.constraints.x0 = np.zeros(nx)
    ocp.constraints.lbu = np.array(MpcConfig.lb_u)
    ocp.constraints.ubu = np.array(MpcConfig.ub_u)
    ocp.constraints.Jbu = np.eye(nu)

    # extra constraints
    # vb = v_dot_q_inverse(sym_v, sym_q)
    # ocp.model.con_h_expr = cs.vertcat(
    #     vb[1])
    # ocp.dims.nh = 1
    # ocp.dims.nsh = 1
    # ocp.dims.ns = 1
    # ocp.constraints.lh = np.array([0])
    # ocp.constraints.uh = np.array([0])
    # slack_L1_penalty = 0.5 * 1 * 2 * 2
    # slack_L2_penalty = 0.25 * 1 * 2 * 2
    # ocp.cost.zu = slack_L1_penalty * np.ones((ocp.dims.ns,))
    # ocp.cost.zl = slack_L1_penalty * np.ones((ocp.dims.ns,))
    # # 松弛变量slack的二次罚
    # ocp.cost.Zu = slack_L2_penalty * np.ones((ocp.dims.ns,))
    # ocp.cost.Zl = slack_L2_penalty * np.ones((ocp.dims.ns,))

    # nolinear_slack_lower_bound = 0.0
    # nolinear_slack_upper_bound = 0.0
    # ocp.constraints.lsh = nolinear_slack_lower_bound * np.ones(ocp.dims.nsh)
    # ocp.constraints.ush = nolinear_slack_upper_bound * np.ones(ocp.dims.nsh)
    # ocp.constraints.idxsh = np.array([0])

    print(ocp.dims)
    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_OSQP
    # FULL_CONDENSING_HPIPM
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
    # ocp.solver_options.nlp_solver_ext_qp_res = 1
    ocp.solver_options.nlp_solver_max_iter = 10
    # ocp.solver_options.nlp_solver_max_iter = 20
    ocp.solver_options.qp_solver_iter_max = 5
    # ocp.solver_options.eps_sufficient_descent = 1e-8
    # ocp.solver_options.qp_tol_eq = 1e-04
    # ocp.solver_options.qp_solver_tol_stat = 1e-02
    # ocp.solver_options.print_level = 1
    # ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    # ocp.solver_options.line_search_use_sufficient_descent = 1
    # ocp.solver_options.alpha_reduction = 0.9

    # ocp.solver_options.alpha_min = 0.001
    # ocp.solver_options.ext_cost_num_hess = 1
    # ocp.solver_options.qp_solver_cond_ric_alg = 0
    # ocp.solver_options.globalization_use_SOC = 1
    # ocp.solver_options.levenberg_marquardt = 1e3
    # ocp.solver_options.nlp_solver_ext_qp_res = 1
    # newton step
    # ocp.solver_options.nlp_solver_step_length = 1.0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    # SPEED_ABS SPEED BALANCE ROBUST
    # ocp.dims.nh_0 = 1
    # ocp.solver_options.regularize_method = "CONVEXIFY"  # "MIRROR" #"CONVEXIFY"
    # 4-order ERK
    ocp.solver_options.sim_method_num_stages = 4
    # ocp.
    return ocp


def create_sim_solver_with_model_defined(Nodes, T):
    sim = AcadosSim()
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 1
    sim.solver_options.T = T / Nodes
    sim.solver_options.integrator_type = 'ERK'
    sim.solver_options.sens_forw = True
    sim.model = tailsitter_flu_sim()
    

    return sim


def main():
    ocp = ocp_nls()
    # acados_ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
    #     ocp, json_file="acados_nls_ocp" + ocp.model.name + ".json"
    # )

    # acados_ocp_solver.set()


if __name__ == "__main__":
    main()
