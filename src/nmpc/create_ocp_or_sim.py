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
    print(model.x)
    print(nx)
    print(nu)
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.cost_type = 'NONLINEAR_LS'

    sym_p = model.x[:3]
    sym_v = model.x[3:6]
    sym_r = model.x[6:9]

    sym_u = model.u
    cost_x = cs.vertcat(
        sym_p,
        sym_v,
        sym_r
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

    # set dimensions
    ocp.dims.nx = nx 
    ocp.dims.nu = nu 


    ocp.cost.yref_0 = np.zeros((ny))
    ocp.cost.yref = np.zeros((ny))
    ocp.cost.yref_e = np.zeros((cost_x_size))


    assert nx == 3 + 3 + 3 + 4
    assert nu == 4
    
    Q_list = np.hstack((
        MpcConfig.qPos, 
        MpcConfig.qVel, 
        MpcConfig.qAngV
        ))
    Q_mat = np.diag(Q_list)  # [x,y,x_d,y_d,th,th_d]
    W_mat= np.zeros((ny,ny))
    We_mat= np.zeros((cost_x_size,cost_x_size))
    W_mat = np.diag(np.hstack((Q_list, MpcConfig.rU)))
    We_mat = Q_mat
    ## weight
    ocp.cost.W_0 = W_mat
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


    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    # ocp.solver_options.nlp_solver_max_iter = 20
    ocp.solver_options.qp_solver_iter_max = 50

    # ocp.solver_options.levenberg_marquardt = 1e4
    ocp.solver_options.hpipm_mode = "ROBUST"
    # ocp.dims.nh_0 = 1

    # 4-order ERK
    ocp.solver_options.sim_method_num_stages = 4
    # ocp.
    return ocp






def create_sim_solver_with_model_defined(Nodes,  T):
    sim = AcadosSim()
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 1
    sim.solver_options.T = T/Nodes
    sim.solver_options.integrator_type = 'ERK'
    sim.solver_options.sens_forw = True
    sim.model = tailsitter_flu()
        
    return sim

def main():
    ocp = ocp_nls()
    # acados_ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
    #     ocp, json_file="acados_nls_ocp" + ocp.model.name + ".json"
    # )

    # acados_ocp_solver.set()


if __name__ == "__main__":
    main()