#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
# TODO remove the dependency to quaternions
from utils.quaternions import *
import numpy as np
import casadi as cs
from config.config import MpcConfig

# TODO MODEL PARARS TO CONFIG
# gz_x500 parameters
    # J_data = np.array([0.0216666667, 0.0216666667, .04])
    # dx = cs.DM(0.707)
    # dy = cs.DM(0.707)
    # c_tau = cs.DM(0.016)

def tailsitter_flu():
    model_name = "tailsitter_flu"
    # c_tau = cs.DM(0.016)
    mass = cs.DM(1.0)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.01715066, 0.01818037, 0.01987597])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    dx = cs.DM(0.08838834764831845)
    dy = cs.DM(0.08838834764831845)
    # set up states & controls
    p = SX.sym('p', 3)
    v = SX.sym('v', 3)
    r = SX.sym('r', 3)
    q = SX.sym('q', 4)
    u = SX.sym('u', 4)
    x = vertcat(p, v, r, q)
    x_aug = vertcat(x, u)
    cz = SX.sym('cz', 1)
    cur_alpha = SX.sym('cur_alpha', 1)
    cz_rate = SX.sym('cz_rate', 1)
    param = cs.vertcat(cz, cur_alpha, cz_rate)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    du = SX.sym('du', 4)
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    p_dot = SX.sym("p_dot", 3, 1)
    v_dot = SX.sym("v_dot", 3, 1)
    r_dot = SX.sym("r_dot", 3, 1)
    q_dot = SX.sym("q_dot", 4, 1)
    u_dot = SX.sym("u_dot", 4, 1)
    x_dot = cs.vertcat(p_dot, v_dot, r_dot, q_dot)
    x_aut_dot = cs.vertcat(x_dot, u_dot)
    # body frame
    U = u[0] + u[1] + u[2] + u[3]
    tb = cs.vertcat(0.0, 0.0, U)

    # aerodynamics
    vb = v_dot_q_inverse(v, q)
    # calculate alpha
    alpha = cs.atan2(vb[0], (vb[2] + 0.1))
    # alpha = cs.if_else(alpha < 0.0, alpha+cs.pi, alpha)
    aero_comp = cs.if_else(
        alpha > 0.0,
        cz_rate * (alpha - cur_alpha),
        0.0
    )
    F_aero_b_frd = cs.SX([0, 0, 0])
    F_aero_b_frd[0] =   - MpcConfig.drag_est * cs.dot(v,v)
    F_aero_b_frd[1] = 0
    F_aero_b_frd[2] =   - 1.0*(cz+aero_comp) * cs.dot(v,v) 
    q_tf = np.array([0, 0.707, 0, 0.707])
    R_tf = q_to_rot_mat(q_tf)
    F_aero_b_flu_quad_frame = cs.mtimes(cs.SX(R_tf), F_aero_b_frd)
    print(F_aero_b_flu_quad_frame)
    # print(F_aero_b_flu)
    # caused by tau_b 
    # data driven simulator configuration
    # 2 yb 3
    #      xb
    # 1    0
    tau_b = cs.veccat(
        dy * (-u[0] - u[1] + u[2] + u[3]),
        dx * (-u[0] + u[1] + u[2] - u[3]),
        c_tau * (-u[0] + u[1] - u[2] + u[3])
    )


    # dynamics
    f_expl_x = vertcat(
        v,
        v_dot_q(tb/mass, q)  + g + v_dot_q(F_aero_b_flu_quad_frame / mass, q),
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        1 / 2 * cs.mtimes(skew_symmetric(r), q)
    )
    f_expl_u = du
    
    f_expl = vertcat(f_expl_x, f_expl_u)
    f_impl = x_aut_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x_aug
    model.xdot = x_aut_dot
    model.u = du
    # model.z = z
    model.p = param
    model.name = model_name

    return model

if __name__ == "__main__":
    tailsitter_flu()
