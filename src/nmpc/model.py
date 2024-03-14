#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from utils.quaternions import *
import numpy as np 
import casadi as cs

# TODO MODEL PARARS TO CONFIG

def tailsitter_flu():
    model_name = "tailsitter_flu"
    # c_tau = cs.DM(0.016)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.0216666667, 0.0216666667, .04])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    dx = cs.DM(0.174)
    dy = cs.DM(0.174)
    # set up states & controls
    p = SX.sym('p',3)
    v = SX.sym('v',3)
    r = SX.sym('r',3)
    q = SX.sym('q',4)
    x = vertcat(p,v,r,q)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    u = SX.sym('u', 4)  
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    p_dot = SX.sym("p_dot",3,1)
    v_dot = SX.sym("v_dot",3,1)
    r_dot = SX.sym("r_dot",3,1)
    q_dot = SX.sym("q_dot",4,1)
    x_dot = cs.vertcat(p_dot,v_dot,r_dot, q_dot)
    # body frame
    U = u[0] + u[1] + u[2] + u[3] 
    tb = cs.vertcat(0.0, 0.0, U)

    # caused by tau_b 
    # data driven simulator configuration
    # 2 yb 3
    #      xb
    # 1    0
    tau_b = cs.veccat(
        dy*(-u[0]-u[1]+u[2]+u[3]),
        dx*(-u[0]+u[1]+u[2]-u[3]),
        c_tau*(-u[0]+u[1]-u[2]+u[3])
    )

    p = []

    # dynamics
    f_expl = vertcat(
        v,
        v_dot_q(tb,q) + g,
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        1 / 2 * cs.mtimes(skew_symmetric(r), q)
    )
    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model


if __name__ == '__main__':
    model = tailsitter_flu()
    # print(model.)

