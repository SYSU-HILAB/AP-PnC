#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from utils.quaternions import *
from utils.aero_dynamics import *
import numpy as np
import casadi as cs


# TODO MODEL PARARS TO CONFIG
# gz_x500 parameters
    # J_data = np.array([0.0216666667, 0.0216666667, .04])
    # dx = cs.DM(0.707)
    # dy = cs.DM(0.707)
    # c_tau = cs.DM(0.016)


def tailsitter_flu():
    model_name = "tailsitter_flu"
    # c_tau = cs.DM(0.016)
    mass = cs.DM(1.152)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.00715066, 0.00818037, 0.00987597])
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
    param = cs.vertcat(cz)
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
# simple_f_body_sx
    # only aero force
    F_aero_b_frd = cs.if_else(cs.norm_2(vb) > 0.5,
                              simple_f_body_sx(vb),
                              cs.DM([0, 0, 0])
                              )
    # F_aero_b_frd = fit_aero_body_sx(vb)
    # F_aero_b_frd = 0 * F_aero_b_frd
    # F_aero_b_frd[0] =  -0.02 * cs.dot(v,v) 
    F_aero_b_frd[0] = 0
    F_aero_b_frd[1] = 0
    F_aero_b_frd[2] =   - 0.2 * cs.dot(v,v) 
    # F_aero_b_frd[2] =   0
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

def tailsitter_flu_sim():
    model_name = "tailsitter_flu_sim"
    # c_tau = cs.DM(0.016)
    mass = cs.DM(1.5)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.0216666667, 0.0216666667, .04])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    dx = cs.DM(0.707)
    dy = cs.DM(0.707)
    # set up states & controls
    p = SX.sym('p', 3)
    v = SX.sym('v', 3)
    r = SX.sym('r', 3)
    q = SX.sym('q', 4)
    x = vertcat(p, v, r, q)
    cz = SX.sym('cz', 1)
    param = cs.vertcat(cz)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    u = SX.sym('u', 4)

    delta_u = SX.sym('delta_u', 4)
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    p_dot = SX.sym("p_dot", 3, 1)
    v_dot = SX.sym("v_dot", 3, 1)
    r_dot = SX.sym("r_dot", 3, 1)
    q_dot = SX.sym("q_dot", 4, 1)
    # u_dot = SX.sym("u_dot", 4, 1)
    x_dot = cs.vertcat(p_dot, v_dot, r_dot, q_dot)
    # body frame
    U = u[0] + u[1] + u[2] + u[3]
    tb = cs.vertcat(0.0, 0.0, U)

    # aerodynamics
    vb = v_dot_q_inverse(v, q)

    # only aero force
    F_aero_b_frd = cs.if_else(cs.norm_2(vb) > 0.5,
                              simple_f_body_sx(vb),
                              cs.DM([0, 0, 0])
                              )
    # F_aero_b_frd = SX.sym('F_aero_b_frd', 3)
    F_aero_b_frd = 0.0 * F_aero_b_frd
    # F_aero_b_frd[0] = 0
    # # F_aero_b_frd[0] = 0
    # F_aero_b_frd[1] = 0
    # F_aero_b_frd[2] = - cz * cs.dot(v,v)
    #  -2.1479*vb[2]*cs.norm_2(vb),
    #  cs.DM(0)
    # )

    # F_aero_b_frd = cs.DM([0, 0, 0])
    # F_aero_b_frd = simple_f_body_sx(vb)
    q_tf = np.array([0, 0.707, 0, 0.707])
    R_tf = q_to_rot_mat(q_tf)
    F_aero_b_flu_quad_frame = cs.mtimes(cs.SX(R_tf), F_aero_b_frd)

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
    f_expl = vertcat(
        v,
        v_dot_q(tb, q) / mass + g + v_dot_q(F_aero_b_flu_quad_frame / mass, q),
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
    model.p = []
    model.name = model_name

    return model


def odometry_model():
    model_name = "odometry_model"
    p = MX.sym('p', 3)
    v = MX.sym('v', 3)
    r = MX.sym('r', 3)
    q = MX.sym('q', 4)

    x = vertcat(p, q)

    # xdot
    p_dot = MX.sym("p_dot", 3, 1)
    q_dot = MX.sym("q_dot", 4, 1)
    x_dot = cs.vertcat(p_dot, q_dot)
    u = cs.vertcat(v, r)
    p = []
    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
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

    model = test_huber()
    # print(model.)
def test_model():
    p = SX.sym('p', 3,1)
    v = SX.sym('v', 3,1)
    a = SX.sym('a', 3,1)
    j = SX.sym('j', 3,1)

    g = cs.DM(3,1)
    g[0] = 0
    g[1] = 0
    g[2] = 9.8
    yb = cs.cross(v, a )

    jv = cs.jacobian(yb,v)
    ja = cs.jacobian(yb,a)
    print(jv)
    print('------')
    print(ja)


if __name__ == "__main__":
    tailsitter_flu()
