# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@outlook.com
#
import casadi as cs
import numpy as np
from acados_template import AcadosModel
from casadi import SX, vertcat

# import torch
# import l4casadi as l4c

def skew(w):
    return cs.vertcat(
            cs.horzcat(0, -w[2], w[1]),
            cs.horzcat(w[2], 0, -w[0]),
            cs.horzcat(-w[1], w[0], 0))

def skew_symmetric(v):
    return cs.vertcat(
        cs.horzcat(0, -v[0], -v[1], -v[2]),
        cs.horzcat(v[0], 0, v[2], -v[1]),
        cs.horzcat(v[1], -v[2], 0, v[0]),
        cs.horzcat(v[2], v[1], -v[0], 0))


# REC: todo optimize these functions using quaternion algebra directly.
# REC: modify the names to be consistent.
def q_oproduct_vec(v, q):
    rotation_matrix = Quat2RotMat(q)
    return cs.mtimes(rotation_matrix, v)

def q_inv_oproduct_vec(v, q):
    rotation_matrix = Quat2RotMat(q)
    return cs.mtimes(rotation_matrix.T, v)

def Quat2RotMat(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    rotation_matrix = cs.vertcat(
        cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
        cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
        cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

    return rotation_matrix

def tailsitter_flu():
    model_name = "tailsitter_flu"

    # DESPRS: parameters are tuned for performance, not accurate
    # it is quite robust actually, since main motion pattern is constrained by planner.
    mass = cs.DM(1.0)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.01715066, 0.01818037, 0.01987597])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    dx = cs.DM(0.08838834764831845)
    dy = cs.DM(0.08838834764831845)
    ####################################################################

    # set up states & controls
    p = SX.sym('p', 3)
    v = SX.sym('v', 3)
    r = SX.sym('r', 3)
    q = SX.sym('q', 4)
    u = SX.sym('u', 4)
    x = vertcat(p, v, r, q)
    x_aug = vertcat(x, u)
    cz_state = SX.sym('cz_state', 1)
    x_aug = vertcat(x_aug, cz_state)
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
    cz_state_dot = SX.sym('cz_state_dot', 1)
    x_aut_dot = cs.vertcat(x_aut_dot, cz_state_dot)

    # body frame
    U = u[0] + u[1] + u[2] + u[3]
    tb = cs.vertcat(0.0, 0.0, U)

    # aerodynamics
    vb = q_inv_oproduct_vec(v, q)
    # calculate alpha
    alpha = cs.atan2(vb[0], (vb[2] + 0.1))


    F_aero_b_frd = cs.SX([0, 0, 0])
    F_aero_b_frd[0] = 0.0
    F_aero_b_frd[1] = 0.0

    # MARKER
    F_aero_b_frd[2] =   - 1.0*(cz_state) * cs.dot(v,v)



    q_tf = np.array([0, 0.707, 0, 0.707]) # REC: This seems complex to be right.
    R_tf = Quat2RotMat(q_tf)
    F_aero_b_flu_quad_frame = cs.mtimes(cs.SX(R_tf), F_aero_b_frd)
    tau_b = cs.veccat(
        dy * (-u[0] - u[1] + u[2] + u[3]),
        dx * (-u[0] + u[1] + u[2] - u[3]),
        c_tau * (-u[0] + u[1] - u[2] + u[3])
    )
    vv_dot = q_oproduct_vec(tb/mass, q)  + g + q_oproduct_vec(F_aero_b_flu_quad_frame / mass, q)
    vb_dot = -skew(r)*vb + q_inv_oproduct_vec(vv_dot, q)  # noqa: F841 (model derivation reference)


    acc_b = q_inv_oproduct_vec(q_oproduct_vec(tb/mass, q)  + g + q_oproduct_vec(F_aero_b_flu_quad_frame / mass, q), q)

    # alpha_dot = (vb_dot[2]*vb[0] - vb[2]*vb_dot[0]) / (cs.dot(v,v) + 0.1)
    # alpha_dot = (acc_b[2] * vb[0] - acc_b[0] * vb[2]) / (cs.dot(v,v)) + r[1]

    # print(alpha_dot)
    # exit()
    alpha_dot = (acc_b[2] * cs.cos(alpha) - acc_b[0] * cs.sin(alpha)) / (cs.norm_2(v) + 0.1) + r[1]

    # alpha_dot = (
    # maybe we should estimate the alpha_dot data out
    # I guess that when gaussian-newton implicitly estimate the alpha_dot, it is damped. But if formulated as equations, it is very noisy.
    cz_dot = cs.if_else(
        cs.dot(v,v) > 1.0,
        -cz_rate * alpha_dot,
        0.0
    )
    # dynamics
    f_expl_x = vertcat(
        v,
        q_oproduct_vec(tb/mass, q)  + g + q_oproduct_vec(F_aero_b_flu_quad_frame / mass, q),
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
    )
    f_expl_u = du

    f_expl = vertcat(f_expl_x, f_expl_u)
    f_expl = vertcat(f_expl, cz_dot)
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
