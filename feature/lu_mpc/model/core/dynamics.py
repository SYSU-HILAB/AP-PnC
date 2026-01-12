"""
CasADi dynamics model for the lu_mpc tailsitter experiments.

The implementation mirrors src/nmpc/model.py but depends only on local
feature utilities so that lu_mpc can evolve independently.
"""

from __future__ import annotations

from acados_template import AcadosModel
from casadi import SX, vertcat
import casadi as cs
import numpy as np

from feature.utils.math_utils import (
    inverse_rotate_vector,
    quaternion_to_rotation_matrix,
    rotate_vector,
    skew_matrix,
    skew_symmetric_matrix,
)


def tailsitter_flu() -> AcadosModel:
    """Create the Acados model for the FLU tailsitter dynamics."""
    model_name = "tailsitter_flu"

    mass = cs.DM(1.0)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.01715066, 0.01818037, 0.01987597])
    J = cs.diag(J_data)
    dx = cs.DM(0.08838834764831845)
    dy = cs.DM(0.08838834764831845)

    # states & controls
    p = SX.sym("p", 3)
    v = SX.sym("v", 3)
    r = SX.sym("r", 3)
    q = SX.sym("q", 4)
    u = SX.sym("u", 4)
    x = vertcat(p, v, r, q)
    x_aug = vertcat(x, u)
    cz_state = SX.sym("cz_state", 1)
    x_aug = vertcat(x_aug, cz_state)
    cz = SX.sym("cz", 1)
    cur_alpha = SX.sym("cur_alpha", 1)
    cz_rate = SX.sym("cz_rate", 1)
    param = cs.vertcat(cz, cur_alpha, cz_rate)
    du = SX.sym("du", 4)

    g = cs.vertcat(0.0, 0.0, -9.81)
    p_dot = SX.sym("p_dot", 3, 1)
    v_dot = SX.sym("v_dot", 3, 1)
    r_dot = SX.sym("r_dot", 3, 1)
    q_dot = SX.sym("q_dot", 4, 1)
    u_dot = SX.sym("u_dot", 4, 1)
    x_dot = cs.vertcat(p_dot, v_dot, r_dot, q_dot)
    x_aut_dot = cs.vertcat(x_dot, u_dot)
    cz_state_dot = SX.sym("cz_state_dot", 1)
    x_aut_dot = cs.vertcat(x_aut_dot, cz_state_dot)

    U = u[0] + u[1] + u[2] + u[3]
    tb = cs.vertcat(0.0, 0.0, U)

    vb = inverse_rotate_vector(v, q)
    alpha = cs.atan2(vb[0], (vb[2] + 0.1))

    F_aero_b_frd = cs.SX([0, 0, 0])
    F_aero_b_frd[0] = 0.0
    F_aero_b_frd[1] = 0.0
    F_aero_b_frd[2] = -(cz_state) * cs.dot(v, v)

    q_tf = cs.DM(np.array([0.0, 0.707, 0.0, 0.707]))
    R_tf = quaternion_to_rotation_matrix(q_tf)
    F_aero_b_flu_quad_frame = cs.mtimes(cs.SX(R_tf), F_aero_b_frd)

    tau_b = cs.veccat(
        dy * (-u[0] - u[1] + u[2] + u[3]),
        dx * (-u[0] + u[1] + u[2] - u[3]),
        c_tau * (-u[0] + u[1] - u[2] + u[3]),
    )
    thrust_world = rotate_vector(tb / mass, q)
    aero_world = rotate_vector(F_aero_b_flu_quad_frame / mass, q)
    vv_dot = thrust_world + g + aero_world
    vb_dot = cs.mtimes(-skew_matrix(r), vb) + inverse_rotate_vector(vv_dot, q)

    acc_b = inverse_rotate_vector(thrust_world + g + aero_world, q)

    alpha_dot = (
        (acc_b[2] * cs.cos(alpha) - acc_b[0] * cs.sin(alpha)) / (cs.norm_2(v) + 0.1)
        + r[1]
    )

    cz_dot = cs.if_else(cs.dot(v, v) > 1.0, -cz_rate * alpha_dot, 0.0)

    f_expl_x = vertcat(
        v,
        vv_dot,
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        0.5 * cs.mtimes(skew_symmetric_matrix(r), q),
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
    model.p = param
    model.name = model_name

    return model
