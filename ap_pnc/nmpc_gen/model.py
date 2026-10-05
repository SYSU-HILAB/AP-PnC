# BSD 3-Clause License
# Copyright (c) 2025 Sun Yat-sen University. All rights reserved.
# Authors: Hanamy: rongerch@outlook.com
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

"""First-order collective-thrust/body-rate NMPC prediction, not a motor model."""
import casadi as cs
from casadi import SX, vertcat


def skew(w):
    return cs.vertcat(cs.horzcat(0, -w[2], w[1]),
                      cs.horzcat(w[2], 0, -w[0]),
                      cs.horzcat(-w[1], w[0], 0))


def skew_symmetric(v):
    return cs.vertcat(cs.horzcat(0, -v[0], -v[1], -v[2]),
                      cs.horzcat(v[0], 0, v[2], -v[1]),
                      cs.horzcat(v[1], -v[2], 0, v[0]),
                      cs.horzcat(v[2], v[1], -v[0], 0))


def Quat2RotMat(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return cs.vertcat(
        cs.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy)),
        cs.horzcat(2 * (qx*qy + qw*qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qw*qx)),
        cs.horzcat(2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx**2 + qy**2)))


def q_oproduct_vec(v, q):
    return cs.mtimes(Quat2RotMat(q), v)


def q_inv_oproduct_vec(v, q):
    return cs.mtimes(Quat2RotMat(q).T, v)


def tailsitter_flu():
    from acados_template import AcadosModel

    # World ENU, body FLU, Hamilton wxyz; mass-normalized collective thrust.
    # Rotor geometry/PWM/mixer/rigid-body torque belong only to the plant.
    p = SX.sym('p', 3)
    v = SX.sym('v', 3)
    rates = SX.sym('rates', 3)
    q = SX.sym('q', 4)
    thrust = SX.sym('specific_force', 1)
    cz_state = SX.sym('cz_state', 1)
    state = vertcat(p, v, rates, q, thrust, cz_state)
    command = SX.sym('command', 4)  # specific_force_sp [N/kg], rates_sp [rad/s]
    params = SX.sym('parameters', 7)  # cz, alpha, cz_rate, tau_thrust, tau_rates xyz
    gravity = cs.vertcat(0, 0, -9.81)
    vb = q_inv_oproduct_vec(v, q)
    alpha = cs.atan2(vb[0], vb[2] + 0.1)
    # Signed body-X specific-force coefficient, consistent with the IMU estimator.
    # No approximate frame quaternion or per-rotor mass factor is needed.
    aero = cs.vertcat(cz_state * cs.dot(v, v), 0, 0)
    acceleration = q_oproduct_vec(cs.vertcat(0, 0, thrust) + aero, q) + gravity
    body_acceleration = q_inv_oproduct_vec(acceleration, q)
    alpha_dot = ((body_acceleration[2] * cs.cos(alpha) - body_acceleration[0] * cs.sin(alpha))
                 / (cs.norm_2(v) + 0.1) + rates[1])
    cz_dot = cs.if_else(cs.dot(v, v) > 1.0, -params[2] * alpha_dot, 0.0)
    rate_dot = cs.vertcat(*[(command[i + 1] - rates[i]) / params[i + 4] for i in range(3)])
    f_expl = vertcat(v, acceleration, rate_dot,
                    0.5 * cs.mtimes(skew_symmetric(rates), q),
                    (command[0] - thrust) / params[3], cz_dot)
    model = AcadosModel()
    model.x = state
    model.xdot = SX.sym('xdot', 15)
    model.u = command
    model.p = params
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl
    model.name = 'tailsitter_flu'
    return model


if __name__ == '__main__':
    tailsitter_flu()
