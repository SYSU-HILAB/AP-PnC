# BSD 3-Clause License
# Copyright (c) 2025 Sun Yat-sen University. All rights reserved.
# Authors: Erchao Rong: rongerch@outlook.com
# Authors: Zihao Liu
# Authors: Junning Liang

"""ENU/FLU NMPC with ideal inner-loop tracking and command integrators.

x = [position_W, velocity_W, body_rate_command, quaternion_WB_wxyz,
     specific_thrust_command, cx] (15).

cx is the LUMPED body-X specific-force coefficient [1/m]: it is scaled by |v|^2
to give the body-X aerodynamic specific force. Because +X_B = +Z_L, this single
axis is the legacy wing-normal (lift) direction. It intentionally also absorbs
any actuator body-X component measured on the IMU.
u = d[specific_thrust_command, body_rate_command]/dt (4).
Motor dynamics, torque, the remaining wing axes and rate-loop lag belong to the
plant, not this model. Ground speed only: zero wind is required.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import casadi as cs

if TYPE_CHECKING:
    from acados_template import AcadosModel


def quaternion_kinematics_matrix(body_rate: cs.SX) -> cs.SX:
    """Hamilton right-product matrix Ω(ω), with q̇ = ½ Ω(ω) q (wxyz)."""
    wx, wy, wz = body_rate[0], body_rate[1], body_rate[2]
    return cs.vertcat(
        cs.horzcat(0, -wx, -wy, -wz),
        cs.horzcat(wx, 0, wz, -wy),
        cs.horzcat(wy, -wz, 0, wx),
        cs.horzcat(wz, wy, -wx, 0),
    )


def rotation_matrix_from_quaternion(quaternion_wxyz: cs.SX) -> cs.SX:
    """R_WB ∈ SO(3): rotate FLU body vectors into ENU world coordinates."""
    qw, qx, qy, qz = (quaternion_wxyz[i] for i in range(4))
    return cs.vertcat(
        cs.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
        cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)),
        cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)),
    )


def create_tailsitter_model() -> AcadosModel:
    """Construct the augmented command-rate control model (no actuator lag)."""
    from acados_template import AcadosModel

    position_world = cs.SX.sym("position_world", 3)
    velocity_world = cs.SX.sym("velocity_world", 3)
    body_rate_command = cs.SX.sym("body_rate_command", 3)
    attitude_wxyz = cs.SX.sym("attitude_wxyz", 4)
    specific_thrust_command = cs.SX.sym("specific_thrust_command")
    aero_coefficient_x = cs.SX.sym("aero_coefficient_x")
    state = cs.vertcat(
        position_world,
        velocity_world,
        body_rate_command,
        attitude_wxyz,
        specific_thrust_command,
        aero_coefficient_x,
    )
    command_derivative = cs.SX.sym("command_derivative", 4)  # N/kg/s, rad/s²
    # SIGNED: d(cx)/d(alpha). Measured negative for this airframe, so the
    # parameter itself carries the sign. There is no per-quadrant flip.
    cx_alpha_slope = cs.SX.sym("cx_alpha_slope")

    rotation_world_from_body = rotation_matrix_from_quaternion(attitude_wxyz)
    velocity_body = rotation_world_from_body.T @ velocity_world
    speed_squared = cs.dot(velocity_world, velocity_world)
    # Paper Eq. 1: alpha = atan2(v_z^B, v_x^B) in the paper's body frame, which is
    # atan2(v_x, v_z) in ours. No offset: the regularised version shifted alpha and
    # its rate exactly where the vehicle operates (alpha near 90 deg).
    # alpha itself is not materialised as a symbol: the coefficient rate below
    # is the exact chain rule of atan2(v_x, v_z), so no separate expression is
    # needed (and the regularised form shifted alpha where the vehicle flies).
    aerodynamic_acceleration_body = cs.vertcat(aero_coefficient_x * speed_squared, 0, 0)
    acceleration_world = rotation_world_from_body @ (
        cs.vertcat(0, 0, specific_thrust_command) + aerodynamic_acceleration_body
    ) + cs.vertcat(0, 0, -9.81)
    acceleration_body = rotation_world_from_body.T @ acceleration_world
    # d(v_B)/dt includes the rotating-frame term -omega_B x v_B. Without it the
    # sign of d(alpha)/dt was wrong (a positive body-Y rotation INCREASES alpha
    # in the legacy wing frame). Differentiate the exact atan2 above instead of
    # re-deriving a separate approximation.
    velocity_body_rate = acceleration_body - cs.cross(body_rate_command, velocity_body)
    # Paper Eq. 28: alpha_dot = Omega_by + (a_z cos(alpha) - a_x sin(alpha)) / V,
    # expressed in our axes. This is the exact chain rule of the atan2 above, so
    # the sign and the -omega x v rotating-frame term are both implied.
    numerator = velocity_body[2] * velocity_body_rate[0] - velocity_body[0] * velocity_body_rate[2]
    denominator = velocity_body[0] ** 2 + velocity_body[2] ** 2
    angle_of_attack_rate = numerator / denominator
    coefficient_derivative = cs.if_else(
        speed_squared > 1.0, cx_alpha_slope * angle_of_attack_rate, 0.0
    )
    state_derivative = cs.vertcat(
        velocity_world,
        acceleration_world,
        command_derivative[1:4],
        0.5 * quaternion_kinematics_matrix(body_rate_command) @ attitude_wxyz,
        command_derivative[0],
        coefficient_derivative,
    )
    model = AcadosModel()
    model.name = "tailsitter_flu"  # generated C symbol prefix, not mathematical notation
    model.x = state
    model.xdot = cs.SX.sym("state_derivative", state.numel())
    model.u = command_derivative
    model.p = cx_alpha_slope
    model.f_expl_expr = state_derivative
    model.f_impl_expr = model.xdot - state_derivative
    return model
