"""
Math utilities shared across experimental feature modules.

These helpers keep quaternion and rotation math in one place instead of
scattering CasADi expressions across controller prototypes.
"""

from __future__ import annotations

import casadi as cs


def skew_matrix(vector):
    """Return the 3x3 skew-symmetric matrix of a 3-vector."""
    return cs.vertcat(
        cs.horzcat(0, -vector[2], vector[1]),
        cs.horzcat(vector[2], 0, -vector[0]),
        cs.horzcat(-vector[1], vector[0], 0),
    )


def skew_symmetric_matrix(vector):
    """Return the 4x4 quaternion skew-symmetric matrix of a 4-vector."""
    return cs.vertcat(
        cs.horzcat(0, -vector[0], -vector[1], -vector[2]),
        cs.horzcat(vector[0], 0, vector[2], -vector[1]),
        cs.horzcat(vector[1], -vector[2], 0, vector[0]),
        cs.horzcat(vector[2], vector[1], -vector[0], 0),
    )


def quaternion_to_rotation_matrix(quaternion):
    """Convert a quaternion [qw, qx, qy, qz] to a 3x3 rotation matrix."""
    qw, qx, qy, qz = (
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3],
    )
    return cs.vertcat(
        cs.horzcat(
            1 - 2 * (qy**2 + qz**2),
            2 * (qx * qy - qw * qz),
            2 * (qx * qz + qw * qy),
        ),
        cs.horzcat(
            2 * (qx * qy + qw * qz),
            1 - 2 * (qx**2 + qz**2),
            2 * (qy * qz - qw * qx),
        ),
        cs.horzcat(
            2 * (qx * qz - qw * qy),
            2 * (qy * qz + qw * qx),
            1 - 2 * (qx**2 + qy**2),
        ),
    )


def rotate_vector(vector, quaternion):
    """Rotate a vector from body to world frame using the quaternion."""
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    return cs.mtimes(rotation_matrix, vector)


def inverse_rotate_vector(vector, quaternion):
    """Rotate a vector from world to body frame using the quaternion."""
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    return cs.mtimes(rotation_matrix.T, vector)
