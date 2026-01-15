# -*- coding: future_fstrings -*-
#
# Mathematical Utilities for Aerodynamic Simulation
# Created on 2024-12-28
#
# This module provides common mathematical utility functions for
# quaternion operations, rotation matrices, and linear algebra
# operations used in aerodynamic simulation.
#

from __future__ import annotations

import casadi as cs
import numpy as np
from typing import Union


def quat_multiply(
    q1: Union[cs.SX, np.ndarray], q2: Union[cs.SX, np.ndarray]
) -> Union[cs.SX, np.ndarray]:
    """
    Quaternion multiplication: q1 * q2.

    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]

    Returns:
        Product quaternion q1 * q2 in [w, x, y, z] format
    """
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

    if isinstance(q1, cs.SX) or isinstance(q2, cs.SX):
        # CasADi version
        return cs.vertcat(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    else:
        # NumPy version
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )


def right_quat_multiply(
    quat: np.ndarray | None, right: np.ndarray | None
) -> np.ndarray | None:
    """
    Multiply quaternion `quat` by `right` on the right-hand side.

    Args:
        quat: Base quaternion [w, x, y, z] or None.
        right: Quaternion applied on the right [w, x, y, z] or None.

    Returns:
        Quaternion product quat * right (normalized) or the original quat if
        `right` is None. Returns None when `quat` is None.
    """
    if quat is None:
        return None

    quat_np = np.asarray(quat, dtype=float)
    if quat_np.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {quat_np.shape}")

    if right is None:
        return quat_np.copy()

    right_np = np.asarray(right, dtype=float)
    if right_np.shape != (4,):
        raise ValueError(f"Right quaternion must have shape (4,), got {right_np.shape}")

    composed = quat_multiply(quat_np, right_np)
    norm = np.linalg.norm(composed)
    if norm > 1e-9:
        composed = composed / norm
    return composed


def quat_to_rotmat(q: Union[cs.SX, np.ndarray]) -> Union[cs.SX, np.ndarray]:
    """
    Convert quaternion to rotation matrix.

    Args:
        q: Quaternion [w, x, y, z] where w is the scalar part

    Returns:
        3x3 rotation matrix
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    if isinstance(q, (cs.SX, cs.MX, cs.DM)):
        # CasADi version (supports symbolic SX/MX and constant DM inputs)
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
    else:
        # NumPy version
        return np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qw * qz),
                    2 * (qx * qz + qw * qy),
                ],
                [
                    2 * (qx * qy + qw * qz),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qw * qx),
                ],
                [
                    2 * (qx * qz - qw * qy),
                    2 * (qy * qz + qw * qx),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ]
        )


def skew_symmetric(v: Union[cs.SX, np.ndarray]) -> Union[cs.SX, np.ndarray]:
    """
    Create skew-symmetric matrix from 3D vector.

    This is the hat operator: v^hat such that v^hat @ u = v x u

    Args:
        v: 3D vector [vx, vy, vz]

    Returns:
        3x3 skew-symmetric matrix
    """
    if isinstance(v, cs.SX):
        # CasADi version
        return cs.vertcat(
            cs.horzcat(0, -v[2], v[1]),
            cs.horzcat(v[2], 0, -v[0]),
            cs.horzcat(-v[1], v[0], 0),
        )
    else:
        # NumPy version
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quat_apply(
    q: Union[cs.SX, np.ndarray], v: Union[cs.SX, np.ndarray]
) -> Union[cs.SX, np.ndarray]:
    """
    Apply quaternion rotation to a vector.

    Rotates vector v by quaternion q using the formula:
    v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)

    This is more efficient than converting to rotation matrix first.

    Args:
        q: Quaternion [w, x, y, z] where w is the scalar part
        v: 3D vector [vx, vy, vz]

    Returns:
        Rotated 3D vector
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    q_xyz = cs.vertcat(qx, qy, qz) if isinstance(q, cs.SX) else np.array([qx, qy, qz])

    # v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)
    t = 2.0 * cs.cross(q_xyz, v) if isinstance(q, cs.SX) else 2.0 * np.cross(q_xyz, v)
    return (
        v + qw * t + cs.cross(q_xyz, t)
        if isinstance(q, cs.SX)
        else v + qw * t + np.cross(q_xyz, t)
    )


def quat_apply_inverse(
    q: Union[cs.SX, np.ndarray], v: Union[cs.SX, np.ndarray]
) -> Union[cs.SX, np.ndarray]:
    """
    Apply inverse quaternion rotation to a vector.

    Rotates vector v by the inverse of quaternion q (equivalent to
    rotating by the conjugate). Uses the formula:
    v' = v + 2 * cross(q_xyz, cross(q_xyz, v) - q_w * v)

    This is more efficient than converting to rotation matrix and transposing.

    Args:
        q: Quaternion [w, x, y, z] where w is the scalar part
        v: 3D vector [vx, vy, vz]

    Returns:
        Rotated 3D vector (by inverse of q)
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    q_xyz = cs.vertcat(qx, qy, qz) if isinstance(q, cs.SX) else np.array([qx, qy, qz])

    # v' = v + 2 * cross(q_xyz, cross(q_xyz, v) - q_w * v)
    # Equivalent to rotating by conjugate [w, -x, -y, -z]
    t = 2.0 * cs.cross(q_xyz, v) if isinstance(q, cs.SX) else 2.0 * np.cross(q_xyz, v)
    return (
        v - qw * t + cs.cross(q_xyz, t)
        if isinstance(q, cs.SX)
        else v - qw * t + np.cross(q_xyz, t)
    )


def quat_to_euler(q: Union[cs.SX, np.ndarray]) -> Union[cs.SX, np.ndarray]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) using Z-X-Y intrinsic sequence.

    Derivation:
    R_zxy = R_z(yaw) * R_x(roll) * R_y(pitch)

    Equations:
    - sin(roll)  = R[2,1] = 2(wx + yz)
    - tan(yaw)   = -R[0,1] / R[1,1] = 2(wz - xy) / (1 - 2(x^2 + z^2))
    - tan(pitch) = -R[2,0] / R[2,2] = 2(wy - xz) / (1 - 2(x^2 + y^2))

    Args:
        q: Quaternion [w, x, y, z] where w is the scalar part

    Returns:
        Euler angles [roll, pitch, yaw] in radians (ZXY order)
    """
    if isinstance(q, cs.SX):
        # CasADi version (single quaternion)
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # --- Roll (x-axis rotation) ---
        # Range: [-pi/2, pi/2]
        sin_roll = 2 * (qw * qx + qy * qz)

        # Handle gimbal lock (singularity at +/- 90 degrees)
        roll = cs.if_else(
            sin_roll >= 1,
            cs.pi / 2,
            cs.if_else(sin_roll <= -1, -cs.pi / 2, cs.asin(sin_roll)),
        )

        # --- Pitch (y-axis rotation) ---
        # atan2(num, den) handles the signs correctly
        pitch = cs.atan2(2 * (qw * qy - qx * qz), 1 - 2 * (qx**2 + qy**2))

        # --- Yaw (z-axis rotation) ---
        yaw = cs.atan2(2 * (qw * qz - qx * qy), 1 - 2 * (qx**2 + qz**2))

        return cs.vertcat(roll, pitch, yaw)

    else:
        # NumPy version
        if q.ndim == 1:
            # Single quaternion
            qw, qx, qy, qz = q[0], q[1], q[2], q[3]

            # Roll
            sin_roll = 2 * (qw * qx + qy * qz)
            roll = np.arcsin(np.clip(sin_roll, -1.0, 1.0))

            # Pitch
            pitch = np.arctan2(2 * (qw * qy - qx * qz), 1 - 2 * (qx**2 + qy**2))

            # Yaw
            yaw = np.arctan2(2 * (qw * qz - qx * qy), 1 - 2 * (qx**2 + qz**2))

            return np.array([roll, pitch, yaw])

        else:
            # Array of quaternions (N, 4)
            qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

            # Roll
            sin_roll = 2 * (qw * qx + qy * qz)
            # Handle potential numerical errors outside [-1, 1]
            roll = np.where(
                np.abs(sin_roll) >= 1,
                np.copysign(np.pi / 2, sin_roll),
                np.arcsin(np.clip(sin_roll, -1.0, 1.0)),
            )

            # Pitch
            pitch = np.arctan2(2 * (qw * qy - qx * qz), 1 - 2 * (qx**2 + qy**2))

            # Yaw
            yaw = np.arctan2(2 * (qw * qz - qx * qy), 1 - 2 * (qx**2 + qz**2))

            return np.stack([roll, pitch, yaw], axis=1)


def alpha_to_quat(alpha: float, v: np.ndarray) -> np.ndarray:
    """
    Compute quaternion from angle of attack (alpha) and velocity vector.

    Based on tailsitter_df logic (df_forward_nominal in tailsitter_df.cpp):
    - x_s = velocity direction (normalized)
    - y_s = cross(x_s, g) / |cross(x_s, g)|
    - z_s = cross(y_s, x_s)
    - yb = y_s
    - xb = cos(alpha) * x_s - sin(alpha) * z_s
    - zb = cos(alpha) * z_s + sin(alpha) * x_s
    - attitude matrix = [xb, yb, zb] (as columns)

    Args:
        alpha: Angle of attack in radians
        v: 3D velocity vector [vx, vy, vz] in any reference frame

    Returns:
        Quaternion [w, x, y, z] in ROS2 standard format
    """
    # Normalize velocity to get x_s (direction of motion)
    v_norm = np.linalg.norm(v)
    if v_norm < 1e-6:
        # Zero velocity: return identity quaternion
        return np.array([1.0, 0.0, 0.0, 0.0])

    x_s = v / v_norm

    # Gravity vector (downward in NED frame)
    g = np.array([0.0, 0.0, -9.81])
    g_norm = g / np.linalg.norm(g)

    # y_s = cross(x_s, g) normalized - perpendicular to both velocity and gravity
    y_s = np.cross(x_s, g_norm)
    y_s_norm = np.linalg.norm(y_s)

    if y_s_norm > 1e-6:
        y_s = y_s / y_s_norm
    else:
        # Fallback: velocity is aligned with gravity, use arbitrary perpendicular
        # Use cross product with a vector that's not parallel to x_s
        if abs(x_s[2]) < 0.9:
            # x_s is not vertical, use z-axis
            y_s = np.cross(x_s, np.array([0.0, 0.0, 1.0]))
        else:
            # x_s is vertical, use x-axis
            y_s = np.cross(x_s, np.array([1.0, 0.0, 0.0]))
        y_s = y_s / np.linalg.norm(y_s)

    # z_s = cross(x_s, y_s) - completes the right-handed coordinate system
    # Order matters for right-handed rule: x -> y -> z
    z_s = np.cross(x_s, y_s)
    z_s = z_s / np.linalg.norm(z_s)

    # Compute body frame axes from alpha
    # This is the key: alpha rotates the body frame around y_s
    yb = y_s
    xb = np.cos(alpha) * x_s - np.sin(alpha) * z_s
    zb = np.cos(alpha) * z_s + np.sin(alpha) * x_s

    # Attitude matrix: body frame axes as columns [xb, yb, zb]
    # Note: We transpose for scipy (expects world-to-body, not body-to-world)
    attitude = np.stack([xb, yb, zb], axis=1)

    # Convert rotation matrix to quaternion
    # scipy Rotation.from_matrix expects the matrix that rotates FROM
    # the parent frame TO the child frame
    from scipy.spatial.transform import Rotation

    # Transpose: scipy expects the active rotation matrix
    rot = Rotation.from_matrix(attitude.T)
    q_scipy = rot.as_quat()  # Returns [x, y, z, w]

    # Convert to ROS2 standard [w, x, y, z]
    return np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])


# ===========================================================================
# Backward compatibility aliases for feature/utils/math_utils.py API
# These maintain the original function names used by proposed_model.py
# ===========================================================================

def rotate_vector(vector, quaternion):
    """
    Alias for quat_apply (body-to-world rotation).

    Note: Arguments are in reverse order compared to quat_apply.
    """
    return quat_apply(quaternion, vector)


def inverse_rotate_vector(vector, quaternion):
    """
    Alias for quat_apply_inverse (world-to-body rotation).

    Note: Arguments are in reverse order compared to quat_apply_inverse.
    """
    return quat_apply_inverse(quaternion, vector)


def quaternion_to_rotation_matrix(quaternion):
    """Alias for quat_to_rotmat."""
    return quat_to_rotmat(quaternion)


def skew_matrix(vector):
    """Alias for skew_symmetric (3x3)."""
    return skew_symmetric(vector)


def skew_symmetric_matrix(vector):
    """
    Return the 4x4 quaternion skew-symmetric matrix of a 4-vector.

    This is used by proposed_model.py for quaternion kinematics.
    """
    return cs.vertcat(
        cs.horzcat(0, -vector[0], -vector[1], -vector[2]),
        cs.horzcat(vector[0], 0, vector[2], -vector[1]),
        cs.horzcat(vector[1], -vector[2], 0, vector[0]),
        cs.horzcat(vector[2], vector[1], -vector[0], 0),
    )


# ===========================================================================
# Convenience aliases for backward compatibility (from lu_mpc/utils)
# ===========================================================================

def quat_multiply_casadi(q1: cs.SX, q2: cs.SX) -> cs.SX:
    """CasADi-specific quaternion multiplication."""
    return quat_multiply(q1, q2)


def quat_multiply_numpy(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """NumPy-specific quaternion multiplication."""
    return quat_multiply(q1, q2)


def quat_to_rotmat_casadi(q: cs.SX) -> cs.SX:
    """CasADi-specific quaternion to rotation matrix conversion."""
    return quat_to_rotmat(q)


def quat_to_rotmat_numpy(q: np.ndarray) -> np.ndarray:
    """NumPy-specific quaternion to rotation matrix conversion."""
    return quat_to_rotmat(q)


# ============================================================================
# Frame Transformation Constants and Functions
# ============================================================================

# Fixed transformation from WING_FRD to QUAD_FLU
# 180° rotation around [1,0,1] axis (diagonal X-Z)
WING_TO_QUAD_QUAT = np.array([0.0, 0.707, 0.0, 0.707])  # WXYZ format


def wing_frd_to_quad_flu(
    quat_wing: Union[cs.SX, np.ndarray]
) -> Union[cs.SX, np.ndarray]:
    """Transform quaternion from WING_FRD_BODY_FRAME to QUAD_FLU_BODY_FRAME.

    WING_FRD_BODY_FRAME: Forward-Right-Down (wing reference frame)
    QUAD_FLU_BODY_FRAME: Forward-Left-Up (quadcopter reference frame)

    The transformation is a 180° rotation around the [1,0,1] axis (diagonal X-Z).

    Args:
        quat_wing: Quaternion in WING_FRD_BODY_FRAME [w, x, y, z]

    Returns:
        Quaternion in QUAD_FLU_BODY_FRAME [w, x, y, z]
    """
    return quat_multiply(WING_TO_QUAD_QUAT, quat_wing)


def extract_yb_from_quat(
    quat: Union[cs.SX, np.ndarray]
) -> Union[cs.SX, np.ndarray]:
    """Extract the body Y-axis (left direction) from a quaternion.

    For QUAD_FLU_BODY_FRAME, the Y-axis points LEFT.
    This rotates the body Y-axis [0, 1, 0] by the quaternion to get world coordinates.

    Args:
        quat: Quaternion [w, x, y, z] representing orientation

    Returns:
        Y-axis vector in world coordinates [x, y, z]
    """
    y_axis = (
        np.array([0.0, 1.0, 0.0])
        if not isinstance(quat, cs.SX)
        else cs.vertcat(0.0, 1.0, 0.0)
    )
    return quat_apply(quat, y_axis)
