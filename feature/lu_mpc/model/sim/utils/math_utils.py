# -*- coding: future_fstrings -*-
#
# Mathematical Utilities for Aerodynamic Simulation
# Created on 2024-12-28
#
# This module provides common mathematical utility functions for
# quaternion operations, rotation matrices, and linear algebra
# operations used in aerodynamic simulation.
#

import casadi as cs
import numpy as np
from typing import Union


def quat_multiply(q1: Union[cs.SX, np.ndarray],
                  q2: Union[cs.SX, np.ndarray]) -> Union[cs.SX, np.ndarray]:
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
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )
    else:
        # NumPy version
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])


def quat_to_rotmat(q: Union[cs.SX, np.ndarray]) -> Union[cs.SX, np.ndarray]:
    """
    Convert quaternion to rotation matrix.

    Args:
        q: Quaternion [w, x, y, z] where w is the scalar part

    Returns:
        3x3 rotation matrix
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    if isinstance(q, cs.SX):
        # CasADi version
        return cs.vertcat(
            cs.horzcat(1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
            cs.horzcat(2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)),
            cs.horzcat(2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2))
        )
    else:
        # NumPy version
        return np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
            [2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
            [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)]
        ])


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
            cs.horzcat(-v[1], v[0], 0)
        )
    else:
        # NumPy version
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])


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
        roll = cs.if_else(sin_roll >= 1,
                          cs.pi/2,
                          cs.if_else(sin_roll <= -1,
                                     -cs.pi/2,
                                     cs.asin(sin_roll)))

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
            roll = np.where(np.abs(sin_roll) >= 1,
                            np.copysign(np.pi / 2, sin_roll),
                            np.arcsin(np.clip(sin_roll, -1.0, 1.0)))

            # Pitch
            pitch = np.arctan2(2 * (qw * qy - qx * qz), 1 - 2 * (qx**2 + qy**2))

            # Yaw
            yaw = np.arctan2(2 * (qw * qz - qx * qy), 1 - 2 * (qx**2 + qz**2))

            return np.stack([roll, pitch, yaw], axis=1)

# Convenience aliases for backward compatibility
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
