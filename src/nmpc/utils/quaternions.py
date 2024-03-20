""" Miscellaneous utility functions.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""


import math
import pyquaternion
import numpy as np
import casadi as cs
from scipy.interpolate.interpolate import interp1d




def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return np.array([qw, qx, qy, qz])


def quaternion_to_euler(q):
    q = pyquaternion.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]


def unit_quat(q):
    """
    Normalizes a quaternion to be unit modulus.
    :param q: 4-dimensional numpy array or CasADi object
    :return: the unit quaternion in the same data format as the original one
    """

    if isinstance(q, np.ndarray):
        # if (q == np.zeros(4)).all():
        #     q = np.array([1, 0, 0, 0])
        q_norm = np.sqrt(np.sum(q ** 2))
    else:
        q_norm = cs.sqrt(cs.sumsqr(q))
    return 1 / q_norm * q


def v_dot_q(v, q):
    rot_mat = q_to_rot_mat(q)
    if isinstance(q, np.ndarray):
        return rot_mat.dot(v)

    return cs.mtimes(rot_mat, v)

def v_dot_q_inverse(v, q):
    rot_mat = q_to_rot_mat(q)
    if isinstance(q, np.ndarray):
        return np.transpose(rot_mat).dot(v)

    return cs.mtimes(rot_mat.T, v)

def vec_dot_xb_yb(v, xb, yb):
    # if isinstance(q, np.ndarray):
    #     return rot_mat.dot(v)
    zb = cs.cross(xb, yb)
    rot_mat = cs.horzcat(xb, yb, zb)

    return cs.mtimes(rot_mat, v)


def q_to_rot_mat(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    if isinstance(q, np.ndarray):
        rot_mat = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]])

    else:
        rot_mat = cs.vertcat(
            cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
            cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
            cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

    return rot_mat


def q_dot_q(q, r):
    """
    Applies the rotation of quaternion r to quaternion q. In order words, rotates quaternion q by r. Quaternion format:
    wxyz.

    :param q: 4-length numpy array or CasADi MX. Initial rotation
    :param r: 4-length numpy array or CasADi MX. Applied rotation
    :return: The quaternion q rotated by r, with the same format as in the input.
    """

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    rw, rx, ry, rz = r[0], r[1], r[2], r[3]

    t0 = rw * qw - rx * qx - ry * qy - rz * qz
    t1 = rw * qx + rx * qw - ry * qz + rz * qy
    t2 = rw * qy + rx * qz + ry * qw - rz * qx
    t3 = rw * qz - rx * qy + ry * qx + rz * qw

    if isinstance(q, np.ndarray):
        return np.array([t0, t1, t2, t3])
    else:
        return cs.vertcat(t0, t1, t2, t3)


def rotation_matrix_to_quat(rot):
    """
    Calculate a quaternion from a 3x3 rotation matrix.

    :param rot: 3x3 numpy array, representing a valid rotation matrix
    :return: a quaternion corresponding to the 3D rotation described by the input matrix. Quaternion format: wxyz
    """

    q = pyquaternion.Quaternion(matrix=rot)
    return np.array([q.w, q.x, q.y, q.z])


def undo_quaternion_flip(q_past, q_current):
    """
    Detects if q_current generated a quaternion jump and corrects it. Requires knowledge of the previous quaternion
    in the series, q_past
    :param q_past: 4-dimensional vector representing a quaternion in wxyz form.
    :param q_current: 4-dimensional vector representing a quaternion in wxyz form. Will be corrected if it generates
    a flip wrt q_past.
    :return: q_current with the flip removed if necessary
    """

    if np.sqrt(np.sum((q_past - q_current) ** 2)) > np.sqrt(np.sum((q_past + q_current) ** 2)):
        return -q_current
    return q_current

def canonical_quaternion(q):
    norm = np.linalg.norm(q)
    if norm <= 1e-6:
        raise "quaternion norm is too small"
    else:
        # 确保实部 w 大于等于零
        sign = 1 if q[0] >= 0 else -1
        return sign * q / norm


def skew_symmetric(v):
    """
    Computes the skew-symmetric matrix of a 3D vector (PAMPC version)

    :param v: 3D numpy vector or CasADi MX
    :return: the corresponding skew-symmetric matrix of v with the same data type as v
    """

    if isinstance(v, np.ndarray):
        return np.array([[0, -v[0], -v[1], -v[2]],
                         [v[0], 0, v[2], -v[1]],
                         [v[1], -v[2], 0, v[0]],
                         [v[2], v[1], -v[0], 0]])

    return cs.vertcat(
        cs.horzcat(0, -v[0], -v[1], -v[2]),
        cs.horzcat(v[0], 0, v[2], -v[1]),
        cs.horzcat(v[1], -v[2], 0, v[0]),
        cs.horzcat(v[2], v[1], -v[0], 0))


def decompose_quaternion(q):
    """
    Decomposes a quaternion into a z rotation and an xy rotation
    :param q: 4-dimensional numpy array of CasADi MX (format qw, qx, qy, qz)
    :return: two 4-dimensional arrays (same format as input), where the first contains the xy rotation and the second
    the z rotation, in quaternion forms.
    """

    w, x, y, z = q[0], q[1], q[2], q[3]

    if isinstance(q, cs.MX):
        qz = unit_quat(cs.vertcat(w, 0, 0, z))
    else:
        qz = unit_quat(np.array([w, 0, 0, z]))
    qxy = q_dot_q(q, quaternion_inverse(qz))

    return qxy, qz


def quaternion_inverse(q):
    w, x, y, z = q[0], q[1], q[2], q[3]

    if isinstance(q, np.ndarray):
        return np.array([w, -x, -y, -z])
    else:
        return cs.vertcat(w, -x, -y, -z)


def rotation_matrix_to_euler(r_mat):
    sy = math.sqrt(r_mat[0, 0] * r_mat[0, 0] + r_mat[1, 0] * r_mat[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(r_mat[2, 1], r_mat[2, 2])
        y = math.atan2(-r_mat[2, 0], sy)
        z = math.atan2(r_mat[1, 0], r_mat[0, 0])
    else:
        x = math.atan2(-r_mat[1, 2], r_mat[1, 1])
        y = math.atan2(-r_mat[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def quaternion_state_mse(x, x_ref, mask):
    """
    Calculates the MSE of the 13-dimensional state (p_xyz, q_wxyz, v_xyz, r_xyz) wrt. the reference state. The MSE of
    the quaternions are treated axes-wise.

    :param x: 13-dimensional state
    :param x_ref: 13-dimensional reference state
    :param mask: 12-dimensional masking for weighted MSE (p_xyz, q_xyz, v_xyz, r_xyz)
    :return: the mean squared error of both
    """

    q_error = q_dot_q(x[3:7], quaternion_inverse(x_ref[3:7]))
    e = np.concatenate((x[:3] - x_ref[:3], q_error[1:], x[7:10] - x_ref[7:10], x[10:] - x_ref[10:]))

    return np.sqrt((e * np.array(mask)).dot(e))

import numpy as np

def quaternion_slerp(q1, q2, t):
    """
    Perform spherical linear interpolation (slerp) between two quaternions.

    Parameters:
    - q1: The starting quaternion as a numpy array [w, x, y, z].
    - q2: The target quaternion as a numpy array [w, x, y, z].
    - t: Interpolation parameter between 0 and 1.

    Returns:
    - Interpolated quaternion as a numpy array [w, x, y, z].
    """
    # Ensure quaternions are unit quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute the dot product between the quaternions
    dot_product = np.dot(q1, q2)

    # Adjust the signs of quaternions if necessary to ensure the shortest path
    if dot_product < 0.0:
        q1 = -q1
        dot_product = -dot_product

    # Clamp dot product to ensure it stays within the valid range [-1, 1]
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Calculate the angle between the quaternions
    theta = np.arccos(dot_product)

    # Interpolate
    sin_theta = np.sin((1 - t) * theta) / np.sin(theta)
    cos_theta = np.cos((1 - t) * theta)

    result = q1 * sin_theta + q2 * cos_theta
    return result / np.linalg.norm(result)



def quaternion_to_rotation_matrix(quaternion):
    """
    Transformation from quaterion to rotaion matrix.

    Parameters:
    - quaternion: [w,x,y,z].

    Returns:
    - Rotation matrix R with the shape of [3,3] 
    - Axis xb with the shape of [3,1] 
    - Axis yb with the shape of [3,1] 
    - Axis zb with the shape of [3,1] 
    """
    q = canonical_quaternion(quaternion)
    w, x, y, z = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    xb = R[:,0]
    yb = R[:,1]
    zb = R[:,2]
    return R, xb, yb, zb