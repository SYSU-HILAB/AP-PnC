import casadi as cs
import numpy as np

from ap_pnc.lu_mpc.model.sim.utils import math_utils


def _manual_quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def test_quat_multiply_numpy_matches_manual_definition():
    q1 = np.array([0.7071, 0.0, 0.7071, 0.0])
    q2 = np.array([0.9239, 0.3827, 0.0, 0.0])
    expected = _manual_quat_multiply(q1, q2)
    result = math_utils.quat_multiply(q1, q2)
    assert isinstance(result, np.ndarray)
    assert np.allclose(result, expected, atol=1e-6)


def test_quat_to_rotmat_symbolic_matches_numeric_evaluation():
    q_value = np.array([0.92387953, 0.38268343, 0.0, 0.0])
    q_symbolic = cs.SX.sym("q", 4)
    rot_symbolic = math_utils.quat_to_rotmat(q_symbolic)
    rot_fn = cs.Function("quat_rot", [q_symbolic], [rot_symbolic])

    symbolic_eval = np.array(rot_fn(q_value))
    numeric_eval = math_utils.quat_to_rotmat(q_value)

    assert symbolic_eval.shape == (3, 3)
    assert np.allclose(symbolic_eval, numeric_eval, atol=1e-7)


def test_quat_apply_inverse_round_trip_identity():
    quat = np.array([0.5, 0.5, 0.5, 0.5])
    quat /= np.linalg.norm(quat)
    vector = np.array([0.3, -0.2, 1.4])

    rotated = math_utils.quat_apply(quat, vector)
    restored = math_utils.quat_apply_inverse(quat, rotated)

    assert np.allclose(restored, vector, atol=1e-7)
