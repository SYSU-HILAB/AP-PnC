import numpy as np

from ap_pnc.utils.math_utils import extract_yb_from_quat, wing_frd_to_quad_flu


def test_wing_frd_to_quad_flu_identity_transform():
    base_quat = np.array([1.0, 0.0, 0.0, 0.0])
    transformed = wing_frd_to_quad_flu(base_quat)
    expected = np.array([0.0, 0.707, 0.0, 0.707])
    assert np.allclose(transformed, expected, atol=1e-6)


def test_extract_yb_from_quat_with_yaw_rotation():
    quat = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    yb_vector = extract_yb_from_quat(quat)
    assert np.allclose(yb_vector, np.array([-1.0, 0.0, 0.0]), atol=1e-6)
