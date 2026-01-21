"""Tests for AeroForceEvaluator."""

import numpy as np
import pytest

from feature.lu_mpc.model.ctrl.aero_evaluator import AeroForceEvaluator


def test_forces_jacobian_shape():
    """Test that the Jacobian has the correct shape."""
    evaluator = AeroForceEvaluator()

    # Test velocity and orientation
    v_world = np.array([10.0, 0.0, 0.0])
    quat_quad_flu = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

    # Compute Jacobian
    jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

    # Verify shape
    assert jacobian.shape == (3, 3), f"Expected (3, 3), got {jacobian.shape}"


def test_forces_jacobian_numerical():
    """Test that the computed Jacobian matches finite differences."""
    evaluator = AeroForceEvaluator()

    # Test velocity and orientation
    v_world = np.array([10.0, 2.0, -1.0])
    quat_quad_flu = np.array([0.99, 0.01, 0.02, 0.13])  # Slight rotation

    # Compute Jacobian
    jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

    # Numerical verification using finite differences
    eps = 1e-6
    forces_0 = evaluator.evaluate_forces(v_world, quat_quad_flu)

    jacobian_fd = np.zeros((3, 3))
    for i in range(3):
        v_perturbed = v_world.copy()
        v_perturbed[i] += eps
        forces_perturbed = evaluator.evaluate_forces(v_perturbed, quat_quad_flu)
        jacobian_fd[:, i] = (forces_perturbed - forces_0) / eps

    # Compare computed and numerical Jacobians
    np.testing.assert_allclose(
        jacobian, jacobian_fd, rtol=1e-4, atol=1e-6,
        err_msg="Computed Jacobian does not match finite difference approximation"
    )


def test_forces_jacobian_identity_orientation():
    """Test Jacobian with identity orientation (no rotation)."""
    evaluator = AeroForceEvaluator()

    v_world = np.array([15.0, 0.0, 0.0])
    quat_quad_flu = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

    jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

    # With identity orientation and velocity purely in x-direction,
    # we should get reasonable finite derivatives
    assert not np.any(np.isnan(jacobian)), "Jacobian contains NaN values"
    assert not np.any(np.isinf(jacobian)), "Jacobian contains infinite values"


def test_forces_jacobian_multiple_velocities():
    """Test Jacobian computation with multiple velocity conditions."""
    evaluator = AeroForceEvaluator()

    quat_quad_flu = np.array([1.0, 0.0, 0.0, 0.0])

    test_velocities = [
        np.array([5.0, 0.0, 0.0]),   # Low speed, forward
        np.array([20.0, 0.0, 0.0]),  # High speed, forward
        np.array([10.0, 5.0, 0.0]),  # With sideslip
        np.array([10.0, 0.0, 5.0]),  # With angle of attack
        np.array([10.0, 3.0, -2.0]), # Combined
    ]

    for v_world in test_velocities:
        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        # Check shape and finite values
        assert jacobian.shape == (3, 3)
        assert not np.any(np.isnan(jacobian))
        assert not np.any(np.isinf(jacobian))


def test_forces_jacobian_with_rotation():
    """Test Jacobian with non-identity orientation."""
    evaluator = AeroForceEvaluator()

    v_world = np.array([10.0, 0.0, 0.0])

    # Test with various orientations
    orientations = [
        np.array([1.0, 0.0, 0.0, 0.0]),  # Identity
        np.array([0.707, 0.707, 0.0, 0.0]),  # 90 deg roll
        np.array([0.707, 0.0, 0.707, 0.0]),  # 90 deg pitch
        np.array([0.707, 0.0, 0.0, 0.707]),  # 90 deg yaw
    ]

    for quat_quad_flu in orientations:
        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        # Check shape and finite values
        assert jacobian.shape == (3, 3)
        assert not np.any(np.isnan(jacobian))
        assert not np.any(np.isinf(jacobian))


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v"])
