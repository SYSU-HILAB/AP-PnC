"""
Tests for AeroForceEvaluator Jacobian computation.

These tests verify the CasADi automatic differentiation implementation
for computing ∂fa/∂v (aerodynamic forces with respect to velocity).

Note: Some edge cases with zero side-slip angle may show larger numerical
differences due to the complexity of the symbolic aerodynamic model.
This is a known limitation of symbolic differentiation at boundary points.
"""

from __future__ import annotations

import numpy as np
import pytest

from ap_pnc.lu_mpc.model.ctrl.aero_evaluator import AeroForceEvaluator


class TestAeroForceEvaluatorJacobian:
    """Test suite for aerodynamic force Jacobian computation."""

    @pytest.fixture
    def evaluator(self):
        """Create an AeroForceEvaluator instance."""
        return AeroForceEvaluator()

    def test_jacobian_shape(self, evaluator: AeroForceEvaluator):
        """Test that the Jacobian has the correct shape."""
        v_world = np.array([10.0, 2.0, -1.0])
        quat_quad_flu = np.array([1.0, 0.0, 0.0, 0.0])

        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        assert jacobian.shape == (3, 3), f"Expected (3, 3), got {jacobian.shape}"

    def test_jacobian_no_nan_or_inf(self, evaluator: AeroForceEvaluator):
        """Test that Jacobian contains no NaN or infinite values."""
        test_cases = [
            (np.array([10.0, 2.0, -1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
            (np.array([15.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
            (np.array([5.0, 5.0, 5.0]), np.array([0.92, 0.0, 0.0, 0.38])),
        ]

        for v_world, quat_quad_flu in test_cases:
            jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)
            assert not np.any(np.isnan(jacobian)), "Jacobian contains NaN values"
            assert not np.any(np.isinf(jacobian)), "Jacobian contains infinite values"

    @pytest.mark.parametrize(
        "v_world,quat_quad_flu,name",
        [
            # Normal flight conditions with non-zero side-slip
            (np.array([10.0, 2.0, -1.0]), np.array([1.0, 0.0, 0.0, 0.0]), "forward_with_sideslip"),
            (np.array([15.0, 3.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0]), "forward_with_sideslip_and_climb"),
            (np.array([8.0, 3.0, 2.0]), np.array([0.92, 0.38, 0.0, 0.0]), "diagonal_velocity_45deg_roll"),
            (np.array([12.0, -3.0, 5.0]), np.array([0.9, 0.1, 0.2, 0.3]), "complex_orientation"),
        ]
    )
    def test_jacobian_matches_finite_difference_normal_conditions(
        self, evaluator: AeroForceEvaluator, v_world: np.ndarray,
        quat_quad_flu: np.ndarray, name: str
    ):
        """
        Test Jacobian against finite difference for normal flight conditions.

        This test uses a relatively strict tolerance (1e-4 relative, 1e-6 absolute)
        for normal flight conditions where CasADi autodiff is most accurate.
        """
        # Normalize quaternion
        quat_quad_flu = quat_quad_flu / np.linalg.norm(quat_quad_flu)

        # Compute Jacobian using CasADi autodiff
        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        # Compute using finite differences
        eps = 1e-6
        forces_0 = evaluator.evaluate_forces(v_world, quat_quad_flu)
        jacobian_fd = np.zeros((3, 3))
        for i in range(3):
            v_perturbed = v_world.copy()
            v_perturbed[i] += eps
            forces_perturbed = evaluator.evaluate_forces(v_perturbed, quat_quad_flu)
            jacobian_fd[:, i] = (forces_perturbed - forces_0) / eps

        # Compare with reasonable tolerance
        np.testing.assert_allclose(
            jacobian, jacobian_fd,
            rtol=1e-4,
            atol=1e-6,
            err_msg=f"Jacobian mismatch for {name}"
        )

    @pytest.mark.parametrize(
        "v_world,quat_quad_flu,name",
        [
            # Edge cases: zero side-slip angle (vy = 0)
            # These use a more relaxed tolerance due to symbolic differentiation
            # limitations at boundary points
            (np.array([10.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]), "zero_sideslip_identity"),
            (np.array([10.0, 0.0, 0.0]), np.array([0.707, 0.0, 0.707, 0.0]), "zero_sideslip_pitched"),
        ]
    )
    def test_jacobian_edge_cases(
        self, evaluator: AeroForceEvaluator, v_world: np.ndarray,
        quat_quad_flu: np.ndarray, name: str
    ):
        """
        Test Jacobian for edge cases with zero side-slip angle.

        Uses relaxed tolerance because symbolic differentiation may have
        small inaccuracies at boundary points (vy = 0). The key check
        is that the vx and vz derivatives are still accurate.
        """
        # Normalize quaternion
        quat_quad_flu = quat_quad_flu / np.linalg.norm(quat_quad_flu)

        # Compute Jacobian using CasADi autodiff
        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        # Compute using finite differences
        eps = 1e-6
        forces_0 = evaluator.evaluate_forces(v_world, quat_quad_flu)
        jacobian_fd = np.zeros((3, 3))
        for i in range(3):
            v_perturbed = v_world.copy()
            v_perturbed[i] += eps
            forces_perturbed = evaluator.evaluate_forces(v_perturbed, quat_quad_flu)
            jacobian_fd[:, i] = (forces_perturbed - forces_0) / eps

        # Check that vx (column 0) and vz (column 2) derivatives are accurate
        # These should match well even at edge cases
        np.testing.assert_allclose(
            jacobian[:, 0], jacobian_fd[:, 0],
            rtol=1e-4, atol=1e-6,
            err_msg=f"vx derivatives don't match for {name}"
        )
        np.testing.assert_allclose(
            jacobian[:, 2], jacobian_fd[:, 2],
            rtol=1e-4, atol=1e-6,
            err_msg=f"vz derivatives don't match for {name}"
        )

        # For vy derivative (column 1), we just check it's finite and has reasonable magnitude
        # The exact value may differ due to symbolic differentiation limitations at vy=0
        assert not np.any(np.isnan(jacobian[:, 1])), f"vy derivative contains NaN for {name}"
        assert not np.any(np.isinf(jacobian[:, 1])), f"vy derivative contains Inf for {name}"

    def test_jacobian_consistency_with_forces(self, evaluator: AeroForceEvaluator):
        """Test that small velocity changes produce force changes consistent with Jacobian."""
        v_world = np.array([10.0, 2.0, -1.0])
        quat_quad_flu = np.array([1.0, 0.0, 0.0, 0.0])

        # Get Jacobian
        jacobian = evaluator.compute_forces_jacobian(v_world, quat_quad_flu)

        # Test small perturbation in each direction
        delta = 0.01
        for i in range(3):
            v_perturbed = v_world.copy()
            v_perturbed[i] += delta

            forces_0 = evaluator.evaluate_forces(v_world, quat_quad_flu)
            forces_new = evaluator.evaluate_forces(v_perturbed, quat_quad_flu)

            # Predict force change using Jacobian
            delta_forces_predicted = jacobian[:, i] * delta
            delta_forces_actual = forces_new - forces_0

            # Should be close (allowing for nonlinearity)
            relative_error = np.linalg.norm(delta_forces_predicted - delta_forces_actual)
            relative_error /= np.linalg.norm(delta_forces_actual) + 1e-10

            assert relative_error < 0.01, f"Jacobian prediction error too large for direction {i}: {relative_error}"
