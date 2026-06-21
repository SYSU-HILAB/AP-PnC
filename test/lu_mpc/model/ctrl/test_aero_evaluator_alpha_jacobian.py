"""
Tests for AeroForceEvaluator Jacobian computation w.r.t. alpha.

These tests verify the CasADi automatic differentiation implementation
for computing ∂B_fa/∂alpha (body-frame aerodynamic forces with respect to angle of attack).
"""

from __future__ import annotations

import numpy as np
import pytest

from ap_pnc.lu_mpc.model.ctrl.aero_evaluator import AeroForceEvaluator
from ap_pnc.lu_mpc.controller.trajectory_loader import TrajectoryReference, find_planning_csv, load_trajectory_csv


class TestAeroForceEvaluatorAlphaJacobian:
    """Test suite for body-frame aerodynamic force Jacobian w.r.t. alpha."""

    @pytest.fixture
    def evaluator(self) -> AeroForceEvaluator:
        """Create an AeroForceEvaluator instance."""
        return AeroForceEvaluator()

    @pytest.fixture
    def trajectory(self) -> TrajectoryReference:
        """Load a real trajectory from the planning CSV."""
        planning_csv = find_planning_csv()
        if planning_csv is None:
            pytest.skip("No planning CSV found. Run the global planner first.")
        trajectory_data = load_trajectory_csv(str(planning_csv))
        return TrajectoryReference(trajectory_data)

    def test_jacobian_alpha_shape(self, evaluator: AeroForceEvaluator):
        """Test that the Jacobian has the correct shape (3,)."""
        V = 10.0
        alpha = 0.1

        jacobian = evaluator.compute_body_forces_jacobian_alpha(V, alpha)

        assert jacobian.shape == (3,), f"Expected (3,), got {jacobian.shape}"

    def test_jacobian_alpha_no_nan_or_inf(self, evaluator: AeroForceEvaluator):
        """Test that Jacobian contains no NaN or infinite values."""
        test_cases = [
            (10.0, 0.0),    # Zero alpha
            (10.0, 0.1),    # Small positive alpha
            (10.0, -0.1),   # Small negative alpha
            (15.0, 0.5),    # Moderate alpha
            (5.0, 1.0),     # Large alpha (~57 degrees)
            (20.0, -0.3),   # High speed, negative alpha
        ]

        for V, alpha in test_cases:
            jacobian = evaluator.compute_body_forces_jacobian_alpha(V, alpha)
            assert not np.any(np.isnan(jacobian)), f"Jacobian contains NaN for V={V}, alpha={alpha}"
            assert not np.any(np.isinf(jacobian)), f"Jacobian contains Inf for V={V}, alpha={alpha}"

    def test_jacobian_alpha_matches_finite_difference(self, evaluator: AeroForceEvaluator):
        """Test Jacobian against finite difference approximation."""
        V = 10.0
        alpha = 0.1
        epsilon = 1e-6

        # Compute analytical Jacobian
        jacobian = evaluator.compute_body_forces_jacobian_alpha(V, alpha)

        # Finite difference approximation
        # Compute body forces at alpha +/- epsilon
        velb_plus = np.array([V * np.cos(alpha + epsilon), 0.0, V * np.sin(alpha + epsilon)])
        velb_minus = np.array([V * np.cos(alpha - epsilon), 0.0, V * np.sin(alpha - epsilon)])

        forces_plus, _, _, _ = evaluator.model.evaluate(velb_plus)
        forces_minus, _, _, _ = evaluator.model.evaluate(velb_minus)

        fd_jacobian = (forces_plus - forces_minus) / (2 * epsilon)

        # Compare with reasonable tolerance
        np.testing.assert_allclose(
            jacobian, fd_jacobian,
            rtol=1e-4,
            atol=1e-6,
            err_msg="Analytical Jacobian does not match finite difference approximation"
        )

    @pytest.mark.parametrize(
        "V,alpha,name",
        [
            (10.0, 0.0, "zero_alpha"),
            (10.0, 0.1, "small_alpha"),
            (10.0, -0.1, "small_negative_alpha"),
            (15.0, 0.5, "moderate_alpha"),
            (5.0, 1.0, "large_alpha"),
            (20.0, -0.3, "high_speed_negative_alpha"),
        ]
    )
    def test_jacobian_alpha_multiple_conditions(
        self, evaluator: AeroForceEvaluator, V: float, alpha: float, name: str
    ):
        """Test Jacobian computation with multiple (V, alpha) conditions."""
        jacobian = evaluator.compute_body_forces_jacobian_alpha(V, alpha)

        # Check shape and finite values
        assert jacobian.shape == (3,), f"Expected (3,) for {name}, got {jacobian.shape}"
        assert not np.any(np.isnan(jacobian)), f"NaN values for {name}"
        assert not np.any(np.isinf(jacobian)), f"Inf values for {name}"

    def test_jacobian_alpha_consistency_with_forces(self, evaluator: AeroForceEvaluator):
        """Test that small alpha changes produce force changes consistent with Jacobian."""
        V = 10.0
        alpha = 0.1
        delta = 0.001  # Smaller delta for better linear approximation

        # Get Jacobian
        jacobian = evaluator.compute_body_forces_jacobian_alpha(V, alpha)

        # Compute forces at alpha and alpha + delta
        velb_0 = np.array([V * np.cos(alpha), 0.0, V * np.sin(alpha)])
        velb_1 = np.array([V * np.cos(alpha + delta), 0.0, V * np.sin(alpha + delta)])

        forces_0, _, _, _ = evaluator.model.evaluate(velb_0)
        forces_1, _, _, _ = evaluator.model.evaluate(velb_1)

        # Predict force change using Jacobian
        delta_forces_predicted = jacobian * delta
        delta_forces_actual = forces_1 - forces_0

        # Should be close (allowing for nonlinearity)
        relative_error = np.linalg.norm(delta_forces_predicted - delta_forces_actual)
        relative_error /= np.linalg.norm(delta_forces_actual) + 1e-10

        assert relative_error < 0.02, f"Jacobian prediction error too large: {relative_error}"

    def test_trajectory_reference_has_aero_evaluator(self, trajectory: TrajectoryReference):
        """Test that TrajectoryReference has aero_evaluator attribute."""
        assert hasattr(trajectory, "aero_evaluator"), "TrajectoryReference missing aero_evaluator attribute"
        assert isinstance(trajectory.aero_evaluator, AeroForceEvaluator), \
            "aero_evaluator is not an instance of AeroForceEvaluator"

    def test_trajectory_reference_aero_evaluator_usable(self, trajectory: TrajectoryReference):
        """Test that TrajectoryReference.aero_evaluator can compute Jacobians."""
        # Use the aero_evaluator to compute a Jacobian
        V = 10.0
        alpha = 0.1
        jacobian = trajectory.aero_evaluator.compute_body_forces_jacobian_alpha(V, alpha)

        assert jacobian.shape == (3,), f"Expected (3,), got {jacobian.shape}"
        assert not np.any(np.isnan(jacobian)), "Jacobian contains NaN values"
        assert not np.any(np.isinf(jacobian)), "Jacobian contains Inf values"


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v"])
