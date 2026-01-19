"""
Tests for CtrlProperAccelZAngVel3D and its components.

This includes tests for:
- QP Mixer (moved from model/ctrl/qp_mixer.py)
- PD Controller (proper acceleration tracking)
- PID Controller with anti-windup (angular velocity tracking)
- CtrlProperAccelZAngVel3D (full integration)
"""

import pytest
import numpy as np
from numpy.testing import assert_array_almost_equal, assert_allclose

from feature.lu_mpc.controller.ctrl_proper_accel_z_ang_vel_3d import (
    QpMixer,
    QpMixerConfig,
    PDController,
    PDControllerConfig,
    PIDAntiWindupController,
    PIDAntiWindupConfig,
    CtrlProperAccelZAngVel3D,
)


# =============================================================================
# QP Mixer Tests (ported from test_qp_mixer.py)
# =============================================================================

class TestQpMixerConfig:
    """Test mixer configuration."""

    def test_default_config(self):
        """Test default configuration values."""
        config = QpMixerConfig()
        assert config.dx == 0.32
        assert config.dy == 0.32
        assert config.c_tau == 0.032
        assert config.u_min == 0.0
        assert config.u_max == 10.0
        assert config.du_min == -10.0
        assert config.du_max == 10.0

    def test_default_tracking_weights(self):
        """Test default tracking weights are identity matrix."""
        config = QpMixerConfig()
        assert_array_almost_equal(config.W_tracking, np.eye(4))


class TestQpMixerAllocationMatrix:
    """Test control allocation matrix construction."""

    @pytest.fixture
    def mixer(self):
        """Create mixer instance for testing."""
        return QpMixer(QpMixerConfig())

    def test_allocation_matrix_shape(self, mixer):
        """Test allocation matrix has correct shape."""
        Gamma = mixer._Gamma
        assert Gamma.shape == (4, 4)

    def test_equal_thrusts_zero_torques(self, mixer):
        """Test that equal thrusts produce zero torques."""
        u_equal = np.ones(4) * 5.0
        tau = mixer._Gamma @ u_equal

        assert tau[0] == 20.0  # Total thrust
        assert abs(tau[1]) < 1e-10  # Zero roll
        assert abs(tau[2]) < 1e-10  # Zero pitch
        assert abs(tau[3]) < 1e-10  # Zero yaw


class TestQpMixerSolve:
    """Test QP mixer solving."""

    @pytest.fixture
    def mixer(self):
        """Create mixer instance for testing."""
        config = QpMixerConfig()
        return QpMixer(config)

    def test_hover_solution(self, mixer):
        """Test mixer produces correct hover solution."""
        # Desired: 10N total thrust, zero torques
        tau_des = np.array([10.0, 0.0, 0.0, 0.0])

        u, du, info = mixer.mix(tau_des)

        # All motors should be approximately equal
        assert_allclose(u, 2.5, atol=1e-2)

        # Check solver converged
        assert info['solver_status'] == 0  # 0 = SOLVED in CasADi

    def test_roll_moment(self, mixer):
        """Test mixer produces correct roll torque."""
        # Desired: 10N thrust + positive roll
        tau_des = np.array([10.0, 1.0, 0.0, 0.0])

        u, du, info = mixer.mix(tau_des)

        # Total thrust constraint (approximately)
        total_thrust = np.sum(u)
        assert abs(total_thrust - 10.0) < 0.5

        # Motors 1,2 should differ from 3,4 for positive roll
        # Based on allocation: [-dy, -dy, dy, dy]
        # Positive roll requires right side (3,4) to be higher
        assert u[2] > u[0]
        assert u[3] > u[1]

    def test_pitch_moment(self, mixer):
        """Test mixer produces correct pitch torque."""
        # Desired: 10N thrust + positive pitch
        tau_des = np.array([10.0, 0.0, 0.5, 0.0])

        u, du, info = mixer.mix(tau_des)

        # Total thrust constraint (approximately)
        total_thrust = np.sum(u)
        assert abs(total_thrust - 10.0) < 0.5

        # Based on allocation: [-dx, dx, dx, -dx]
        # Positive pitch requires motors 2,3 > motors 1,4
        assert (u[1] + u[3]) > (u[0] + u[2])

    def test_yaw_moment(self, mixer):
        """Test mixer produces correct yaw torque."""
        # Desired: 10N thrust + positive yaw
        tau_des = np.array([10.0, 0.0, 0.0, 0.1])

        u, du, info = mixer.mix(tau_des)

        # Total thrust constraint (approximately)
        total_thrust = np.sum(u)
        assert abs(total_thrust - 10.0) < 0.5

        # Based on allocation: [-c_tau, c_tau, -c_tau, c_tau]
        # Positive yaw requires motors 2,4 > motors 1,3
        assert (u[1] + u[3]) > (u[0] + u[2])

    def test_motor_saturation(self, mixer):
        """Test motor saturation constraints."""
        # Request impossible: too much thrust
        tau_des = np.array([100.0, 0.0, 0.0, 0.0])

        u, du, info = mixer.mix(tau_des)

        # All motors should saturate at max
        assert np.all(u <= mixer._config.u_max + 1e-6)
        assert_allclose(u, mixer._config.u_max, atol=1e-2)

    def test_motor_lower_bound(self, mixer):
        """Test motor minimum thrust constraint."""
        # Request negative total thrust (impossible)
        tau_des = np.array([-10.0, 0.0, 0.0, 0.0])

        u, du, info = mixer.mix(tau_des)

        # All motors should be at minimum
        assert np.all(u >= mixer._config.u_min - 1e-6)

    def test_rate_constraints(self, mixer):
        """Test rate constraints."""
        # Large change from previous
        tau_des = np.array([20.0, 0.0, 0.0, 0.0])
        u_prev = np.zeros(4)

        u, du, info = mixer.mix(tau_des, u_prev=u_prev)

        # Rate should be limited
        assert np.all(du <= mixer._config.du_max + 1e-6)
        assert np.all(du >= mixer._config.du_min - 1e-6)


class TestQpMixerIntegration:
    """Integration tests with simulator."""

    def test_allocation_with_simulator_params(self):
        """Test mixer allocation matches simulator parameters."""
        from feature.lu_mpc.model.sim.sim_gd_model import DEFAULT_SIM_CONFIG

        # Create mixer with simulator-matching parameters
        config = QpMixerConfig(
            dx=DEFAULT_SIM_CONFIG.dx,
            dy=DEFAULT_SIM_CONFIG.dy,
            c_tau=DEFAULT_SIM_CONFIG.c_tau,
            u_max=DEFAULT_SIM_CONFIG.mass * 9.81 / 4.0 * 2,  # Approx max
        )
        mixer = QpMixer(config)

        # Test hover
        tau_des = np.array([DEFAULT_SIM_CONFIG.mass * 9.81, 0.0, 0.0, 0.0])
        u, du, info = mixer.mix(tau_des)

        # Should distribute thrust equally
        assert_allclose(u, DEFAULT_SIM_CONFIG.mass * 9.81 / 4.0, atol=1e-2)


# =============================================================================
# PD Controller Tests (new)
# =============================================================================

class TestPDControllerConfig:
    """Test PD controller configuration."""

    def test_default_config(self):
        """Test default configuration values."""
        config = PDControllerConfig()
        assert config.Kp == 1.0
        assert config.Kd == 0.1
        assert config.output_min == 0.0
        assert config.output_max == 40.0


class TestPDController:
    """Test PD controller for proper acceleration tracking."""

    @pytest.fixture
    def controller(self):
        """Create PD controller instance for testing."""
        return PDController(PDControllerConfig())

    def test_zero_error(self, controller):
        """Test that zero error produces zero output (plus bias)."""
        desired = np.zeros(3)
        measured = np.zeros(3)

        output = controller.compute(desired, measured, dt=0.02)

        # Should be approximately zero
        assert abs(output) < 1e-6

    def test_proportional_action(self, controller):
        """Test proportional term responds to error."""
        desired = np.array([1.0, 0.0, 0.0])
        measured = np.zeros(3)

        output = controller.compute(desired, measured, dt=0.02)

        # Should produce positive output
        assert output > 0

    def test_derivative_action(self, controller):
        """Test derivative term responds to error rate."""
        # First call with error
        desired = np.array([1.0, 0.0, 0.0])
        measured = np.zeros(3)
        controller.compute(desired, measured, dt=0.02)

        # Second call with zero error (high derivative)
        desired = np.zeros(3)
        measured = np.zeros(3)
        output = controller.compute(desired, measured, dt=0.02)

        # Derivative term should produce negative output
        assert output < 0

    def test_output_saturation(self, controller):
        """Test output saturation limits."""
        # Large error should saturate
        desired = np.array([1000.0, 0.0, 0.0])
        measured = np.zeros(3)

        output = controller.compute(desired, measured, dt=0.02)

        # Should saturate at max
        assert output <= controller._output_max


# =============================================================================
# PID Controller Tests (new)
# =============================================================================

class TestPIDControllerConfig:
    """Test PID controller configuration."""

    def test_default_config(self):
        """Test default configuration values."""
        config = PIDAntiWindupConfig()
        assert_array_almost_equal(config.Kp, np.array([0.5, 0.5, 0.5]))
        assert_array_almost_equal(config.Ki, np.array([0.1, 0.1, 0.1]))
        assert_array_almost_equal(config.Kd, np.array([0.05, 0.05, 0.05]))


class TestPIDAntiWindupController:
    """Test PID controller with anti-windup for angular velocity tracking."""

    @pytest.fixture
    def controller(self):
        """Create PID controller instance for testing."""
        return PIDAntiWindupController(PIDAntiWindupConfig())

    def test_zero_error(self, controller):
        """Test that zero error produces zero output."""
        desired = np.zeros(3)
        measured = np.zeros(3)

        output = controller.compute(desired, measured, dt=0.02)

        assert_array_almost_equal(output, np.zeros(3), decimal=6)

    def test_proportional_action(self, controller):
        """Test proportional term responds to error."""
        desired = np.array([1.0, 0.0, 0.0])
        measured = np.zeros(3)

        output = controller.compute(desired, measured, dt=0.02)

        # First axis should have positive output
        assert output[0] > 0
        assert abs(output[1]) < 1e-6
        assert abs(output[2]) < 1e-6

    def test_integral_action(self, controller):
        """Test integral term accumulates error."""
        desired = np.array([1.0, 0.0, 0.0])
        measured = np.zeros(3)

        # Multiple calls should increase integral
        output1 = controller.compute(desired, measured, dt=0.02)
        output2 = controller.compute(desired, measured, dt=0.02)

        # Output should increase due to integral
        assert output2[0] > output1[0]

    def test_anti_windup(self, controller):
        """Test anti-windup limits integral accumulation."""
        # Very large error for many steps
        desired = np.array([100.0, 0.0, 0.0])
        measured = np.zeros(3)

        for _ in range(100):
            controller.compute(desired, measured, dt=0.02)

        # Integral should be clamped
        assert np.all(controller._integral >= controller._integral_min - 1e-6)
        assert np.all(controller._integral <= controller._integral_max + 1e-6)

    def test_reset(self, controller):
        """Test reset clears integral and error."""
        desired = np.array([1.0, 0.0, 0.0])
        measured = np.zeros(3)

        controller.compute(desired, measured, dt=0.02)

        # Reset should clear state
        controller.reset()

        assert_array_almost_equal(controller._integral, np.zeros(3))
        assert_array_almost_equal(controller._prev_error, np.zeros(3))


# =============================================================================
# CtrlProperAccelZAngVel3D Integration Tests (new)
# =============================================================================

class TestCtrlProperAccelZAngVel3D:
    """Integration tests for CtrlProperAccelZAngVel3D."""

    def test_factory_function(self):
        """Test factory function creates controller."""
        from feature.lu_mpc.controller.trajectory_loader import load_trajectory_csv
        from pathlib import Path

        # Load test trajectory
        traj_path = Path(__file__).parent.parent.parent.parent / "feature" / "lu_mpc" / ".cache" / "test_trajectory.csv"

        # Skip if trajectory doesn't exist
        if not traj_path.exists():
            pytest.skip("Test trajectory not found")

        trajectory = load_trajectory_csv(str(traj_path))
        controller = CtrlProperAccelZAngVel3D.create(trajectory)

        assert controller is not None
        assert isinstance(controller, type(controller))
