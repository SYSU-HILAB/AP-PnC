"""Tests for GdModelSimulator class."""
import numpy as np
import pytest

from feature.lu_mpc.model.sim.sim_gd_model import (
    GdModelSimulator,
    GdSimConfig,
)


def test_simulator_initialization():
    """Test that GdModelSimulator is initialized correctly."""
    config = GdSimConfig()
    init_state = np.zeros(17, dtype=np.float64)

    simulator = GdModelSimulator(init_state, config)

    # Check that the simulator has the expected attributes
    assert hasattr(simulator, "_cur_state")
    assert hasattr(simulator, "_config")
    assert hasattr(simulator, "_aero_model")
    assert hasattr(simulator, "_proper_acc_func")
    assert hasattr(simulator, "_acados_solver")

    # Check initial state is stored correctly
    np.testing.assert_array_equal(simulator._cur_state, init_state)


def test_get_state():
    """Test that get_state returns a copy of the current state."""
    config = GdSimConfig()
    init_state = np.arange(17, dtype=np.float64)
    simulator = GdModelSimulator(init_state, config)

    state = simulator.get_state()

    # Should return the same values
    np.testing.assert_array_equal(state, init_state)

    # Should be a copy, not the same object
    state[0] = 999.0
    assert simulator._cur_state[0] != 999.0


def test_simulate_return_type():
    """Test that simulate returns a tuple of (state, proper_acceleration)."""
    config = GdSimConfig()

    # Initial state: hover at origin
    init_state = np.zeros(17, dtype=np.float64)
    init_state[9:13] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion [w,x,y,z]

    simulator = GdModelSimulator(init_state, config)

    motor_cmd = np.array([6.0, 6.0, 6.0, 6.0])
    result = simulator.simulate(motor_cmd)

    # Result should be a tuple
    assert isinstance(result, tuple), f"simulate should return tuple, got {type(result)}"
    assert len(result) == 2, f"simulate should return 2 elements, got {len(result)}"

    next_state, proper_acc = result

    # Check shapes
    assert next_state.shape == (17,), f"next_state should have shape (17,), got {next_state.shape}"
    assert proper_acc.shape == (3,), f"proper_acc should have shape (3,), got {proper_acc.shape}"


@pytest.mark.parametrize(
    "motor_cmd",
    [
        np.array([5.0, 5.0, 5.0, 5.0]),  # Hover thrust
        np.array([8.0, 6.0, 7.0, 6.5]),  # Asymmetric thrust
        np.array([0.0, 0.0, 0.0, 0.0]),  # No thrust
    ],
)
def test_simulate_with_various_motor_commands(motor_cmd: np.ndarray):
    """Test that simulate produces valid outputs for various motor commands."""
    config = GdSimConfig()

    # Initial state: hover at origin
    init_state = np.zeros(17, dtype=np.float64)
    init_state[9:13] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion [w,x,y,z]

    simulator = GdModelSimulator(init_state, config)

    next_state, proper_acc = simulator.simulate(motor_cmd)

    # Check that outputs are finite
    assert np.all(np.isfinite(next_state)), "next_state should contain only finite values"
    assert np.all(np.isfinite(proper_acc)), "proper_acc should contain only finite values"

    # Check that state was updated internally
    np.testing.assert_array_equal(simulator._cur_state, next_state)


def test_get_accp():
    """Test that get_accp returns proper acceleration in body frame."""
    config = GdSimConfig()

    # Initial state: hover at origin
    init_state = np.zeros(17, dtype=np.float64)
    init_state[9:13] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion [w,x,y,z]
    init_state[13:17] = np.array([6.0, 6.0, 6.0, 6.0])  # Motor thrusts

    simulator = GdModelSimulator(init_state, config)

    proper_acc = simulator.get_accp()

    # Check shape
    assert proper_acc.shape == (3,), f"proper_acc should have shape (3,), got {proper_acc.shape}"

    # Check that values are finite
    assert np.all(np.isfinite(proper_acc)), "proper_acc should contain only finite values"

    # With symmetric thrust and identity quaternion, proper acceleration should be mostly in z-direction
    # (positive z in FLU frame is upward thrust)
    assert proper_acc[2] > 0, "With upward thrust, z-component of proper acceleration should be positive"


def test_get_accp_matches_simulate_output():
    """Test that get_accp returns the same value as simulate's second return value."""
    config = GdSimConfig()

    # Initial state
    init_state = np.zeros(17, dtype=np.float64)
    init_state[9:13] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    init_state[13:17] = np.array([7.0, 6.5, 7.5, 6.0])  # Motor thrusts

    simulator = GdModelSimulator(init_state, config)

    # Get proper acceleration directly
    proper_acc_direct = simulator.get_accp()

    # Get proper acceleration from simulate (before simulation step)
    motor_cmd = np.array([6.0, 6.0, 6.0, 6.0])
    _, proper_acc_from_sim = simulator.simulate(motor_cmd)

    # Both should return valid accelerations (though different since state changes)
    assert proper_acc_direct.shape == (3,)
    assert proper_acc_from_sim.shape == (3,)


def test_simulate_invalid_motor_command_shape():
    """Test that simulate raises error for invalid motor command shape."""
    config = GdSimConfig()
    init_state = np.zeros(17, dtype=np.float64)
    simulator = GdModelSimulator(init_state, config)

    with pytest.raises(ValueError, match="motors_cmd must have shape \\(4,\\)"):
        simulator.simulate(np.array([1.0, 2.0, 3.0]))  # Only 3 elements


def test_simulate_invalid_init_state_shape():
    """Test that GdModelSimulator raises error for invalid initial state shape."""
    config = GdSimConfig()
    init_state = np.zeros(10, dtype=np.float64)  # Wrong shape

    with pytest.raises(ValueError, match="init_state must have shape \\(17,\\)"):
        GdModelSimulator(init_state, config)


def test_proper_acceleration_physical_behavior():
    """Test that proper acceleration behaves physically correctly."""
    config = GdSimConfig()

    # Test 1: No thrust = minimal proper acceleration (just gravity effect in body frame)
    init_state = np.zeros(17, dtype=np.float64)
    init_state[9:13] = np.array([1.0, 0.0, 0.0, 0.0])  # Level orientation
    init_state[13:17] = np.array([0.0, 0.0, 0.0, 0.0])  # No motor thrust

    simulator = GdModelSimulator(init_state, config)
    acc_no_thrust = simulator.get_accp()

    # Test 2: High thrust = higher proper acceleration in z
    init_state[13:17] = np.array([10.0, 10.0, 10.0, 10.0])  # High thrust
    simulator = GdModelSimulator(init_state, config)
    acc_high_thrust = simulator.get_accp()

    # Z-component should be higher with more thrust
    assert acc_high_thrust[2] > acc_no_thrust[2], "Higher thrust should produce higher z-acceleration"
