"""Test script for ErrStateLmpcController."""

import numpy as np
from feature.lu_mpc.controller import load_trajectory_csv, TrajectoryReference, ErrStateLmpcController


def test_err_state_lmpc():
    """Test basic functionality of ErrStateLmpcController."""
    print("=" * 60)
    print("Testing ErrStateLmpcController")
    print("=" * 60)

    # Load a test trajectory
    csv_path = "/home/rec/AP-PnC/artifacts/simulations/temp_data/planning_circle_vel12.0_thrust-4.5-4.5.csv"
    print(f"\nLoading trajectory from: {csv_path}")

    trajectory_data = load_trajectory_csv(csv_path)
    trajectory = TrajectoryReference(trajectory_data)

    # Create controller
    print("\nCreating controller...")
    controller = ErrStateLmpcController(
        trajectory=trajectory,
        is_hover_task=False,  # Tracking mode
        ctrl_dt=0.1,
    )

    # Test with a sample state
    print("\nTesting compute_control()...")

    # Create a sample simulation state
    # [p(3), v(3), r(3), q(4), u(4)]
    sim_state = np.array([
        0.0, 0.0, -30.0,      # Position
        12.0, 0.0, 0.0,       # Velocity
        0.0, 0.0, 0.0,        # Angular rates
        1.0, 0.0, 0.0, 0.0,   # Quaternion (identity)
        2.45, 2.45, 2.45, 2.45,  # Motor thrusts
    ])

    print(f"Simulation state: {sim_state}")

    # Compute control
    control_output = controller.compute_control(sim_state)

    print(f"\nControl output: {control_output}")
    print(f"Control output shape: {control_output.shape}")
    print(f"Control output type: {control_output.dtype}")

    # Verify output shape
    assert control_output.shape == (4,), f"Expected shape (4,), got {control_output.shape}"

    # Verify output is finite
    assert np.all(np.isfinite(control_output)), "Control output contains non-finite values"

    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)

    # Test reset functionality
    print("\nTesting reset()...")
    controller.reset()
    print("Reset successful")

    # Test hover mode
    print("\nTesting hover mode...")
    controller_hover = ErrStateLmpcController(
        trajectory=trajectory,
        is_hover_task=True,
        ctrl_dt=0.1,
    )

    control_hover = controller_hover.compute_control(sim_state)
    print(f"Hover mode control output: {control_hover}")
    assert control_hover.shape == (4,), "Hover mode control output shape incorrect"

    print("\n" + "=" * 60)
    print("All hover mode tests passed!")
    print("=" * 60)


if __name__ == "__main__":
    test_err_state_lmpc()
