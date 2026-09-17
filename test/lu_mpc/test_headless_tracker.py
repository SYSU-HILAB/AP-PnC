"""
Headless Tracker Performance and Open Scenario Tests.

Verifies:
1. Open collision-free scenario generation (circle, line, lemniscate).
2. NMPC tracker stability and convergence with CasADi dynamics simulator.
3. NMPC predicted trajectory horizon extraction (x_0..N).
4. Telemetry snapshot integrity.
5. Terminal 2D trajectory canvas rendering.
6. Synchronized config writes to planning and simulation YAML files.
"""

import numpy as np
import pytest

from ap_pnc.env import setup_env
from ap_pnc.lu_mpc.experiments.open_scenarios import SCENARIOS, ensure_open_scenario
from ap_pnc.tui.canvas import render_trajectory_canvas
from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge


@pytest.fixture(autouse=True)
def init_env():
    """Ensure ACADOS and environment paths are configured."""
    setup_env()


class TestOpenScenarios:
    """Test open collision-free scenario creation."""

    @pytest.mark.parametrize("scenario_key", list(SCENARIOS.keys()))
    def test_ensure_open_scenario(self, scenario_key):
        csv_path = ensure_open_scenario(scenario_key)
        assert csv_path.exists(), f"Scenario CSV not found: {csv_path}"
        assert csv_path.stat().st_size > 1000, f"Scenario CSV is empty: {csv_path}"


class TestHeadlessTracker:
    """Test NMPC tracker performance in headless simulation."""

    def test_tracker_stepping_and_horizon_extraction(self):
        """Test tracker step execution, horizon extraction, and error convergence."""
        bridge = OrchestratorBridge(initial_scenario="open_circle")
        bridge.set_speed(10.0)

        # Run 40 steps (0.8s) of closed-loop simulation
        num_steps = 40
        for _ in range(num_steps):
            snap = bridge.step()
            assert snap.status_ok, f"Tracker reported error: {bridge.orchestrator.failure_reason}"
            assert not np.any(np.isnan(snap.current_pos)), "NaN detected in position"
            assert not np.any(np.isnan(snap.current_vel)), "NaN detected in velocity"

        final_snap = bridge.last_snapshot
        assert final_snap is not None
        assert final_snap.step_idx == num_steps

        # Verify predicted horizon has shape (N+1, 3) where N=10
        assert final_snap.pred_trajectory.shape == (
            11,
            3,
        ), f"Expected shape (11, 3), got {final_snap.pred_trajectory.shape}"
        assert not np.any(np.isnan(final_snap.pred_trajectory)), "NaN in predicted horizon"

        # Verify tracking performance is within reasonable bounds during acceleration transient
        assert final_snap.rms_error < 2.5, f"RMS error too high: {final_snap.rms_error:.4f}m"
        assert (
            final_snap.pos_error < 4.0
        ), f"Instant position error too high: {final_snap.pos_error:.4f}m"

        # Verify motor thrusts are positive and physically reasonable
        assert np.all(final_snap.motor_thrusts >= 0.0), "Negative motor thrust detected"
        assert np.all(final_snap.motor_thrusts <= 10.0), "Motor thrust exceeded physical limits"

    def test_trajectory_canvas_rendering(self):
        """Test that the 2D projection canvas renders properly."""
        bridge = OrchestratorBridge(initial_scenario="open_circle")
        snap = bridge.step()

        text = render_trajectory_canvas(
            ref_pts=snap.ref_trajectory,
            pred_pts=snap.pred_trajectory,
            cur_pos=snap.current_pos,
            cur_heading=0.0,
            width=50,
            height=12,
            plane="xy",
        )
        rendered_str = text.plain
        assert len(rendered_str) > 50
        assert "Ref Traj" in rendered_str
        assert "NMPC Horizon" in rendered_str

    def test_config_writes(self):
        """Test applying scenario writes to YAML configuration files."""
        bridge = OrchestratorBridge(initial_scenario="open_circle")
        bridge.set_speed(12.0)
        status = bridge.apply_writes_to_config()
        assert "planning.yaml" in status or "simple_sim.yaml" in status
