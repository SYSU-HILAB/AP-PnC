"""
Orchestrator Bridge for the AP-PnC TUI Dashboard.

Binds the TUI to the underlying simulation/tracker engine, managing
simulation stepping, scenario switching, wind disturbance, and config writes.
"""

from __future__ import annotations

import threading

import yaml

from ap_pnc.env import get_git_root, setup_env
from ap_pnc.lu_mpc.experiments.headless_orchestrator import (
    HeadlessOrchestrator,
    OrchestratorSnapshot,
)
from ap_pnc.lu_mpc.experiments.open_scenarios import SCENARIOS, ensure_open_scenario


class OrchestratorBridge:
    """Manages simulation life cycle, scenarios, and config synchronization."""

    def __init__(self, initial_scenario: str = "open_circle"):
        setup_env()
        self.lock = threading.Lock()
        self.git_root = get_git_root()
        self.scenario_key = initial_scenario
        self.wind_side = 0.0
        self.wind_longitudinal = 0.0
        self.target_speed = SCENARIOS.get(initial_scenario, {}).get("speed", 10.0)
        self.is_running = False
        self.sim_dt = 0.02
        self.orchestrator: HeadlessOrchestrator | None = None
        self.last_snapshot: OrchestratorSnapshot | None = None
        self.last_write_status: str = "Ready"

        with self.lock:
            self._init_orchestrator()

    def _init_orchestrator(self) -> None:
        """Create or recreate the HeadlessOrchestrator instance."""
        csv_path = ensure_open_scenario(self.scenario_key, speed=self.target_speed)
        self.orchestrator = HeadlessOrchestrator(
            trajectory_csv=str(csv_path),
            wind_side=self.wind_side,
            wind_longitudinal=self.wind_longitudinal,
            velocity=self.target_speed,
            sim_dt=self.sim_dt,
            controller_type="nmpc",
            verbose=False,
        )
        self.last_snapshot = self.orchestrator.get_snapshot()

    def switch_scenario(self, scenario_key: str) -> None:
        """Switch to a different open scenario."""
        with self.lock:
            if scenario_key in SCENARIOS and scenario_key != self.scenario_key:
                self.scenario_key = scenario_key
                self.target_speed = SCENARIOS[scenario_key]["speed"]
                self._init_orchestrator()
                self.last_write_status = f"Switched to {scenario_key}"

    def set_wind(self, wind_side: float, wind_longitudinal: float) -> None:
        """Adjust wind disturbance components."""
        with self.lock:
            self.wind_side = float(wind_side)
            self.wind_longitudinal = float(wind_longitudinal)
            self._init_orchestrator()
            self.last_write_status = (
                f"Wind set to [{self.wind_side:.1f}, {self.wind_longitudinal:.1f}] m/s"
            )

    def set_speed(self, speed: float) -> None:
        """Set target speed and recreate scenario."""
        with self.lock:
            self.target_speed = max(1.0, float(speed))
            # Re-generate trajectory with scaled speed
            ensure_open_scenario(self.scenario_key, speed=self.target_speed)
            self._init_orchestrator()
            self.last_write_status = f"Speed set to {self.target_speed:.1f} m/s"

    def step(self) -> OrchestratorSnapshot:
        """Advance one simulation timestep."""
        with self.lock:
            if self.orchestrator is None:
                self._init_orchestrator()

            assert self.orchestrator is not None
            if not self.orchestrator.controller.is_trajectory_complete:
                self.orchestrator.step()
            self.last_snapshot = self.orchestrator.get_snapshot()
            return self.last_snapshot

    def reset(self) -> OrchestratorSnapshot:
        """Reset simulation back to start."""
        with self.lock:
            self._init_orchestrator()
            assert self.orchestrator is not None
            self.last_snapshot = self.orchestrator.get_snapshot()
            self.last_write_status = "Reset to start"
            return self.last_snapshot

    def get_snapshot_dict(self) -> dict | None:
        """Return thread-safe JSON-serializable snapshot."""
        with self.lock:
            if self.last_snapshot is None and self.orchestrator is not None:
                self.last_snapshot = self.orchestrator.get_snapshot()
            if self.last_snapshot is None:
                return None
            data = self.last_snapshot.to_dict()
            data["scenario_key"] = self.scenario_key
            data["target_speed"] = self.target_speed
            data["wind"] = [self.wind_side, self.wind_longitudinal]
            data["is_running"] = self.is_running
            data["last_write_status"] = self.last_write_status
            return data

    def apply_writes_to_config(self) -> str:
        """
        Synchronize current scenario parameters to bringup/config/planning.yaml
        and infra/sim_infra/simple_sim/config/simple_sim.yaml.
        """
        messages = []

        # 1. Write to bringup/config/planning.yaml
        planning_yaml = self.git_root / "core" / "bringup" / "config" / "planning.yaml"
        if planning_yaml.exists():
            try:
                data = yaml.safe_load(planning_yaml.read_text()) or {}
                prob = data.setdefault("problem_formulation", {})
                cost = prob.setdefault("cost", {})
                basic = prob.setdefault("basic_traj", {})

                traj_type_map = {"open_lemniscate": 1, "open_circle": 2, "open_line": 4}
                basic["type"] = traj_type_map.get(self.scenario_key, 2)
                basic["radius"] = SCENARIOS.get(self.scenario_key, {}).get("radius", 10.0)
                cost["v_max"] = float(self.target_speed)

                planning_yaml.write_text(yaml.safe_dump(data, sort_keys=False))
                messages.append("planning.yaml (type/radius/vmax)")
            except Exception as e:
                messages.append(f"planning.yaml error: {e}")

        # 2. Write to simulations/simple_sim/config/simple_sim.yaml (open scenario)
        sim_yaml = self.git_root / "infra" / "sim_infra" / "simple_sim" / "config" / "simple_sim.yaml"
        if sim_yaml.exists():
            try:
                sim_data = yaml.safe_load(sim_yaml.read_text()) or {}
                root_params = sim_data.setdefault("/simple_sim", {}).setdefault(
                    "ros__parameters", {}
                )
                # For open scenario: no pillars, no bounding shell
                root_params["build_shell"] = False
                scene = root_params.setdefault("scene", {})
                scene["pillar_count"] = 0
                aero = root_params.setdefault("aero", {})
                aero["wind"] = [float(self.wind_longitudinal), float(self.wind_side), 0.0]

                sim_yaml.write_text(yaml.safe_dump(sim_data, sort_keys=False))
                messages.append("simple_sim.yaml (open collision-free)")
            except Exception as e:
                messages.append(f"simple_sim.yaml error: {e}")

        self.last_write_status = f"Wrote: {', '.join(messages)}"
        return self.last_write_status
