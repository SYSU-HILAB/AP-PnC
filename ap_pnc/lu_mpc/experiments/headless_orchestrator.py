"""
Headless MPC Orchestrator for Automated Wind Performance Testing

This module provides a ROS2-free simulation orchestrator for automated testing.
It follows the same control flow as nmpc_orchestrator.py but without visualization
or ROS dependencies, focusing on metrics collection for performance analysis.

Components:
    - GdModelSimulator: CasADi-based dynamics simulator
    - ProposedNmpcAngVelActuatedController: NMPC controller for trajectory tracking
    - MetricsCollector: Position error tracking and statistics

Usage:
    orchestrator = HeadlessOrchestrator(
        trajectory_csv="path/to/trajectory.csv",
        wind_side=1.0,           # 1 m/s wind from below (along body_z)
        wind_longitudinal=2.0,   # 2 m/s wind from right (along body_y)
    )
    orchestrator.run()
    metrics = orchestrator.get_metrics()
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from ..controller.ctrl_proposed_nmpc_ang_vel_actuated import (
        ProposedNmpcAngVelActuatedController,
    )
    from ..model.sim.sim_gd_model import GdModelSimulator

from ..controller.config import ProposedMpcAngVelActuatedCfg
from ..controller.ctrl_proposed_nmpc_ang_vel_actuated import ProposedNmpcAngVelActuatedController
from ..controller.trajectory_loader import TrajectoryReference
from ..model.sim.sim_gd_model import GdModelSimulator, GdSimConfig


@dataclass
class MetricsResult:
    """Performance metrics from a simulation run."""
    velocity: float
    wind_side: float
    wind_longitudinal: float
    rms_error: float
    max_error: float
    final_error: float
    success: bool
    num_steps: int
    trajectory_duration: float


@dataclass
class OrchestratorSnapshot:
    """Real-time telemetry and state snapshot for TUI / monitoring."""
    step_idx: int
    sim_time: float
    current_pos: np.ndarray       # (3,) [x, y, z]
    current_vel: np.ndarray       # (3,) [vx, vy, vz]
    current_quat: np.ndarray      # (4,) [qw, qx, qy, qz]
    current_rpy: np.ndarray       # (3,) [roll, pitch, yaw] deg
    body_rates: np.ndarray        # (3,) [p, q, r] rad/s
    motor_thrusts: np.ndarray     # (4,) [u0, u1, u2, u3] N
    total_thrust: float           # N
    rates_des: np.ndarray         # (3,) [p, q, r] rad/s
    pitch_des: float              # deg
    ref_pos: np.ndarray           # (3,) [x, y, z]
    ref_vel: np.ndarray           # (3,) [vx, vy, vz]
    pos_error: float              # m
    rms_error: float              # m
    max_error: float              # m
    pred_trajectory: np.ndarray   # (N+1, 3)
    ref_trajectory: np.ndarray    # (M, 3)
    is_complete: bool
    status_ok: bool

    def to_dict(self) -> dict:
        """Convert snapshot to JSON-serializable dictionary."""
        return {
            "step_idx": int(self.step_idx),
            "sim_time": round(float(self.sim_time), 3),
            "current_pos": [round(float(v), 3) for v in self.current_pos],
            "current_vel": [round(float(v), 3) for v in self.current_vel],
            "current_quat": [round(float(v), 4) for v in self.current_quat],
            "current_rpy": [round(float(v), 2) for v in self.current_rpy],
            "body_rates": [round(float(v), 3) for v in self.body_rates],
            "motor_thrusts": [round(float(v), 2) for v in self.motor_thrusts],
            "total_thrust": round(float(self.total_thrust), 2),
            "rates_des": [round(float(v), 3) for v in self.rates_des],
            "pitch_des": round(float(self.pitch_des), 2),
            "ref_pos": [round(float(v), 3) for v in self.ref_pos],
            "ref_vel": [round(float(v), 3) for v in self.ref_vel],
            "pos_error": round(float(self.pos_error), 4),
            "rms_error": round(float(self.rms_error), 4),
            "max_error": round(float(self.max_error), 4),
            "pred_trajectory": [[round(float(coord), 3) for coord in pt] for pt in self.pred_trajectory],
            "ref_trajectory": [[round(float(coord), 3) for coord in pt] for pt in self.ref_trajectory[::2]],
            "is_complete": bool(self.is_complete),
            "status_ok": bool(self.status_ok),
        }


class MetricsCollector:
    """Collects and computes tracking error metrics."""

    def __init__(self):
        self.position_errors: list[float] = []
        self.velocity_errors: list[float] = []
        self.timestamps: list[float] = []
        self.reference_positions: list[np.ndarray] = []
        self.actual_positions: list[np.ndarray] = []

    def add_sample(
        self,
        ref_pos: np.ndarray,
        actual_pos: np.ndarray,
        ref_vel: np.ndarray,
        actual_vel: np.ndarray,
        timestamp: float,
    ) -> None:
        """Add a sample to the metrics collection."""
        pos_error = np.linalg.norm(ref_pos - actual_pos)
        vel_error = np.linalg.norm(ref_vel - actual_vel)

        self.position_errors.append(pos_error)
        self.velocity_errors.append(vel_error)
        self.timestamps.append(timestamp)
        self.reference_positions.append(ref_pos.copy())
        self.actual_positions.append(actual_pos.copy())

    def compute_metrics(self) -> dict[str, float]:
        """Compute summary statistics from collected samples."""
        if not self.position_errors:
            return {
                "rms_error": float("inf"),
                "max_error": float("inf"),
                "final_error": float("inf"),
                "rms_velocity_error": float("inf"),
            }

        pos_errors = np.array(self.position_errors)
        vel_errors = np.array(self.velocity_errors)

        return {
            "rms_error": float(np.sqrt(np.mean(pos_errors**2))),
            "max_error": float(np.max(pos_errors)),
            "final_error": float(pos_errors[-1]),
            "rms_velocity_error": float(np.sqrt(np.mean(vel_errors**2))),
        }

    def get_error_series(self) -> dict[str, np.ndarray]:
        """Return the collected error series as arrays."""
        return {
            "position_errors": np.array(self.position_errors),
            "velocity_errors": np.array(self.velocity_errors),
            "timestamps": np.array(self.timestamps),
        }


class HeadlessOrchestrator:
    """
    Headless simulation orchestrator for automated wind performance testing.

    This class orchestrates the MPC controller and CasADi simulator to run
    closed-loop trajectory tracking simulations without ROS2 or visualization.
    It collects performance metrics for analysis.

    Key differences from nmpc_orchestrator.py:
    - No ROS2 dependencies (no rclpy.Node)
    - No visualization (no NmpcVisualizer)
    - Focus on metrics collection (MetricsCollector)
    - Returns results for analysis instead of publishing to topics
    """

    def __init__(
        self,
        trajectory_csv: str,
        wind_side: float = 0.0,
        wind_longitudinal: float = 0.0,
        velocity: float = 0.0,
        sim_dt: float = 0.02,
        controller_type: str = "nmpc",
        verbose: bool = False,
    ):
        """
        Initialize the headless orchestrator.

        Args:
            trajectory_csv: Path to trajectory CSV file
            wind_side: Wind speed along body_z axis (positive = wind from below) in m/s
            wind_longitudinal: Wind speed along body_y axis (positive = wind from right)
                             Normal to body_x/body_z plane and horizontal to world frame in m/s
            velocity: Target velocity in m/s (used for reporting, not computed from trajectory)
            sim_dt: Simulation integration timestep in seconds
            controller_type: Type of MPC controller - "nmpc" is supported.
            verbose: Whether to print progress messages
        """
        # Store parameters
        self.sim_dt = sim_dt
        self.wind_side = float(wind_side)
        self.wind_longitudinal = float(wind_longitudinal)
        self.velocity = float(velocity)
        self.controller_type = controller_type
        self.verbose = verbose

        # Wind will be computed from initial orientation in _setup_simulation
        self.wind_world: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Metrics collection
        self.metrics = MetricsCollector()
        self.success = False
        self.failure_reason = ""

        # Setup simulation and controller
        self._setup_simulation(trajectory_csv)

        if self.verbose:
            print(f"[HeadlessOrchestrator] Initialized with {self.controller_type.upper()} controller")
            print(f"  Sim dt: {sim_dt}s")
            print(f"  Wind side: {wind_side} m/s, longitudinal: {wind_longitudinal} m/s")
            print(f"  Trajectory: {trajectory_csv}")

    def _setup_simulation(self, trajectory_csv: str) -> None:
        """Initialize simulator and controller from trajectory CSV."""
        self.trajectory = TrajectoryReference(trajectory_csv)

        # Initial state from trajectory (WXYZ format)
        p0 = self.trajectory.position[0]
        v0 = self.trajectory.velocity[0]
        r0 = np.array([0.0, 0.0, 0.0])
        q0 = self.trajectory.quat_quad_flu[0]  # WXYZ format

        print(f"Initial position: {p0}")
        print(f"Initial orientation (quaternion WXYZ): {q0}")

        u0 = np.array([2.45, 2.45, 2.45, 2.45]) * 2.0
        init_state = np.concatenate([p0, v0, r0, q0, u0])

        wind_body = np.array([self.wind_longitudinal, self.wind_side, 0.0], dtype=np.float64)
        self.wind_world = self._quat_to_rot_matrix(q0) @ wind_body

        print("Initial wind in body frame:", wind_body)
        print(f"Computed wind in world frame: {self.wind_world}")
        # Create simulator with wind
        sim_config = GdSimConfig(sim_dt=self.sim_dt, wind_world=self.wind_world)
        self.simulator: GdModelSimulator = GdModelSimulator.create_gd_simulator(
            init_state, sim_config
        )

        # Create controller based on controller_type
        if self.controller_type == "nmpc":
            config = ProposedMpcAngVelActuatedCfg.create_high_performance(
                trajectory=self.trajectory
            )
            config.is_hover_task = False
            self.controller = ProposedNmpcAngVelActuatedController(config)
        else:
            raise ValueError(f"Unknown controller_type: {self.controller_type}")

        # Reference tracking
        self._ref_idx = 0
        self._current_time = 0.0

    def _sim_state_to_controller_state(self, sim_state: np.ndarray) -> np.ndarray:
        """
        Convert simulator state to controller state format.

        Simulator: [p(3), v(3), r(3), q(4), u(4)] = 17D
        nmpc:      [p(3), v(3), r(3), q(4), u(4), cz_rate(1)] = 18D

        Args:
            sim_state: State from GdModelSimulator (17D)

        Returns:
            Controller state (18D for nmpc with appended cz_rate)
        """
        return sim_state

    @staticmethod
    def _quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to rotation matrix.

        Args:
            q: Quaternion in WXYZ format

        Returns:
            3x3 rotation matrix
        """
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # Normalize quaternion
        norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

        # Compute rotation matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float64)

        return R

    def step(self) -> bool:
        """
        Run one simulation step and collect metrics.

        Returns:
            True if simulation should continue, False if done
        """
        # Check if trajectory is complete
        if self.controller.is_trajectory_complete:
            if self.verbose:
                print("[HeadlessOrchestrator] Trajectory complete")
            self.success = True
            return False

        # Get reference state at current time
        ref_data = self.trajectory.get_reference_window(
            self._ref_idx, self.controller.ctrl_dt, self.controller._N
        )
        ref_pos = ref_data["position"][0]  # Current reference position
        ref_vel = ref_data["velocity"][0]  # Current reference velocity

        # Compute control (different interface for different controllers)
        sim_state = self.simulator.get_state()
        controller_state = self._sim_state_to_controller_state(sim_state)

        try:
            if self.controller_type == "nmpc":
                control = self.controller.compute_control(
                    sim_state=controller_state,
                    acc_p_meas=self.simulator.get_accp()
                )
            else:
                control = self.controller.compute_control(sim_state=controller_state)
        except RuntimeError as e:
            self.failure_reason = f"NMPC solver failed: {e}"
            if self.verbose:
                print(f"[HeadlessOrchestrator] {self.failure_reason}")
            return False

        # Step simulation
        try:
            self.simulator.simulate(control)
        except RuntimeError as e:
            self.failure_reason = f"Simulation failed: {e}"
            if self.verbose:
                print(f"[HeadlessOrchestrator] {self.failure_reason}")
            return False

        # Check for NaN
        state = self.simulator.get_state()
        if np.any(np.isnan(state)):
            self.failure_reason = "NaN detected in simulation state"
            if self.verbose:
                print(f"[HeadlessOrchestrator] {self.failure_reason}")
            return False

        # Collect metrics
        actual_pos = state[0:3]
        actual_vel = state[3:6]
        self.metrics.add_sample(ref_pos, actual_pos, ref_vel, actual_vel, self._current_time)

        # Update time and reference index
        self._current_time += self.sim_dt
        self._ref_idx += 1

        return True

    def run(self, max_steps: int = 10000) -> MetricsResult:
        """
        Run the simulation loop until completion or failure.

        Args:
            max_steps: Maximum number of simulation steps

        Returns:
            MetricsResult with performance statistics
        """
        if self.verbose:
            print("[HeadlessOrchestrator] Starting simulation...")

        for step_idx in range(max_steps):
            continue_sim = self.step()
            if not continue_sim:
                break

            if self.verbose and (step_idx + 1) % 100 == 0:
                rms_err = np.sqrt(np.mean(np.array(self.metrics.position_errors)**2))
                print(f"  Step {step_idx + 1}: RMS error = {rms_err:.4f} m")

        # Compute final metrics
        stats = self.metrics.compute_metrics()

        result = MetricsResult(
            velocity=self.velocity,
            wind_side=self.wind_side,
            wind_longitudinal=self.wind_longitudinal,
            rms_error=stats["rms_error"],
            max_error=stats["max_error"],
            final_error=stats["final_error"],
            success=self.success,
            num_steps=len(self.metrics.position_errors),
            trajectory_duration=self._current_time,
        )

        if self.verbose:
            print("[HeadlessOrchestrator] Simulation finished")
            print(f"  Success: {self.success}")
            print(f"  RMS Error: {result.rms_error:.4f} m")
            print(f"  Max Error: {result.max_error:.4f} m")
            print(f"  Steps: {result.num_steps}")
            print(f"  Duration: {result.trajectory_duration:.2f} s")

        return result

    def get_metrics(self) -> MetricsResult:
        """Get the metrics result from the simulation."""
        stats = self.metrics.compute_metrics()

        return MetricsResult(
            velocity=self.velocity,
            wind_side=self.wind_side,
            wind_longitudinal=self.wind_longitudinal,
            rms_error=stats["rms_error"],
            max_error=stats["max_error"],
            final_error=stats["final_error"],
            success=self.success,
            num_steps=len(self.metrics.position_errors),
            trajectory_duration=self._current_time,
        )

    def get_snapshot(self) -> OrchestratorSnapshot:
        """Capture comprehensive snapshot for TUI / telemetry."""
        state = self.simulator.get_state()
        pos = state[0:3]
        vel = state[3:6]
        rates = state[6:9]
        quat = state[9:13]  # WXYZ

        # Euler angles from quaternion [w, x, y, z]
        qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
        roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)) * 180.0 / np.pi
        pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)) * 180.0 / np.pi
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)) * 180.0 / np.pi
        rpy = np.array([roll, pitch, yaw])

        ctrl_info = self.controller.get_latest_control_info()
        motors = ctrl_info.get("motors", np.zeros(4))
        total_thrust = ctrl_info.get("total_thrust", float(np.sum(motors)))
        rates_des = ctrl_info.get("rates_des", np.zeros(3))
        pitch_des = ctrl_info.get("pitch_des", 0.0) * 180.0 / np.pi

        ref_idx = min(self._ref_idx, len(self.trajectory.position) - 1)
        ref_pos = self.trajectory.position[ref_idx]
        ref_vel = self.trajectory.velocity[ref_idx]

        pos_err = float(np.linalg.norm(pos - ref_pos))
        metrics = self.metrics.compute_metrics()

        pred_traj = self.controller.get_predicted_trajectory()

        return OrchestratorSnapshot(
            step_idx=self._ref_idx,
            sim_time=self._current_time,
            current_pos=pos.copy(),
            current_vel=vel.copy(),
            current_quat=quat.copy(),
            current_rpy=rpy,
            body_rates=rates.copy(),
            motor_thrusts=motors.copy(),
            total_thrust=total_thrust,
            rates_des=rates_des.copy(),
            pitch_des=pitch_des,
            ref_pos=ref_pos.copy(),
            ref_vel=ref_vel.copy(),
            pos_error=pos_err,
            rms_error=metrics.get("rms_error", 0.0),
            max_error=metrics.get("max_error", 0.0),
            pred_trajectory=pred_traj,
            ref_trajectory=self.trajectory.position,
            is_complete=self.controller.is_trajectory_complete,
            status_ok=not bool(self.failure_reason),
        )

    def reset(self, trajectory_csv: str | None = None) -> None:
        """Reset the orchestrator to initial state."""
        self.metrics = MetricsCollector()
        self.success = False
        self.failure_reason = ""
        csv_file = trajectory_csv or str(self.trajectory.file_path if hasattr(self.trajectory, "file_path") else "")
        if not csv_file:
            # Reconstruct from initial trajectory
            csv_file = self.trajectory.filename
        self._setup_simulation(csv_file)
