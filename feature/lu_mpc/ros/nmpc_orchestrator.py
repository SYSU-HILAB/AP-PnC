"""
NMPC Orchestrator for Closed-Loop Simulation

This node orchestrates the NMPC controller and CasADi simulator to run
closed-loop trajectory tracking simulations. It composes the simulation
and controller components, and delegates visualization to NmpcVisualizer.

Components:
    - GdModelSimulator: CasADi-based dynamics simulator
    - CtrlProperAccelZAngVel3D: NMPC controller for trajectory tracking
    - NmpcVisualizer: Pure visualization publisher for ROS topics

Topics Published (via NmpcVisualizer):
    - /nmpc/odometry: Odometry - Position, velocity, orientation from simulator
    - /nmpc/aero_forces: Vector3 - Aerodynamic force vector (body frame)
    - /nmpc/reference_path: Path - Reference trajectory from CSV (static)
    - /nmpc/tracking_path: Path - Actual path from simulator (rolling buffer)
    - /nmpc/motor_thrusts: Float32MultiArray - Motor thrust commands [N]

Dependencies:
    - feature.lu_mpc.controller.ctrl_proper_accel_z_ang_vel_3d: NMPC controller
    - feature.lu_mpc.model.sim.sim_gd_model: CasADi simulator
    - feature.lu_mpc.controller.trajectory_loader: CSV trajectory loading
    - feature.lu_mpc.ros.nmpc_visualizer: ROS visualization publisher

Parameters:
    - trajectory_csv: str - Path to trajectory CSV file
    - sim_dt: float - Simulation integration timestep in seconds (default: 0.02)
    - ctrl_dt: float - Controller update period in seconds (default: 0.1)
    - real_time_factor: float - Speed multiplier (1.0 = real-time, 2.0 = 2x) (default: 1.0)
    - is_hover_task: bool - If True, controller hovers at initial reference
                         if False, tracks full trajectory (default: False)

Key Functionality:
    - Runs closed-loop simulation with NMPC controller
    - Publishes odometry, motor commands, and tracking path at sim_dt
    - Delegates all ROS publishing to NmpcVisualizer
    - Differs from reference_visualizer.py: Uses controller+simulator instead of CSV replay
"""

from __future__ import annotations

from typing import Optional
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node

from ..model.sim.sim_gd_model import GdModelSimulator, GdSimConfig
from ..controller.trajectory_loader import (
    TrajectoryReference,
    load_trajectory_csv,
)
from ..controller.ctrl_proposed_nmpc import CtrlProperAccelZAngVel3D
from ..controller.config import ProposedMpcAngVelActuatedCfg
from .nmpc_visualizer import NmpcVisualizer
from ...utils.ros_viz_utils import sleep_period


class ProposedNmpcOrchestrator(Node):
    """
    ROS2 node that orchestrates NMPC simulation and delegates visualization.

    This node integrates the controller and simulator to provide closed-loop
    trajectory tracking, delegating all ROS publishing to NmpcVisualizer.
    """

    def __init__(
        self,
        trajectory_csv: str,
        sim_dt: float = 0.02,
        real_time_factor: float = 1.0,
        is_hover_task: bool = False,
    ):
        """
        Initialize the NMPC orchestrator node.

        Args:
            trajectory_csv: Path to trajectory CSV file
            sim_dt: Simulation integration timestep in seconds
            real_time_factor: Speed multiplier (1.0 = real-time, 2.0 = 2x)
            is_hover_task: If True, controller hovers at initial reference;
                           if False, tracks full trajectory (default)
        """
        super().__init__("nmpc_orchestrator")

        # Store parameters
        self.sim_dt = sim_dt
        self.real_time_factor = real_time_factor
        self.is_hover_task = is_hover_task

        # Create visualizer for ROS publishing
        self.visualizer = NmpcVisualizer(self)

        # Setup simulation and controller
        self._setup_simulation(trajectory_csv)

        # Publish static reference path
        self.visualizer.publish_reference_path(self.trajectory, self)

        self.get_logger().info("ProposedNmpcOrchestrator initialized")
        self.get_logger().info(f"  Sim dt: {sim_dt}s")
        self.get_logger().info(f"  Ctrl dt: {self.controller.ctrl_dt}s")
        self.get_logger().info(f"  Real-time factor: {real_time_factor}x")
        self.get_logger().info(f"  Mode: {'Hover' if self.is_hover_task else 'Tracking'}")

    def _setup_simulation(self, trajectory_csv: str) -> None:
        """Initialize simulator and controller from trajectory CSV."""
        trajectory_data = load_trajectory_csv(trajectory_csv)
        self.trajectory = TrajectoryReference(trajectory_data)

        # Initial state from trajectory (WXYZ format)
        p0 = trajectory_data['position'][0]
        v0 = trajectory_data['velocity'][0]
        r0 = np.array([0.0, 0.0, 0.0])
        q0 = np.array([0.707, 0.0, 0.0, 0.707])  # WXYZ format
        u0 = np.array([2.45, 2.45, 2.45, 2.45]) * 2.0
        init_state = np.concatenate([p0, v0, r0, q0, u0])

        sim_config = GdSimConfig(sim_dt=self.sim_dt)
        self.simulator = GdModelSimulator.create_gd_simulator(init_state, sim_config)

        # Create controller configuration using factory method
        config = ProposedMpcAngVelActuatedCfg.create_high_performance(
            trajectory=self.trajectory
        )
        config.is_hover_task = self.is_hover_task
        self.controller = CtrlProperAccelZAngVel3D(config)

    def step(self) -> bool:
        """
        Run one simulation step and publish results.

        Returns:
            True if simulation should continue, False if done
        """
        # Check if trajectory is complete
        if self.controller.is_trajectory_complete:
            self.get_logger().info("Trajectory complete")
            return False

        # Compute control every sim_dt, reference sampling uses ctrl_dt internally
        control = self.controller.compute_control(
            sim_state = self.simulator.get_state(),
            acc_p_meas=self.simulator.get_accp()
        )

        # Step simulation using the new stateful API
        try:
            self.simulator.simulate(control)
        except RuntimeError as e:
            self.get_logger().error(f"Simulation failed: {e}")
            return False

        # Check for NaN
        state = self.simulator.get_state()
        if np.any(np.isnan(state)):
            self.get_logger().error("NaN detected in simulation state")
            return False

        # Publish all data via visualizer
        stamp = self.get_clock().now().to_msg()
        self.visualizer.publish_odometry(state, stamp)
        self.visualizer.publish_motor_thrusts(state, stamp)
        self.visualizer.publish_tracking_path(state, stamp)

        return True

    def run(self) -> None:
        """Run the simulation loop."""
        self.get_logger().info("Starting simulation...")

        sleep_time = self.sim_dt / self.real_time_factor

        while rclpy.ok():
            continue_sim = self.step()
            if not continue_sim:
                break

            sleep_period(sleep_time)

        self.get_logger().info("Simulation finished")