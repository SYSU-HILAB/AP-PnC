"""
Simulation Visualization Node for nMPC Controller

This node publishes state data from the nMPC controller and CasADi simulator
for real-time visualization in RViz2.

Data Sources:
    - nMPC Controller: Computes control inputs at ctrl_dt frequency
    - CasADi Simulator: Integrates vehicle dynamics at sim_dt frequency
    - Trajectory CSV: Provides reference trajectory for comparison

Topics Published:
    - /nmpc/odometry: Odometry - Position, velocity, orientation from simulator
    - /nmpc/aero_forces: Vector3 - Aerodynamic force vector (body frame)
    - /nmpc/reference_path: Path - Reference trajectory from CSV (static)
    - /nmpc/tracking_path: Path - Actual path from simulator (rolling buffer)
    - /nmpc/motor_thrusts: Float32MultiArray - Motor thrust commands [N]

Dependencies:
    - feature.lu_mpc.controller.ctrl_proposed_nmpc: nMPC controller
    - feature.lu_mpc.model.sim.sim_gd_model: CasADi simulator
    - feature.lu_mpc.controller.trajectory_loader: CSV trajectory loading
    - feature.utils.ros_visualizer: ROS message conversion utilities

Parameters:
    - trajectory_csv: str - Path to trajectory CSV file
    - sim_tot_dur: float | None - Total simulation time in seconds (None = run forever)
    - sim_dt: float - Simulation integration timestep in seconds (default: 0.01)
    - ctrl_dt: float - Controller update period in seconds (default: 0.1)
    - real_time_factor: float - Speed multiplier (1.0 = real-time, 2.0 = 2x) (default: 1.0)

Key Functionality:
    - Runs closed-loop simulation with nMPC controller
    - Publishes odometry, motor commands, and tracking path at sim_dt
    - Publishes aerodynamic forces at 10x sim_dt (to reduce overhead)
    - Differs from reference_visualizer.py: Uses controller+simulator instead of CSV replay
"""

from __future__ import annotations

from typing import Optional
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

from feature.lu_mpc.model.sim.sim_gd_model import GdModelSimulator, GdSimConfig
from feature.lu_mpc.controller.trajectory_loader import (
    TrajectoryReference,
    load_trajectory_csv,
)
from feature.lu_mpc.controller.ctrl_proposed_nmpc import ProposedNmpcController
from feature.utils.ros_visualizer import (
    make_topic,
    compute_reference_sample_times,
    numpy_to_point,
    numpy_to_vector3,
    numpy_to_quaternion_wxyz,
    create_path_message,
    add_pose_to_path,
    truncate_path,
    create_reliable_qos,
    create_transient_local_qos,
    sleep_period,
)


class SimulationVisualizer(Node):
    """
    ROS2 node that runs nMPC simulation and publishes state for visualization.

    This node integrates the controller and simulator to provide closed-loop
    trajectory tracking visualization in RViz2.
    """

    def __init__(
        self,
        trajectory_csv: str,
        sim_tot_dur: float | None = None,
        sim_dt: float = 0.02,
        ctrl_dt: float = 0.1,
        real_time_factor: float = 1.0,
    ):
        """
        Initialize the simulation visualizer node.

        Args:
            trajectory_csv: Path to trajectory CSV file
            sim_tot_dur: Total simulation time in seconds (None = run forever)
            sim_dt: Simulation integration timestep in seconds
            ctrl_dt: Controller update period in seconds
            real_time_factor: Speed multiplier (1.0 = real-time, 2.0 = 2x)
        """
        super().__init__("simulation_visualizer")

        # Store parameters
        self.sim_tot_dur = sim_tot_dur
        self.sim_dt = sim_dt
        self.ctrl_dt = ctrl_dt
        self.real_time_factor = real_time_factor

        # Create publishers with shared QoS
        qos_profile = create_reliable_qos(depth=10)
        qos_transient_local = create_transient_local_qos(depth=10)

        self.odom_pub = self.create_publisher(
            Odometry, make_topic("odometry"), qos_profile
        )
        self.aero_pub = self.create_publisher(
            Vector3, make_topic("aero_forces"), 10
        )
        self.ref_path_pub = self.create_publisher(
            Path, make_topic("reference_path"), qos_transient_local
        )
        self.track_path_pub = self.create_publisher(
            Path, make_topic("tracking_path"), qos_profile
        )
        self.motor_pub = self.create_publisher(
            Float32MultiArray, make_topic("motor_thrusts"), 10
        )

        # Setup simulation and controller
        self._setup_simulation(trajectory_csv)

        # Publish static reference path
        self._publish_reference_path()

        # Initialize tracking path
        self.tracking_path = create_path_message(frame_id="world")

        self.get_logger().info("SimulationVisualizer initialized")
        sim_dur_msg = "infinite" if sim_tot_dur is None else f"{sim_tot_dur}s"
        self.get_logger().info(f"  Sim duration: {sim_dur_msg}, dt: {sim_dt}s")
        self.get_logger().info(f"  Control dt: {ctrl_dt}s")
        self.get_logger().info(f"  Real-time factor: {real_time_factor}x")

    def _setup_simulation(self, trajectory_csv: str) -> None:
        """Initialize simulator and controller from trajectory CSV."""
        trajectory_data = load_trajectory_csv(trajectory_csv)
        self.trajectory = TrajectoryReference(trajectory_data)

        sim_tot_dur = self.sim_tot_dur if self.sim_tot_dur is not None else self.sim_dt
        sim_config = GdSimConfig(sim_dt=self.sim_dt, sim_tot_dur=sim_tot_dur)
        self.sim = GdModelSimulator.create_gd_simulator(sim_config)

        self.controller = ProposedNmpcController(
            self.trajectory, ctrl_dt=self.ctrl_dt
        )

        # Initial state from trajectory (WXYZ format)
        p0 = trajectory_data['position'][0]
        v0 = trajectory_data['velocity'][0]
        r0 = np.array([0.0, 0.0, 0.0])
        q0 = np.array([0.707, 0.0, 0.0, 0.707])  # WXYZ format
        u0 = np.array([2.45, 2.45, 2.45, 2.45]) * 2.0
        self.current_state = np.concatenate([p0, v0, r0, q0, u0])

        self.current_time = 0.0
        self.step_count = 0
        self.N_sim = None if self.sim_tot_dur is None else int(self.sim_tot_dur / self.sim_dt)
        self.prev_control = np.zeros(4)

    def _publish_reference_path(self) -> None:
        """Publish the static reference trajectory as a Path message."""
        ref_path = create_path_message(frame_id="world")
        ref_path.header.stamp = self.get_clock().now().to_msg()

        sample_times = compute_reference_sample_times(
            self.trajectory, sample_count=400
        )

        for t in sample_times:
            ref = self.trajectory.get_reference_at_time(t)
            add_pose_to_path(
                ref_path,
                ref['position'],
                ref.get('quaternion'),
                frame_id="world",
            )

        self.ref_path_pub.publish(ref_path)
        self.get_logger().info("Reference trajectory published")

    def publish_odometry(self, state: np.ndarray) -> None:
        """Publish odometry message from simulation state."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position = numpy_to_point(state[0:3])

        # Orientation (WXYZ format from state[9:13])
        odom.pose.pose.orientation = numpy_to_quaternion_wxyz(state[9:13])

        # Velocity
        odom.twist.twist.linear = numpy_to_vector3(state[3:6])

        # Angular rates
        odom.twist.twist.angular = numpy_to_vector3(state[6:9])

        self.odom_pub.publish(odom)

    def publish_aero_data(self, aero_data: dict) -> None:
        """Publish aerodynamic forces as Vector3."""
        forces = numpy_to_vector3(aero_data['forces_body'])
        self.aero_pub.publish(forces)

    def publish_motor_thrusts(self, state: np.ndarray) -> None:
        """Publish motor thrust commands."""
        motor_msg = Float32MultiArray()
        motor_msg.data = state[13:17].tolist()
        self.motor_pub.publish(motor_msg)

    def publish_tracking_path(self, state: np.ndarray) -> None:
        """Publish tracking path with rolling buffer."""
        add_pose_to_path(
            self.tracking_path,
            state[0:3],
            state[9:13],
            frame_id="world",
            stamp=self.get_clock().now().to_msg(),
        )

        # Keep path size manageable
        truncate_path(self.tracking_path, max_poses=500)

        self.track_path_pub.publish(self.tracking_path)

    def step(self) -> bool:
        """
        Run one simulation step and publish results.

        Returns:
            True if simulation should continue, False if done
        """
        if self.N_sim is not None and self.step_count >= self.N_sim:
            self.get_logger().info("Simulation completed")
            return False

        # Compute control at control frequency
        if self.step_count % int(self.ctrl_dt / self.sim_dt) == 0:
            control = self.controller.compute_control(self.current_state)
        else:
            control = self.prev_control if self.step_count > 0 else np.zeros(4)

        self.prev_control = control

        # Step simulation
        self.sim.acados_solver.set("x", self.current_state)
        self.sim.acados_solver.set("u", control)

        if self.sim.config.integrator_type == "IRK":
            xdot_guess = np.zeros(self.current_state.shape[0])
            self.sim.acados_solver.set("xdot", xdot_guess)

        status = self.sim.acados_solver.solve()

        if status != 0:
            self.get_logger().error(f"Simulation failed at step {self.step_count}")
            return False

        self.current_state = self.sim.acados_solver.get("x")
        self.current_time += self.sim_dt
        self.step_count += 1

        # Check for NaN
        if np.any(np.isnan(self.current_state)):
            self.get_logger().error(f"NaN detected at step {self.step_count}")
            return False

        # Publish all data
        self.publish_odometry(self.current_state)
        self.publish_motor_thrusts(self.current_state)

        if self.step_count % 10 == 0:
            self.publish_tracking_path(self.current_state)

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


def main(args=None):
    """Main entry point for simulation visualizer."""
    rclpy.init(args=args)

    ros_dir = Path(__file__).parent
    traj_csv = ros_dir.parent / 'controller' / 'refs' / 'v14-alpha.csv'

    if not traj_csv.exists():
        print(f"Error: Trajectory file not found: {traj_csv}")
        return 1

    node = SimulationVisualizer(
        trajectory_csv=str(traj_csv),
        sim_tot_dur=10.0,
        sim_dt=0.02,
        ctrl_dt=0.1,
        real_time_factor=1.0,
    )

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
