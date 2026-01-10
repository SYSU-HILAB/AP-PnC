# -*- coding: future_fstrings -*-
#
# ROS2 Visualization Node for nMPC Controller
# Created on 2025-01-10
#
# This node publishes odometry and aerodynamic data from the
# nMPC controller + CasADi simulator for visualization in RViz2.
#

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from pathlib import Path
from typing import Optional

# ROS2 messages
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Twist
from std_msgs.msg import Float32MultiArray, Header

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sim.sim_gd_model import create_gd_simulator, GdSimConfig
from ctrl.trajectory_loader import load_trajectory_csv, TrajectoryReference
from ctrl.proposed_nmpc_ctrl import ProposedNmpcController


class StatePublisherNode(Node):
    """
    ROS2 node that publishes simulation state for visualization.

    Topics:
        /odometry: Odometry message with position, velocity, orientation
        /aero_data: Custom aerodynamic data message
        /reference_trajectory: Reference path from CSV
        /tracking_trajectory: Actual path from simulator
        /motor_thrusts: Motor thrust commands
    """

    def __init__(
        self,
        trajectory_csv: str,
        sim_time: float = 10.0,
        sim_dt: float = 0.01,
        control_dt: float = 0.1,
        real_time_factor: float = 1.0,
        collect_aero: bool = True,
    ):
        """
        Initialize the state publisher node.

        Args:
            trajectory_csv: Path to trajectory CSV file
            sim_time: Total simulation time
            sim_dt: Simulation time step
            control_dt: Controller time step
            real_time_factor: Speed factor (1.0 = real time, 2.0 = 2x speed)
            collect_aero: Whether to collect and publish aero data
        """
        super().__init__("state_publisher_node")

        # Parameters
        self.sim_time = sim_time
        self.sim_dt = sim_dt
        self.control_dt = control_dt
        self.real_time_factor = real_time_factor
        self.collect_aero = collect_aero

        # Create publishers
        # Use reliable QoS for important messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_pub = self.create_publisher(Odometry, "/odometry", qos_profile)
        self.aero_pub = self.create_publisher(Vector3, "/aero_data", 10)  # Simplified for now
        self.ref_path_pub = self.create_publisher(Path, "/reference_trajectory", qos_profile)
        self.track_path_pub = self.create_publisher(Path, "/tracking_trajectory", qos_profile)
        self.motor_pub = self.create_publisher(Float32MultiArray, "/motor_thrusts", 10)

        # Setup simulation and controller
        self._setup_simulation(trajectory_csv)

        # Reference trajectory for path publishing
        self._publish_reference_path()

        # Tracking path
        self.tracking_path = Path()
        self.tracking_path.header.frame_id = "world"

        self.get_logger().info("StatePublisherNode initialized")
        self.get_logger().info(f"  Sim time: {sim_time}s")
        self.get_logger().info(f"  Real-time factor: {real_time_factor}x")

    def _setup_simulation(self, trajectory_csv: str):
        """Setup simulator and controller."""
        # Load trajectory
        trajectory_data = load_trajectory_csv(trajectory_csv)
        self.trajectory = TrajectoryReference(trajectory_data)

        # Create simulator
        sim_config = GdSimConfig(dt=self.sim_dt, sim_time=self.sim_time)
        self.sim = create_gd_simulator(sim_config)

        # Create controller
        self.controller = ProposedNmpcController(self.trajectory, control_dt=self.control_dt)

        # Initial state
        p0 = trajectory_data['position'][0]
        v0 = trajectory_data['velocity'][0]
        r0 = np.array([0.0, 0.0, 0.0])
        q0 = np.array([1.0, 0.0, 0.0, 0.0])
        u0 = np.array([2.45, 2.45, 2.45, 2.45])
        self.current_state = np.concatenate([p0, v0, r0, q0, u0])

        self.current_time = 0.0
        self.step_count = 0
        self.N_sim = int(self.sim_time / self.sim_dt)

    def _publish_reference_path(self):
        """Publish the reference trajectory as a Path message."""
        ref_path = Path()
        ref_path.header.frame_id = "world"
        ref_path.header.stamp = self.get_clock().now().to_msg()

        # Sample reference trajectory
        sample_times = np.linspace(0, self.sim_time, 200)
        for t in sample_times:
            ref = self.trajectory.get_reference_at_time(t)
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = ref['position'][0]
            pose.pose.position.y = ref['position'][1]
            pose.pose.position.z = ref['position'][2]
            ref_path.poses.append(pose)

        self.ref_path_pub.publish(ref_path)
        self.get_logger().info("Reference trajectory published")

    def _numpy_to_quaternion(self, q_np: np.ndarray) -> Quaternion:
        """Convert numpy quaternion [qx, qy, qz, qw] to ROS Quaternion."""
        q_ros = Quaternion()
        q_ros.x = float(q_np[1])
        q_ros.y = float(q_np[2])
        q_ros.z = float(q_np[3])
        q_ros.w = float(q_np[0])
        return q_ros

    def _numpy_to_vector3(self, v_np: np.ndarray) -> Vector3:
        """Convert numpy array to ROS Vector3."""
        v_ros = Vector3()
        v_ros.x = float(v_np[0])
        v_ros.y = float(v_np[1])
        v_ros.z = float(v_np[2])
        return v_ros

    def publish_odometry(self, state: np.ndarray):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        # Position (NED frame)
        p = state[0:3]
        odom.pose.pose.position = self._numpy_to_vector3(p)

        # Orientation
        q = state[9:13]
        odom.pose.pose.orientation = self._numpy_to_quaternion(q)

        # Velocity
        v = state[3:6]
        odom.twist.twist.linear = self._numpy_to_vector3(v)

        # Angular rates
        r = state[6:9]
        odom.twist.twist.angular = self._numpy_to_vector3(r)

        self.odom_pub.publish(odom)

    def publish_aero_data(self, aero_data: dict):
        """Publish aerodynamic data (simplified as Vector3 for now)."""
        # For simplicity, publish forces as Vector3
        forces = self._numpy_to_vector3(aero_data['forces_body'])
        self.aero_pub.publish(forces)

    def publish_motor_thrusts(self, state: np.ndarray):
        """Publish motor thrusts."""
        motor_msg = Float32MultiArray()
        motor_msg.data = state[13:17].tolist()
        self.motor_pub.publish(motor_msg)

    def publish_tracking_path(self, state: np.ndarray):
        """Publish tracking path."""
        pose = PoseStamped()
        pose.header = self.tracking_path.header
        pose.pose.position = self._numpy_to_vector3(state[0:3])
        q = state[9:13]
        pose.pose.orientation = self._numpy_to_quaternion(q)
        self.tracking_path.poses.append(pose)

        # Keep path size manageable
        if len(self.tracking_path.poses) > 500:
            self.tracking_path.poses.pop(0)

        self.track_path_pub.publish(self.tracking_path)

    def step(self):
        """
        Run one simulation step and publish results.

        Returns:
            True if simulation should continue, False if done
        """
        if self.step_count >= self.N_sim:
            self.get_logger().info("Simulation completed")
            return False

        # Compute control at control frequency
        if self.step_count % int(self.control_dt / self.sim_dt) == 0:
            control = self.controller.compute_control(self.current_state, self.current_time)
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

        # Publish odometry
        self.publish_odometry(self.current_state)

        # Collect and publish aero data
        if self.collect_aero and self.step_count % 10 == 0:
            aero_data = self.sim.get_aerodynamic_forces(self.current_state)
            self.publish_aero_data(aero_data)

        # Publish motor thrusts
        self.publish_motor_thrusts(self.current_state)

        # Publish tracking path (every 10 steps)
        if self.step_count % 10 == 0:
            self.publish_tracking_path(self.current_state)

        return True

    def run(self):
        """Run the simulation loop."""
        self.get_logger().info("Starting simulation...")

        # Calculate sleep time for real-time execution
        sleep_time = self.sim_dt / self.real_time_factor

        while rclpy.ok():
            # Step simulation
            continue_sim = self.step()
            if not continue_sim:
                break

            # Sleep to maintain real-time rate
            self.sleep_period(sleep_time)

        self.get_logger().info("Simulation finished")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Get trajectory path
    ros_dir = Path(__file__).parent
    traj_csv = ros_dir.parent / 'model' / 'refs' / 'v14-alpha.csv'

    if not traj_csv.exists():
        print(f"Error: Trajectory file not found: {traj_csv}")
        return 1

    # Create node
    node = StatePublisherNode(
        trajectory_csv=str(traj_csv),
        sim_time=10.0,      # 10 seconds simulation
        sim_dt=0.01,        # 100 Hz
        control_dt=0.1,     # 10 Hz control
        real_time_factor=1.0,  # Real time
        collect_aero=True,
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
