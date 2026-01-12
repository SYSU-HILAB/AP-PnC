"""
Thin helpers to launch ROS-based visualization flows from feature modules.

Keeping the launching logic here avoids re-implementing rclpy bootstrapping
inside every experimental node.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor


def run_state_visualizer(
    trajectory_csv: str,
    sim_time: float = 10.0,
    sim_dt: float = 0.01,
    control_dt: float = 0.1,
    real_time_factor: float = 1.0,
    collect_aero: bool = True,
    executor: Optional[SingleThreadedExecutor] = None,
) -> None:
    """
    Launch the lu_mpc StatePublisherNode inside a ROS2 executor.

    Args mirror feature.lu_mpc.ros.viz_node.StatePublisherNode so this helper
    can be reused by CLI tools or notebooks without duplicating ROS boilerplate.
    """
    from feature.lu_mpc.ros.viz_node import StatePublisherNode

    rclpy.init(args=None)
    node = StatePublisherNode(
        trajectory_csv=trajectory_csv,
        sim_time=sim_time,
        sim_dt=sim_dt,
        control_dt=control_dt,
        real_time_factor=real_time_factor,
        collect_aero=collect_aero,
    )
    executor = executor or SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
