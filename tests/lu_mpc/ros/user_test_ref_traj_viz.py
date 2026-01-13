# -*- coding: future_fstrings -*-
"""
User-facing launcher that publishes the reference trajectory and opens RViz2.
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
from pathlib import Path

import rclpy

from feature.lu_mpc.controller.trajectory_loader import get_trajectory_path
from feature.lu_mpc.ros.keyboard_listener import SpacebarListener
from feature.lu_mpc.ros.ref_path_node import ReferencePathNode


def _resolve_rviz_config(user_path: str | None) -> Path:
    if user_path:
        return Path(user_path).expanduser().resolve()
    repo_root = Path(__file__).parents[3]
    return (repo_root / "feature" / "lu_mpc" / "ros" / "config" / "nmpc_viz.rviz").resolve()


def _launch_rviz(config_path: Path) -> subprocess.Popen[bytes]:
    print(f"[viz] Launching RViz2 with config: {config_path}")
    return subprocess.Popen(["rviz2", "-d", str(config_path)])


def _shutdown_process(proc: subprocess.Popen[bytes] | None) -> None:
    if not proc or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def _resolve_trajectory_arg(user_path: Path | None) -> Path:
    """Resolve --trajectory to an absolute CSV path, defaulting to planner outputs."""
    if user_path:
        trajectory = user_path.expanduser().resolve()
    else:
        trajectory = Path(get_trajectory_path()).resolve()
    print(f"[viz] Visualizing trajectory CSV: {trajectory}")
    return trajectory


def _run_reference_publisher(args: argparse.Namespace) -> None:
    rclpy.init(args=None)
    node = ReferencePathNode(
        trajectory_csv=str(args.trajectory),
        playback_dt=args.playback_dt,
        real_time_factor=args.real_time_factor,
        loop_playback=not args.no_loop,
    )

    listener = SpacebarListener(node.toggle_pause)
    listener_started = listener.start()

    try:
        rclpy.spin(node)
    finally:
        if listener_started:
            listener.stop()
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize reference trajectory in RViz2.")
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=None,
        help="Path to trajectory CSV (defaults to artifacts/simulations/temp_data planning output).",
    )
    parser.add_argument("--playback-dt", type=float, default=0.05, help="Playback timestep in seconds.")
    parser.add_argument(
        "--real-time-factor",
        type=float,
        default=1.0,
        help="Speed multiplier for the simulation loop.",
    )
    parser.add_argument("--no-loop", action="store_true", help="Play the reference once instead of looping.")
    parser.add_argument(
        "--rviz-config",
        type=str,
        default=None,
        help="Optional RViz2 config path. Defaults to feature/lu_mpc/ros/config/nmpc_viz.rviz.",
    )
    parser.add_argument("--no-rviz", action="store_true", help="Do not launch RViz2 alongside the node.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.trajectory = _resolve_trajectory_arg(args.trajectory)

    rviz_proc: subprocess.Popen[bytes] | None = None
    config_path = _resolve_rviz_config(args.rviz_config)

    if not args.no_rviz:
        rviz_proc = _launch_rviz(config_path)

    try:
        _run_reference_publisher(args)
    except KeyboardInterrupt:
        print("[viz] Caught Ctrl-C, shutting down...")
    finally:
        _shutdown_process(rviz_proc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
