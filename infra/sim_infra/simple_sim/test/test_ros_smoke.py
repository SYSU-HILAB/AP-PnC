"""Isolated ROS integration smoke test; never connects to PX4 or flight topics."""

import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


def main() -> None:
    executable = Path(sys.argv[1]).resolve()
    root = Path(sys.argv[2]).resolve()
    # Keep the test away from the application's normal domain; no custom RMW/QoS profiles.
    os.environ["ROS_DOMAIN_ID"] = str(215 + os.getpid() % 10)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = rclpy.create_node("simple_sim_smoke_observer")
    clocks: list[int] = []
    subscription = node.create_subscription(
        Clock,
        "/clock",
        lambda msg: clocks.append(msg.clock.sec * 10**9 + msg.clock.nanosec),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    clients = {
        name: node.create_client(Trigger, f"/simple_sim/{name}")
        for name in ("pause", "resume", "step", "reset")
    }

    def spin_until(predicate, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError("ROS observation/service timed out")
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(name: str) -> None:
        spin_until(lambda: clients[name].service_is_ready())
        future = clients[name].call_async(Trigger.Request())
        spin_until(future.done)
        result = future.result()
        assert result is not None and result.success, f"{name}: {result}"

    with (executable.parent / "ros-smoke.log").open("w") as log:
        process = subprocess.Popen(
            [str(executable), "--ros-args", "-p", f"controller:={sys.argv[3]}"],
            env={**os.environ, "AP_PNC_DIR": str(root)},
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            spin_until(lambda: clients["resume"].service_is_ready())
            call("pause")
            call("step")
            spin_until(lambda: bool(clocks) and clocks[-1] > 0)
            before = clocks[-1]
            # A paused simulator must not advance while wall time passes.
            deadline = time.monotonic() + 0.15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert clocks[-1] == before, "pause advanced simulated time"
            call("step")
            spin_until(lambda: clocks[-1] > before)
            dt = clocks[-1] - before
            assert dt > 0, "single step did not advance a complete control period"
            before = clocks[-1]
            call("resume")
            spin_until(lambda: clocks[-1] >= before + 3 * dt)
            call("pause")
            old_count = len(clocks)
            call("reset")
            spin_until(lambda: len(clocks) > old_count and clocks[-1] == 0)
            call("step")
            spin_until(lambda: clocks[-1] == dt)
            assert process.poll() is None, "simulator exited unexpectedly"
            print("PASS ROS /clock, pause, resume, single-step and full reset")
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
