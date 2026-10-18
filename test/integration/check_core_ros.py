"""Isolated ROS adapter smoke test with real solver and synthetic sensor input.

Run only in a dedicated ROS_DOMAIN_ID. This is not a flight/plant acceptance test.
"""

import argparse
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from interface.msg import Control, ReferenceHorizon, ReferencePoint, State, TrackingInfo
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from tooling.env import artifact_path, get_git_root


def run(install: Path):
    if os.environ.get("ROS_DOMAIN_ID") != "213":
        raise RuntimeError("This smoke test requires isolated ROS_DOMAIN_ID=213")
    config = yaml.safe_load((get_git_root() / "core/bringup/config/nmpc.yaml").read_text())["nmpc"]
    count = round(config["horizon_s"] / config["traj_res_s"]) + 1
    output = artifact_path(".artifacts/tests/core-adapters/ros")
    output.mkdir(parents=True, exist_ok=True)
    processes = []
    logs = []
    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"])
    node = rclpy.create_node("core_adapter_probe")
    try:

        def start(package):
            candidates = (
                install / package / "lib" / package / f"{package}_node",
                install / "lib" / package / f"{package}_node",
            )
            binary = next((path for path in candidates if path.is_file()), None)
            if binary is None:
                raise FileNotFoundError(f"Missing {package}_node under {install}")
            log = (output / f"{package}.log").open("w")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    [str(binary), "--ros-args", "--disable-external-lib-logs"],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            )

        start("nmpc")
        controls, tracking, references = [], [], []
        node.create_subscription(Control, "/nmpc/control", controls.append, 1)
        node.create_subscription(TrackingInfo, "/nmpc/tracking_info", tracking.append, 1)
        node.create_subscription(ReferenceHorizon, "/nmpc/reference", references.append, 1)
        state_pub = node.create_publisher(State, "/px4ctrl/state", 1)
        imu_pub = node.create_publisher(Imu, "/mavros/imu/data_raw", qos_profile_sensor_data)
        ref_pub = node.create_publisher(ReferenceHorizon, "/nmpc/reference", 1)
        mode_pub = node.create_publisher(Bool, "/planner/mode", 1)
        state = State()
        state.state[9] = 1.0
        imu = Imu()
        imu.linear_acceleration.x = 0.02
        horizon = ReferenceHorizon()
        horizon.traj_resolution = config["traj_res_s"]
        horizon.tracking_valid = True
        for _ in range(count):
            point = ReferencePoint()
            point.yb[1] = 1.0
            horizon.points.append(point)

        def spin_until(condition, timeout, publish_reference=False):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                for process in processes:
                    if process.poll() is not None:
                        raise RuntimeError("ROS adapter exited; inspect saved logs")
                state_pub.publish(state)
                imu_pub.publish(imu)
                if publish_reference:
                    ref_pub.publish(horizon)
                rclpy.spin_once(node, timeout_sec=0.02)
                if condition():
                    return
            raise TimeoutError("ROS smoke test condition not met; inspect saved logs")

        spin_until(
            lambda: state_pub.get_subscription_count() > 0
            and imu_pub.get_subscription_count() > 0
            and ref_pub.get_subscription_count() > 0,
            10.0,
        )
        # Allow both sensor streams to arrive before asking the real solver to run.
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            state_pub.publish(state)
            imu_pub.publish(imu)
            rclpy.spin_once(node, timeout_sec=0.02)
        spin_until(lambda: bool(tracking) and bool(controls), 10.0, publish_reference=True)
        info = tracking[-1]
        expected_cx = imu.linear_acceleration.x / 0.1  # zero speed, regularized denominator
        if not math.isclose(info.cx, expected_cx, rel_tol=1e-6, abs_tol=1e-6):
            raise AssertionError(f"body-X coefficient mismatch: {info.cx} != {expected_cx}")
        if set(State.get_fields_and_field_types()) != {"timestamp", "state"}:
            raise AssertionError("State schema does not match the quadrotor state contract")
        if set(TrackingInfo.get_fields_and_field_types()) != {
            "timestamp",
            "ref_position",
            "ref_velocity",
            "actual_position",
            "actual_velocity",
            "ref_vxy",
            "actual_vxy",
            "ref_yb",
            "actual_yb",
            "alpha",
            "beta",
            "cx",
        }:
            raise AssertionError("TrackingInfo schema does not match the FLU feedback contract")
        command = controls[-1]
        if not all(
            math.isfinite(value) for value in (*command.rates_sp, command.specific_force_sp)
        ):
            raise AssertionError("non-finite real-solver command")
        bounds = config["command_bounds"]
        if not bounds["lower"][0] <= command.specific_force_sp <= bounds["upper"][0]:
            raise AssertionError("specific thrust outside configured bounds")
        print("PASS real NMPC control delivery and correctly labeled body-X feedback")

        # No planner runs yet: a wrong-grid horizon must not emit a control.
        deadline = time.monotonic() + 0.2
        spin_until(lambda: time.monotonic() >= deadline, 1.0)
        controls.clear()
        horizon.traj_resolution += 0.01
        deadline = time.monotonic() + 0.3
        spin_until(lambda: time.monotonic() >= deadline, 1.0, publish_reference=True)
        if controls:
            raise AssertionError("invalid reference grid emitted a command")
        horizon.traj_resolution = config["traj_res_s"]
        imu.linear_acceleration.x = 0.08
        deadline = time.monotonic() + 0.2
        spin_until(lambda: time.monotonic() >= deadline, 1.0)
        tracking.clear()
        spin_until(lambda: bool(tracking) and bool(controls), 10.0, publish_reference=True)
        if not math.isclose(tracking[-1].cx, 0.4, rel_tol=1e-6, abs_tol=1e-6):
            raise AssertionError("coefficient feedback saturation/recovery failed")
        print("PASS wrong-grid rejection, recovery and saturated body-X feedback")

        # Nonidentity q_WB and nonzero velocity expose axis swaps, reversed
        # quaternion direction and a flipped sideslip sign.
        yaw = 0.7
        velocity = (0.3, 0.2, 0.1)
        state.state[3:6] = velocity
        state.state[9:13] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        body = (
            math.cos(yaw) * velocity[0] + math.sin(yaw) * velocity[1],
            -math.sin(yaw) * velocity[0] + math.cos(yaw) * velocity[1],
            velocity[2],
        )
        tracking.clear()
        spin_until(
            lambda: bool(tracking)
            and all(
                math.isclose(a, b, abs_tol=1e-6)
                for a, b in zip(tracking[-1].actual_velocity, velocity, strict=True)
            ),
            10.0,
            publish_reference=True,
        )
        info = tracking[-1]
        if not math.isclose(info.alpha, math.atan2(body[0], body[2]), abs_tol=1e-6):
            raise AssertionError("FLU alpha/quaternion direction mismatch")
        if not math.isclose(
            info.beta, math.atan2(body[1], math.hypot(body[0], body[2])), abs_tol=1e-6
        ):
            raise AssertionError("FLU sideslip sign mismatch")
        if not all(
            math.isclose(a, b, abs_tol=1e-6)
            for a, b in zip(info.actual_yb, (-math.sin(yaw), math.cos(yaw), 0.0), strict=True)
        ):
            raise AssertionError("body-Y direction is not expressed in world ENU")
        expected_cx = imu.linear_acceleration.x / (sum(v * v for v in velocity) + 0.1)
        if not math.isclose(info.cx, expected_cx, abs_tol=1e-6):
            raise AssertionError("moving-state body-X coefficient mismatch")
        print("PASS nonidentity q_WB, FLU angles and world-ENU body-Y direction")
        state.state[3:6] = (0.0, 0.0, 0.0)
        state.state[9:13] = (1.0, 0.0, 0.0, 0.0)
        deadline = time.monotonic() + 0.2
        spin_until(lambda: time.monotonic() >= deadline, 1.0)

        # Stop synthetic reference publication. The real planner must supply the horizon.
        start("planner")
        references.clear()
        spin_until(
            lambda: any(not message.tracking_valid for message in references)
            and mode_pub.get_subscription_count() > 0,
            15.0,
        )
        if any(len(message.points) != count for message in references):
            raise AssertionError("planner horizon size mismatch")
        references.clear()
        controls.clear()
        tracking.clear()
        mode = Bool()
        mode.data = True
        mode_pub.publish(mode)
        spin_until(lambda: any(message.tracking_valid for message in references), 10.0)
        spin_until(lambda: bool(controls) and bool(tracking), 10.0)
        print("PASS planner startup, horizon delivery, mode switch and real NMPC consumption")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        for log in logs:
            log.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-install", type=Path, required=True)
    args = parser.parse_args()
    if not args.ros_install.is_absolute():
        parser.error("--ros-install must be absolute")
    run(args.ros_install)
