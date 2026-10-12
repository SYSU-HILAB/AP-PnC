"""Run only inside the isolated ROS2 dev container; never an aircraft domain."""

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from lidar_sim.geometry import quaternion_rotation
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "213" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("requires isolated local ROS domain 213")
    root = Path(os.environ["AP_PNC_DIR"]).resolve()
    if not args.config.is_absolute() or not args.output.is_absolute() or not args.output.is_relative_to(root / ".artifacts"):
        parser.error("absolute config and project artifact output required")
    args.output.mkdir(parents=True, exist_ok=True)
    lidar_path = args.config.with_name("lidar_sim.yaml")
    config = yaml.safe_load(args.config.read_text())["simple_sim"]
    lidar = yaml.safe_load(lidar_path.read_text())["lidar_sim"]
    assert config["aero"]["model"] == "lyu" and config.get("actuator"), "requires practical+lyu"
    log = (args.output / "launch.log").open("w")
    child = subprocess.Popen(["ros2", "launch", "bringup", "dev_sim.launch.py",
                              f"config:={args.config}", f"lidar_config:={lidar_path}", "rviz:=false"],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("dev_sim_probe")
    clouds, odometry, maps = [], {}, []
    subscriptions = [
        node.create_subscription(PointCloud2, "/lidar_sim/points", clouds.append, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/simple_sim/odom", lambda msg: odometry.update({
            msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec: msg}), 100),
        node.create_subscription(PointCloud2, "/lidar_sim/map", maps.append, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]

    def wait(predicate, seconds=20):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if child.poll() is not None:
                raise RuntimeError("dev launch exited; inspect launch.log")
        raise TimeoutError("ROS observation timed out; inspect launch.log")

    def call(service):
        client = node.create_client(Trigger, f"/simple_sim/{service}")
        assert client.wait_for_service(timeout_sec=15), service
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success, (service, future.result().message)
        node.destroy_client(client)

    try:
        wait(lambda: bool(maps))
        assert len(maps[-1].data) == maps[-1].row_step and maps[-1].header.frame_id == "world"
        # A late subscriber may miss the initial volatile scan. Single step
        # forces a new committed stamp and scan without requiring full flight.
        for _ in range(6):
            call("step")
        wait(lambda: bool(clouds) and bool(odometry))
        paused_count = len(clouds)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        assert len(clouds) == paused_count, "paused plant must not generate fresh scans"
        call("resume")
        wait(lambda: len(clouds) >= paused_count + 12, seconds=15)
        call("pause")
        wait(lambda: clouds[-1].header.stamp.sec * 1000000000 + clouds[-1].header.stamp.nanosec in odometry)
        cloud = clouds[-1]
        stamp = cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec
        odom = odometry[stamp]
        points = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
        assert cloud.header.frame_id == "lidar" and cloud.width == len(points) > 0
        assert cloud.point_step == 12 and cloud.is_dense and np.isfinite(points).all()
        assert np.max(np.linalg.norm(points, axis=1)) <= lidar["sensor"]["max_range_m"] + 1e-5
        q, p = odom.pose.pose.orientation, odom.pose.pose.position
        rotation = quaternion_rotation(q.x, q.y, q.z, q.w)
        mount = quaternion_rotation(*lidar["sensor"]["orientation_body_xyzw"])
        origin = np.array([p.x, p.y, p.z]) + rotation @ np.array(lidar["sensor"]["position_body_m"])
        world = points @ (rotation @ mount).T + origin
        scene = lidar["scene"]
        error = np.abs(world[:, 2] - scene["ground_z"])
        for pillar in scene["pillars"]:
            radial = np.hypot(world[:, 0] - pillar["x"], world[:, 1] - pillar["y"])
            vertical = (world[:, 2] >= -0.1) & (world[:, 2] <= pillar["height"] + 0.1)
            error = np.minimum(error, np.where(vertical, np.abs(radial - pillar["radius"]), np.inf))
            cap = radial <= pillar["radius"] + 0.1
            error = np.minimum(error, np.where(cap, np.abs(world[:, 2] - pillar["height"]), np.inf))
        assert np.percentile(error, 99) < 0.08, "scan does not align with physical scene"
        before = stamp
        call("reset")
        wait(lambda: clouds[-1].header.stamp.sec == 0 and clouds[-1].header.stamp.nanosec == 0)
        report = {"status": "passed", "profile": "practical", "aero": "lyu", "controller": config["controller"],
                  "map_points": maps[-1].width, "observed_scans": len(clouds), "checked_scan_points": len(points),
                  "checked_time_ns": before, "surface_error_p99_m": float(np.percentile(error, 99)),
                  "checks": ["map_latched", "committed_odometry", "scan_fields_and_frame", "pause_no_new_scan",
                             "resume_advances_scans", "first_returns_align_with_scene", "reset_to_zero"],
                  "collision_avoidance_verified": False, "hardware_connected": False}
        (args.output / "verification.json").write_text(json.dumps(report, indent=2))
        np.savez(args.output / "checked-scan.npz", sensor_points=points, world_points=world)
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=5)
        log.close()
        del subscriptions


if __name__ == "__main__":
    main()
