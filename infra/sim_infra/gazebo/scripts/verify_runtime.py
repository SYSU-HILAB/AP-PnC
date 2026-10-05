"""Read-only SITL smoke test. No arming, mode changes or setpoint publication."""

import argparse
import json
import math
import time

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("docker_runtime_smoke")
    result = {
        "connected": False,
        "armed": None,
        "imu": 0,
        "odom": 0,
        "state_odom": 0,
        "thrust_scaling": None,
    }
    parameters = node.create_client(GetParameters, "/mavros/setpoint_raw/get_parameters")
    request = GetParameters.Request()
    request.names = ["thrust_scaling"]
    future = None

    def state(message):
        result["connected"] = message.connected
        result["armed"] = message.armed

    def count(key):
        def received(_message):
            result[key] += 1

        return received

    subscriptions = [
        node.create_subscription(State, "/mavros/state", state, 10),
        node.create_subscription(Imu, "/mavros/imu/data", count("imu"), qos_profile_sensor_data),
        node.create_subscription(
            Odometry, "/mavros/local_position/odom", count("odom"), qos_profile_sensor_data
        ),
        node.create_subscription(
            Odometry, "/px4ctrl/state_odom", count("state_odom"), qos_profile_sensor_data
        ),
    ]
    passed = False
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            result["nodes"] = sorted(set(node.get_node_names()))
            if future is None and parameters.service_is_ready():
                future = parameters.call_async(request)
            if future is not None and future.done():
                response = future.result()
                if response and response.values:
                    value = response.values[0].double_value
                    result["thrust_scaling"] = value if math.isfinite(value) else None
                future = None
            passed = (
                result["connected"]
                and result["armed"] is False
                and all(result[key] >= 10 for key in ("imu", "odom", "state_odom"))
                and {"planner_node", "nmpc_node"}.issubset(result["nodes"])
                and result["thrust_scaling"] == 1.0
            )
            if passed:
                break
        print(json.dumps({"passed": passed, **result}, indent=2))
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit("SITL smoke failed: inspect connection, data streams and core nodes")


if __name__ == "__main__":
    main()
