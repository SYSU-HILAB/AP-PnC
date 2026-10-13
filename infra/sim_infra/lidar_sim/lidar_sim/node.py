"""ROS2 map and snapshot LiDAR scans driven only by committed simulator odometry."""

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from lidar_sim.geometry import beam_directions, load_scene, quaternion_rotation, scan


def point_cloud(points, stamp, frame):
    points = np.ascontiguousarray(points, dtype="<f4").reshape(-1, 3)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=axis, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, axis in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class LidarSim(Node):
    def __init__(self):
        super().__init__("lidar_sim")
        config = Path(self.declare_parameter("config", "").value)
        if not config.is_absolute():
            raise ValueError("LiDAR config must be absolute")
        data = yaml.safe_load(config.read_text())["lidar_sim"]
        self.scene = load_scene(data["scene"])
        self.settings = data["sensor"]
        bounds = [float(self.settings[key]) for key in ("min_range_m", "max_range_m", "noise_std_m")]
        if not np.isfinite(bounds).all() or not 0 < bounds[0] < bounds[1] or bounds[2] < 0:
            raise ValueError("invalid LiDAR range/noise configuration")
        rate = float(self.settings["rate_hz"])
        if not np.isfinite(rate) or not 0 < rate <= 100:
            raise ValueError("invalid scan rate")
        self.period_ns = round(1e9 / rate)
        self.directions = beam_directions(int(self.settings["channels"]), int(self.settings["columns"]),
                                          float(self.settings["vertical_fov_deg"]))
        self.offset = np.array(self.settings.get("position_body_m", [0, 0, 0]), dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("invalid LiDAR mount position")
        mount_q = self.settings.get("orientation_body_xyzw", [0, 0, 0, 1])
        self.mount_rotation = quaternion_rotation(*mount_q)
        self.frame = data.get("sensor_frame", "lidar")
        self.world = data.get("world_frame", "world")
        self.base = data.get("body_frame", "base_link")
        self.last_ns = None
        self.scan_count = 0
        self.pub = self.create_publisher(PointCloud2, "~/points", qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, "~/map", map_qos)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = self.base, self.frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.offset.tolist()
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = mount_q
        self.static_tf.sendTransform(transform)
        self.map_pub.publish(point_cloud(self.scene.point_cloud(), transform.header.stamp, self.world))
        self.sub = self.create_subscription(Odometry, data.get("odom_topic", "/simple_sim/odom"),
                                            self.on_odom, QoSProfile(depth=1))
        self.get_logger().info(f"Ready: {len(self.directions)} beams, {rate:g}Hz, /lidar_sim/points; paused odometry creates no new scans")

    def on_odom(self, msg):
        stamp = msg.header.stamp
        ns = int(stamp.sec) * 1000000000 + stamp.nanosec
        if msg.header.frame_id != self.world or msg.child_frame_id != self.base:
            self.get_logger().error("Unexpected odometry frames; no scan published")
            return
        if self.last_ns is not None and ns >= self.last_ns and ns - self.last_ns < self.period_ns:
            return
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        try:
            rotation = quaternion_rotation(q.x, q.y, q.z, q.w)
            position = np.array([p.x, p.y, p.z], dtype=float)
            sensor_rotation = rotation @ self.mount_rotation
            origin = position + rotation @ self.offset
            # Seed by committed time, not callback count or wall time.
            seed = [int(self.settings.get("seed", 0)), ns & 0xffffffff, ns >> 32]
            points = scan(self.scene, origin, sensor_rotation, self.directions,
                          float(self.settings["min_range_m"]), float(self.settings["max_range_m"]),
                          float(self.settings.get("noise_std_m", 0)), seed)
        except ValueError:
            self.get_logger().error("Invalid sensor state/config; no scan published")
            return
        transform = TransformStamped()
        transform.header, transform.child_frame_id = msg.header, self.base
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.tf.sendTransform(transform)
        self.pub.publish(point_cloud(points, stamp, self.frame))
        self.last_ns = ns
        self.scan_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarSim()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
