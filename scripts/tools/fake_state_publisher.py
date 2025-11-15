#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from interface.msg import State
import time
import math

class FakeStatePublisher(Node):
    def __init__(self):
        super().__init__('fake_state_publisher')

        # QoS profile to match c_nmpc_node requirements
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(State, '/cur_state', qos_profile)
        self.timer = self.create_timer(0.02, self.publish_state)  # 50Hz = 0.02s
        self.start_time = time.time()
        self.get_logger().info('Fake State publisher started at 50Hz')

    def publish_state(self):
        msg = State()
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Set timestamp (in nanoseconds)
        msg.timestamp = int(current_time * 1e9)

        # Generate fake trajectory data
        # Position: circular motion in x-y plane
        radius = 2.0
        angular_freq = 0.2  # rad/s

        msg.state[0] = radius * math.cos(angular_freq * elapsed_time)  # x position
        msg.state[1] = radius * math.sin(angular_freq * elapsed_time)  # y position
        msg.state[2] = 1.0 + 0.5 * math.sin(0.5 * angular_freq * elapsed_time)  # z position (varying altitude)

        # Velocity: derivative of position
        msg.state[3] = -radius * angular_freq * math.sin(angular_freq * elapsed_time)  # x velocity
        msg.state[4] = radius * angular_freq * math.cos(angular_freq * elapsed_time)   # y velocity
        msg.state[5] = 0.25 * angular_freq * math.cos(0.5 * angular_freq * elapsed_time)  # z velocity

        # Angular rates (fake small values)
        msg.state[6] = 0.01 * math.sin(elapsed_time)  # roll rate
        msg.state[7] = 0.01 * math.cos(elapsed_time)  # pitch rate
        msg.state[8] = 0.02 * math.sin(2 * elapsed_time)  # yaw rate

        # Quaternion (simplified - assume small rotation)
        yaw = angular_freq * elapsed_time
        msg.state[9] = math.cos(yaw / 2)   # w
        msg.state[10] = 0.0                # x
        msg.state[11] = 0.0                # y
        msg.state[12] = math.sin(yaw / 2)  # z

        # cur_cz (fake current control zone)
        msg.cur_cz = 0.5 + 0.3 * math.sin(0.3 * elapsed_time)

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published state at time {elapsed_time:.2f}s')

def main(args=None):
    rclpy.init(args=args)

    fake_state_publisher = FakeStatePublisher()

    try:
        rclpy.spin(fake_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
