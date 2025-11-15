#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from interface.msg import State, RefTrajectoryBullet, TrajectoryPoint
import time
import math

class FakeRefBulletPublisher(Node):
    def __init__(self):
        super().__init__('fake_ref_bullet_publisher')

        # QoS profile to match c_nmpc_node requirements
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribe to current state
        self.subscription = self.create_subscription(
            State,
            '/cur_state',
            self.state_callback,
            qos_profile
        )

        # Publisher for ReferenceBullet (note: c_nmpc expects /ref_trajectory_bullet)
        self.publisher_ = self.create_publisher(
            RefTrajectoryBullet,
            '/ref_trajectory_bullet',  # Fixed topic name to match c_nmpc expectations
            qos_profile
        )

        self.get_logger().info('Fake ReferenceBullet publisher started')
        self.last_state = None

    def state_callback(self, msg):
        """Called when receiving current state message"""
        self.last_state = msg
        self.publish_ref_bullet()

    def publish_ref_bullet(self):
        """Generate and publish fake reference trajectory bullet"""
        if self.last_state is None:
            return

        msg = RefTrajectoryBullet()
        msg.timestamp = self.get_clock().now().to_msg()
        # SIZE is a constant field (default=50), no need to set it
        msg.tracking_valid = True

        current_time = time.time()

        # Extract current state values
        cur_x = self.last_state.state[0]
        cur_y = self.last_state.state[1]
        cur_z = self.last_state.state[2]
        cur_vx = self.last_state.state[3]
        cur_vy = self.last_state.state[4]
        cur_vz = self.last_state.state[5]
        cur_yaw = math.atan2(2 * (self.last_state.state[9] * self.last_state.state[12]),
                            1 - 2 * (self.last_state.state[12] ** 2))

        # Generate 50 trajectory points (1 second ahead at 50Hz)
        for i in range(50):
            point = TrajectoryPoint()
            t = i * 0.02  # time step: 0.02s (50Hz)

            # Generate smooth trajectory from current position
            # Circular motion with upward spiral
            radius = 1.5
            angular_freq = 0.3

            # Position prediction
            future_x = cur_x + radius * math.cos(angular_freq * t + cur_yaw)
            future_y = cur_y + radius * math.sin(angular_freq * t + cur_yaw)
            future_z = cur_z + 0.1 * t  # Slow upward motion

            # Velocity prediction
            future_vx = -radius * angular_freq * math.sin(angular_freq * t + cur_yaw)
            future_vy = radius * angular_freq * math.cos(angular_freq * t + cur_yaw)
            future_vz = 0.1

            # Angular rates (small values)
            future_rate_x = 0.01 * math.sin(t)
            future_rate_y = 0.01 * math.cos(t)
            future_rate_z = angular_freq

            # Yaw (following the circular path)
            future_yaw = cur_yaw + angular_freq * t

            # Thrust (fake values for altitude control)
            future_thrust = 0.5 + 0.1 * math.sin(2 * t)

            # Fill the 16-dimensional point array
            # [0-2]: position, [3-5]: velocity, [6-8]: rates, [9-11]: yb, [12-15]: thrust
            point.point[0] = future_x
            point.point[1] = future_y
            point.point[2] = future_z
            point.point[3] = future_vx
            point.point[4] = future_vy
            point.point[5] = future_vz
            point.point[6] = future_rate_x
            point.point[7] = future_rate_y
            point.point[8] = future_rate_z
            # [9-11]: quaternion components (w, x, y)
            point.point[9] = math.cos(future_yaw / 2)  # quaternion w component
            point.point[10] = 0.0                     # quaternion x
            point.point[11] = math.sin(future_yaw / 2) # quaternion y component
            point.point[12] = future_thrust            # thrust component
            point.point[13] = future_thrust * 0.9      # additional thrust components
            point.point[14] = future_thrust * 1.1
            point.point[15] = future_thrust

            # Fake control zone prediction
            point.cz_prediction = 0.5 + 0.2 * math.sin(0.5 * t)

            msg.points[i] = point

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published RefTrajectoryBullet with {msg.SIZE} points')

def main(args=None):
    rclpy.init(args=args)

    fake_ref_bullet_publisher = FakeRefBulletPublisher()

    try:
        rclpy.spin(fake_ref_bullet_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_ref_bullet_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
