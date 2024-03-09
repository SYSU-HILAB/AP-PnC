import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

class MeshPublisher(Node):

    def __init__(self):
        super().__init__('mesh_publisher')
        self.publisher_ = self.create_publisher(Marker, '/visualizer/uav_traj', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.pose.position.x = 20.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.707
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.707
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # Replace this with the path to your mesh resource
        # marker.mesh_resource = "package://gcopter/meshes/x500/meshes/NXP-HGD-CF.dae"
        marker.mesh_resource = "package://gcopter/meshes/tailsitter/meshes/tailsitter_fuselage.dae"
        print("pub mesh ...")
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    mesh_publisher = MeshPublisher()

    rclpy.spin(mesh_publisher)
    mesh_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
