import numpy as np
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from utils.quaternions import q_dot_q, quaternion_inverse
# from nav_msgs.msg import 
# z substitute x 
quat_pitch_90 = np.array([0.707106781186548, 0 , 0.707106781186548, 0])
quat_pitch_m90 = quaternion_inverse(quat_pitch_90)

from utils.qos_setting import reliable_qos, keep_last_qos
class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.start_pub = self.create_publisher(Marker, "start_point", reliable_qos)
        self.end_pub = self.create_publisher(Marker, "end_point", reliable_qos)
    def pub_path(self, pos, quat, pub):
        '''
        pos: Nx3  (x,y,z)
        quat: Nx4 (w,x,y,z)
        pub: pub
        '''
        assert pos.shape[0] == quat.shape[0]
        len = pos.shape[0]
        msg = PoseArray()
        msg.header.frame_id = "/world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len):
            pose = Pose()
            pose.position.x = pos[i,0] 
            pose.position.y = pos[i,1] 
            pose.position.z = pos[i,2]
            quat_correct = q_dot_q(quat[i], quat_pitch_m90)
            pose.orientation.w = quat_correct[0] 
            pose.orientation.x = quat_correct[1] 
            pose.orientation.y = quat_correct[2] 
            pose.orientation.z = quat_correct[3]             
            msg.poses.append(pose)

        # msg.header.stamp = self.get_clock().now().to_msg()
        # pub.publish(msg)
        self.ref_path_pub.publish(msg)

    def pub_start_point(self, pos, quat):
        marker = Marker()
        marker.header.frame_id="world"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = Pose()
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]

        # to visualization only z axis
        quat = q_dot_q(quat, quat_pitch_m90)
        
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        marker.scale.x = 5.0
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.g = 1.0
        marker.color.a = 1.0
        self.start_pub.publish(marker)

    def pub_end_point(self, pos, quat):
        marker = Marker()
        marker.header.frame_id="world"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = Pose()
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]

        # to visualization only z axis
        quat = q_dot_q(quat, quat_pitch_m90)


        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        marker.scale.x = 5.0
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.end_pub.publish(marker)

    def publish_odometry(self, pos, quat, pub, info="publishing odometry"):
        '''
            pos : 1x3
            quat : 1x4
        '''

        odometry_msg = Odometry()
        odometry_msg.header.frame_id = 'world'
        odometry_msg.child_frame_id = 'base_link'

        odometry_msg.pose.pose.position.x = pos[0]
        odometry_msg.pose.pose.position.y = pos[1]
        odometry_msg.pose.pose.position.z = pos[2]
        odometry_msg.pose.pose.orientation.w = quat[0]
        odometry_msg.pose.pose.orientation.x = quat[1]
        odometry_msg.pose.pose.orientation.y = quat[2]
        odometry_msg.pose.pose.orientation.z = quat[3]
        # odometry_msg.pose.
        # odometry_msg.twist.twist = Twist()
        # odometry_msg.twist.twist.linear = Point(0.1, 0.0, 0.0)
        # odometry_msg.twist.twist.angular = Point(0.0, 0.0, 0.0)

        pub.publish(odometry_msg)
        if info is not None:
            self.get_logger().info(info)    