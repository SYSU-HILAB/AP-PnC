#
# Created on Fri Nov 17 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
# this file is used for ouput stable odometry for mpc optimization

# frq: controlFrequency

# in : PX4-odometry 
# out : stable control Frequency

import time
from model import *
from utils import *
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleCommand
from px4_msgs.msg import VehicleOdometry, VehicleRatesSetpoint
from px4_msgs.msg import VehicleThrustAccSetpoint
from interface.msg import Controls, State
from nav_msgs.msg import Odometry

from config.config import PX4Interface

from acados_template import AcadosSim, AcadosSimSolver

from utils.qos_setting import keep_last_qos, reliable_qos
from utils.px4_conversions import *
from utils.quaternions import *

from config.config import MpcConfig

class PX4InterfaceNode(Node):
    """PX4 interface to ROS2."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', keep_last_qos)
        self.vehicle_thrust_acc_pub = self.create_publisher(
            VehicleThrustAccSetpoint, '/fmu/in/vehicle_thrust_acc_setpoint', keep_last_qos
        )
        # Create subscribers
        self.mpc_ctrls_sub = self.create_subscription(
            Controls, '/mpc_controls', self.convert_to_px4_controls, keep_last_qos
        )
        self.last_run_mpc = int(0);


    def convert_to_px4_controls(self, control_msg):            
        mpc_controls:Controls = control_msg
        px4_msg = VehicleThrustAccSetpoint()
        px4_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_msg.thrust_acc_sp = float(mpc_controls.total_thrust)
        
        px4_msg.rates_sp = FLUtoFRD(mpc_controls.rates_sp).astype(np.float32)
        self.vehicle_thrust_acc_pub.publish(px4_msg)
        self.last_run_mpc = px4_msg.timestamp



    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if (self.last_run_mpc and timestamp - self.last_run_mpc > MpcConfig.mpc_timeout * 1e5):
            return
            
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)



        self.offboard_control_mode_publisher.publish(msg)




    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.arm()
        self.engage_offboard_mode()
        self.publish_offboard_control_heartbeat_signal()
        # self.nm_pub()
        #TODO status check
        self.heartbeat_connect_on = True

class StableOdometryNode(Node):
    def __init__(self):
        super().__init__('stable_odometry_node')
        self.px4_odometry_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.update_state, keep_last_qos)
        

        #p,v,r,xb,yb
        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        self.r = np.zeros((3,))
        self.xb = np.zeros((3,))
        self.yb = np.zeros((3,))

        # publisher
        self.state_pub = self.create_publisher(State, "cur_state", keep_last_qos)
        self.cur_odom_pub = self.create_publisher(Odometry, "cur_state_odom", reliable_qos)
        
    def stably_pub_state(self):
        msg = State()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.state = self.state
        msg.mpc_state = self.mpc_state
        msg.p = self.p 
        msg.v = self.v 
        msg.r = self.r
        msg.xb = self.xb
        msg.yb = self.yb
        self.state_pub.publish(msg)

        odom = Odometry()
        odom.header.frame_id = "world"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.state[2]
        odom.pose.pose.orientation.w = self.state[3]
        odom.pose.pose.orientation.x = self.state[4]
        odom.pose.pose.orientation.y = self.state[5]
        odom.pose.pose.orientation.z = self.state[6]
        self.cur_odom_pub.publish(odom)

    def update_state(self, msg):
        vehicle_odom:VehicleOdometry = msg
        cur_stamp = int(self.get_clock().now().nanoseconds / 1000)
        mpc_state = self.mpc_state
        self.mpc_state[:3] = self.state[:3]
        self.mpc_state[3:6] = self.state[7:10]
        self.mpc_state[6:9] = self.state[10:13]
        self.mpc_state[9:9+4] = self.state[3:7]



def main(args=None) -> None:

    print('Starting PX4 interface')
    rclpy.init(args=args)
    # rclpy.Executor()
    executor = MultiThreadedExecutor(1)
    fmu_in_node = PX4InterfaceNode()
    # rclpy.spin(fmu_in_node)
    executor.add_node(fmu_in_node)
    executor.spin()
    fmu_in_node.destroy_node()
    rclpy.shutdown()
    # sys.stdout = original_stdout # R


if __name__ == '__main__':
    main()
