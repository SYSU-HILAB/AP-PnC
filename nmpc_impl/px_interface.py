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

        self.vehicle_thrust_acc_pub = self.create_publisher(
            VehicleThrustAccSetpoint, '/fmu/in/vehicle_thrust_acc_setpoint', keep_last_qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)


    def directly_pub_px4control_test(self):            
        px4_msg = VehicleThrustAccSetpoint()
        px4_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_msg.thrust_acc_sp = float(20.0)
        #print
        print("px4_msg.thrust_acc_sp: ", px4_msg.thrust_acc_sp)
        self.vehicle_thrust_acc_pub.publish(px4_msg)


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.directly_pub_px4control_test()




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
