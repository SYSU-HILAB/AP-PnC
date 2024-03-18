import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from utils.qos_setting import *
from interface.msg import Controls

import numpy as np





class MpcNode(Node):
    def __init__(self):
        super().__init__('mpc_solver')
        self.controls_pub = self.create_publisher(Controls, "mpc_controls", 10)
        self.controls = Controls()

        self.timer = self.create_timer(0.02, self.pub_controls)
        self.u = 9.8 * 1.5 * np.ones((4,)) / 4

    def pub_controls(self):
        self.controls.motors_u = self.u
        # self.controls.rates_sp = self.mpc_solver.sim_one_step_rate()
        self.controls.total_thrust = np.sum(self.controls.motors_u)
        self.controls_pub.publish(self.controls)


def main():
    rclpy.init()
    mpc_node = MpcNode()
    mpc_executor = SingleThreadedExecutor()
    mpc_executor.add_node(mpc_node)
    mpc_executor.spin()



if __name__ == "__main__":
    main()