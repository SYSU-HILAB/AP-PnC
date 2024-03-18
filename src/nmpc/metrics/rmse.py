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
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from utils.quaternions import *
from utils.qos_setting import reliable_qos
from interface.msg import Metrics
from utils.terminal_color import *

def calculate_rmse(new_data_point, current_mean, n):
    # for my setting, mean is mse
    n += 1
    old_mean = current_mean
    current_mean += (new_data_point - current_mean) / n
    rmse = np.sqrt(current_mean)
    return current_mean, rmse, n



class MpcMetrics(Node):
    def __init__(self) -> None:
        super().__init__('model_predicitve_control_metrics')
        self.mpc_metrics_sub = self.create_subscription(Metrics, "mpc_metrics", self.mpc_metrics_cb, reliable_qos)
        self.create_timer(1.0, self.rmse_print)
        self.n = 0
        self.mse = 0
        self.rmse = -1
        self.vb_beta = 0.0
        self.v_norm = 0
        self.max_v_beta = 0.0
        self.cur_q = np.zeros((4,))
        # MAX a-ig rate max rms 
        # Create publishers
    def rmse_print(self):
        if self.rmse > 0:
          print(f"POS RMSE: {self.rmse}, Cur Vel: {self.v_norm} vz: {self.vb[2]} vx: {self.vb[0]} Vb_beta: {self.vb_beta}  MAX v_beta: {self.max_v_beta}")
          print(f"{TerminalColor.OKCYAN} {quaternion_to_euler(q=self.cur_q)} {TerminalColor.ENDC}")
    def mpc_metrics_cb(self, msg):
        metrics_msg:Metrics = msg
        pos_err = metrics_msg.ref_p - metrics_msg.cur_p
        pos_e_l2 = np.dot(pos_err, pos_err)
        quat_err = metrics_msg.ref_q - metrics_msg.cur_q
        self.cur_q = metrics_msg.cur_q
        quat_e_l2 = np.dot(quat_err, quat_err)
        mse, rmse, n = calculate_rmse(pos_e_l2, self.mse, self.n)
        self.mse = mse
        self.n = n
        self.rmse = rmse
        self.v_norm = metrics_msg.cur_v_norm
        self.vb = metrics_msg.vb
        self.vb_beta = metrics_msg.vb[1]
        if abs(self.vb_beta) > self.max_v_beta:
            self.max_v_beta = abs(self.vb_beta)




def main(args=None) -> None:

    print('RMSE')
    rclpy.init(args=args)
    # rclpy.Executor()
    mpc_metrics_node = MpcMetrics()
    executor = MultiThreadedExecutor(1)
    # rclpy.spin(fmu_in_node)
    executor.add_node(mpc_metrics_node)
    executor.spin()

    mpc_metrics_node.destroy_node()
    rclpy.shutdown()
    # sys.stdout = original_stdout # R


if __name__ == '__main__':
    main()
