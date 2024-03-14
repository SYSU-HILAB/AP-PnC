from config.config import MpcConfig, ControlConfig
from utils.trajectories import get_reference_chunk

from model import tailsitter_flu

# msgs setting 
from interface.msg import RefTrajectoryBullet
from interface.msg import Controls, State, Metrics
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from utils.qos_setting import *
from utils.visualizations import *
from config.config import Simulator
from mpc_solver import MpcSolver

import threading
import time
import copy
import numpy as np

from utils.stat_plot import plot_density
from utils.simple_simulator import SimNode
traj_bullet = RefTrajectoryBullet()




class MpcNode(Node):
    def __init__(self):
        super().__init__('mpc_solver')

        self.mpc_solver = MpcSolver(cython=True)
        # self.mpc_solver.setRefTrajectory()
        # TODO change to TrajectoryControl
        self.create_subscription(RefTrajectoryBullet, "ref_trajecotry_bullet", self.traj_bullet_cb, keep_last_qos)
        self.create_subscription(State, "cur_state", self.state_cb, keep_last_qos)
        self.controls_pub = self.create_publisher(Controls, "mpc_controls", reliable_qos)
        self.pred_traj_pub = self.create_publisher(Path, "mpc_pred_traj", keep_last_qos)
        self.controls = Controls()
        
        # self.metrics_pub = self.create_publisher(Metrics, "mpc_metrics", reliable_qos)

        self.x0 = np.zeros((13,), dtype=np.float64)
        self.u = np.zeros((4,), dtype=np.float64)
        self.start = time.time()
        self.end = time.time()
        self.time_seq = []
        self.init = False

        self.ref_q = np.zeros((3,))
        self.ref_states = np.zeros((MpcConfig.nodes, 3+3+3))
    
#   this function reads the reference trajectory from the message and sets it to the mpc solver
    def traj_bullet_cb(self, msg):
        traj_bullet:RefTrajectoryBullet = msg
        traj_bullet.timestamp = self.get_clock().now().to_msg()
        i = 0
        for point in traj_bullet.points:
            self.ref_states[i] = point.point
            i = i + 1
        assert i == MpcConfig.nodes

        self.mpc_solver.setRefPath(self.ref_states)

    def run_optimization(self):
        # for i in range(5):
        self.start = time.time()
        self.mpc_solver.solve(self.x0)

        self.u = self.mpc_solver.u
        self.pub_controls()
        self.end = time.time()

        self.time_seq.append(self.end-self.start)

    def pub_predict_traj(self):
        self.controls.motors_u = self.u
        # self.controls.rates_sp = self.mpc_solver.sim_one_step()[10:]
        self.controls.total_thrust = np.sum(self.controls.motors_u)
        self.controls_pub.publish(self.controls)
        rsts = self.mpc_solver.sim_traj()
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(rsts.shape[0]):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = rsts[i, 0]
            pose.pose.position.y = rsts[i, 1]
            pose.pose.position.z = rsts[i, 2]
            pose.pose.orientation.w = rsts[i, 3]
            pose.pose.orientation.x = rsts[i, 4]
            pose.pose.orientation.y = rsts[i, 5]
            pose.pose.orientation.z = rsts[i, 6]
            msg.poses.append(pose)
        self.pred_traj_pub.publish(msg)
        
    @property
    def opt_time(self):
        return self.mpc_solver.solve_time

    def state_cb(self, state:State):
        self.x0 = state.mpc_state
        # if MpcConfig.Metrics == True:
        #     metrics_msg = Metrics()
        #     metrics_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        #     metrics_msg.cur_p = self.x0[:3]
        #     metrics_msg.cur_q = self.x0[3:7]
        #     metrics_msg.ref_p = self.ref_p
        #     metrics_msg.ref_v_norm = self.ref_v_norm
        #     metrics_msg.cur_v_norm = np.linalg.norm(self.x0[7:10], 2)
        #     self.metrics_pub.publish(metrics_msg)
        # self.opt_thread.start()
        self.run_optimization()
        print(self.x0[9:])
        # self.pub_predict_traj()
        

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
    
    # time = np.array(mpc_node.time_seq)
    
    # print("***********")
    # print(f"mean time: {np.mean(time)}, max time: {np.max(time)}")
    # plot_density(time)

    # try:
    #     while True:
    #     # 你的代码逻辑放在这里
    #         pass
    # except KeyboardInterrupt:
        
    #     time = np.array(mpc_node.time_seq)
    #     print("***********")
    #     print(f"mean time: {np.mean(time)}, max time: {np.max(time)}")
    #     plot_density(time)






if __name__ == "__main__":
    main()