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
from mpc_solver import MpcSolver

import threading
import time
import copy
import numpy as np

from utils.stat_plot import plot_density

import threading


mpc_solver = MpcSolver(cython=True)

ref_states = np.zeros((MpcConfig.nodes, 3+3+3))
cz_predict = np.zeros((MpcConfig.nodes, 1))




class MpcOptNode(Node):
    def __init__(self):
        super().__init__('MpcOptNode')

        # TODO change to TrajectoryControl
        self.create_subscription(State, "cur_state", self.state_cb, reliable_qos)
        self.controls_pub = self.create_publisher(Controls, "mpc_controls", reliable_qos)
        self.pred_traj_pub = self.create_publisher(Path, "mpc_pred_traj", keep_last_qos)
        self.controls = Controls()
        # self.metrics_pub = self.create_publisher(Metrics, "mpc_metrics", reliable_qos)

        self.ref_can_set_pub = self.create_publisher(Bool, "ref_can_set", reliable_qos)

        self.x0 = np.zeros((13,), dtype=np.float64)
        self.x0[9] = 1.0
        self.u = np.zeros((4,), dtype=np.float64)
        self.start = time.time()
        self.end = time.time()
        self.time_seq = []
        self.init = False

        self.ref_q = np.zeros((3,))
    
            
    def run_optimization(self):
        # for i in range(5):
        self.start = time.time()
        mpc_solver.solve(self.x0)

        self.u = mpc_solver.u
        self.pub_controls()
        self.end = time.time()

        self.time_seq.append(self.end-self.start)

    def pub_predict_traj(self):
        self.controls.motors_u = self.u
        # self.controls.rates_sp = mpc_solver.sim_one_step()[10:]
        self.controls.total_thrust = np.sum(self.controls.motors_u)
        self.controls_pub.publish(self.controls)
        rsts = mpc_solver.sim_traj()
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
        return mpc_solver.solve_time

    def pub_ref_can_set(self):
        msg = Bool()
        msg.data = True
        self.ref_can_set_pub.publish(msg)

    def state_cb(self, state: State):
        global ref_states
        global cond_of_set_ref
        global mpc_solver
        global cz_predict
        self.x0 = state.mpc_state
        
        # v = state.mpc_state[3:6]
        # q = state.mpc_state[9:13]
        # vb = v_dot_q_inverse(v, q)
        # # v_norm 
    
        # alpha = np.arctan2(vb[2], vb[0])


        with cond_of_set_ref:
            self.pub_ref_can_set()
            cond_of_set_ref.wait()
        # print(cz_predict.T)
        # print(ref_states)
        start = time.time()
        mpc_solver.setRefPath(ref_states, cz_predict)          
        self.run_optimization()
        end = time.time()
        print(end - start)
    def pub_controls(self):
        self.controls.motors_u = self.u
        # self.controls.rates_sp = mpc_solver.sim_one_step_rate()
        self.controls.total_thrust = np.sum(self.controls.motors_u)
        self.controls_pub.publish(self.controls)


def main():
    rclpy.init()
    mpc_node = MpcOptNode()
    ref_set_node = MpcRefNode()
    executror = MultiThreadedExecutor(num_threads=2)
    executror.add_node(mpc_node)
    executror.add_node(ref_set_node)
    executror.spin()
    

if __name__ == "__main__":
    main()