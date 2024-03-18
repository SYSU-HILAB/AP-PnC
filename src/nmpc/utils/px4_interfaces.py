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

# INDI estimation setting 
# requires 
# vehicle angular veloctiy
# VehicleAcceleration.msg
from px4_msgs.msg import VehicleAcceleration
# first must do thrust calibrate， INDI？

def getNormalizedThrust(thrust: float):
    if (thrust > 16.):
        print("WARN:MAX_THRUST")
        return 1.
    if (thrust < 0.):
        print("WARN:MIN_THRUST")
        return 0.
    a = PX4Interface.thrust_curve.a
    b = PX4Interface.thrust_curve.b
    return ((b*b + 4 * a * thrust) ** (1/2) - b)/(2*a)


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
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', keep_last_qos
        )
        # Create subscribers
        self.mpc_ctrls_sub = self.create_subscription(
            Controls, '/mpc_controls', self.convert_to_px4_controls, keep_last_qos
        )
        self.last_run_mpc = int(0);

        self.offboard = True

        # link connect
        self.heartbeat_connect_on = False
        # init yref

        param = rclpy.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, PX4Interface.use_sim_time
        )
        self.set_parameters([param])
        self.timer = self.create_timer(1.0, self.timer_callback)


    def convert_to_px4_controls(self, control_msg):            
        mpc_controls:Controls = control_msg
        px4_msg = VehicleThrustAccSetpoint()
        px4_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_msg.thrust_acc_sp = float(mpc_controls.total_thrust)
        
        px4_msg.rates_sp = FLUtoFRD(mpc_controls.rates_sp).astype(np.float32)
        self.vehicle_thrust_acc_pub.publish(px4_msg)
        self.last_run_mpc = px4_msg.timestamp


    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        # in px4 commander:
        # 			} else if (!strcmp(argv[1], "offboard")) {
		# 		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        if self.offboard:
            self.get_logger().warn("ENGAGING OFFCTL MODE")
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        # self.get_logger().info("Switching to offboard mode")

    def engage_position_mode(self):
        """Switch to POSCTL mode."""
        # in px4 commander :
        # 			} else if (!strcmp(argv[1], "auto:loiter")) {
		# 		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
		# 				     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
        self.offboard = False
        self.get_logger().warn("ENGAGING POSCTL MODE")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0, param3=3.0)


    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if (self.last_run_mpc and timestamp - self.last_run_mpc > MpcConfig.mpc_timeout * 1e5):
            # DON'T PUBLISH OFFBOARD MODE
            self.engage_position_mode()
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





def odometry_sim():

  sim = AcadosSim()
  sim.solver_options.num_stages = 4
  sim.solver_options.num_steps = 1
  sim.solver_options.T = 1
  sim.solver_options.integrator_type = 'ERK'
  sim.solver_options.sens_forw = False
  # sim.solver_options.
  sim.model = odometry_model()
  # sim.dims.nx = sim.model.
  AcadosSimSolver.generate(
      sim, json_file="acados_sim_" + sim.model.name + ".json"
  )
  AcadosSimSolver.build(sim.code_export_directory, with_cython=True)
  sim_solver = AcadosSimSolver.create_cython_solver('acados_sim_' + sim.model.name + '.json')
  return sim_solver


class StableOdometryNode(Node):
    def __init__(self):
        super().__init__('stable_odometry_node')
        self.px4_odometry_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.update_state, keep_last_qos)
        
        param = rclpy.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, PX4Interface.use_sim_time
        )
        self.set_parameters([param])

        self.last_stamp = None;
        self.incremental_time_s = 1 / PX4Interface.stable_odom_frq
        self.incremental_time_us = int( 1 / PX4Interface.stable_odom_frq * 1e6)
        self.frq = PX4Interface.stable_odom_frq
        self.state = np.ndarray((13,))
        self.mpc_state = np.ndarray((13,))
        self.sim_solver = odometry_sim()

        #p,v,r,xb,yb
        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        self.r = np.zeros((3,))
        self.xb = np.zeros((3,))
        self.yb = np.zeros((3,))

        # This sim time callback
        self.tim_bas = self.create_timer(self.incremental_time_s, self.stably_pub_state)

        # publisher
        self.state_pub = self.create_publisher(State, "cur_state", keep_last_qos)
        self.cur_odom_pub = self.create_publisher(Odometry, "cur_state_odom", reliable_qos)

    def set_next_step_state(self, pos, quat, vel, rate, time):
        """
        IN
          pos: 1x3, quat: 1x4, as state
          vel: 1x3, rate: 1x3, as control
          time: time difference to next update
          WARN rate is rad/s
        OUT
        """
        odom_sim:AcadosSimSolver = self.sim_solver
        odom_sim.set('T', time)
        # x is list object 
        x = odom_sim.simulate(x=np.concatenate((pos,quat)), u= np.concatenate((vel, rate)))
        self.state = np.concatenate((np.array(x), vel, rate))
        
    def stably_pub_state(self):
        self.last_stamp = int(self.get_clock().now().nanoseconds / 1000)
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
        # TODO set low pass filter to filtering msg
        # PROBELM vo.stamp is wrong!, may caused by PX4-gazebo, since px4 boot time
        vo:VehicleOdometry = msg
        # TODO check valid odometry, all must have
        cur_stamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.last_stamp is None:
            return
        td = self.last_stamp + self.incremental_time_us - cur_stamp
        self.set_next_step_state(NEDtoENU(vo.position), canonical_quaternion(NEDtoENU(vo.q)), NEDtoENU(vo.velocity), FRDtoFLU(vo.angular_velocity), td * 1e-6)
        ## self.state is local FLU world, body is FLU.
        ## we only translate it to mpc solver, visualize things keep not changed.
        # as we see, odometry outputs [p,q,v,r] 
        # for mpc use, we need to reorganize [p,v,r,q]
        mpc_state = self.mpc_state

        if MpcConfig.airframe == MpcConfig.Airframe.Quadrotor:
            self.mpc_state[:3] = self.state[:3]
            self.mpc_state[3:6] = self.state[7:10]
            self.mpc_state[6:9] = self.state[10:13]
            self.mpc_state[9:9+4] = self.state[3:7]
        if MpcConfig.airframe == MpcConfig.Airframe.Tailsitter:
            # pos 
            mpc_state[0:3] = self.state[0:3]
            # vel
            mpc_state[3:6] = self.state[7:10] #
            # rate
            mpc_state[6:9] = tailsitter_rate(self.state[10:13])
            mpc_state[9:13] = tailsiiter_quat(self.state[3:7])

        self.p = self.mpc_state[:3]
        self.v = self.mpc_state[3:6]
        self.r = self.mpc_state[6:9]
        q = self.mpc_state[9:]
        _, self.xb, self.yb, _ = quaternion_to_rotation_matrix(q)


def main(args=None) -> None:

    print('Starting PX4 interface')
    rclpy.init(args=args)
    # rclpy.Executor()
    executor = MultiThreadedExecutor(2)
    fmu_out_node = StableOdometryNode()
    fmu_in_node = PX4InterfaceNode()
    # rclpy.spin(fmu_in_node)
    executor.add_node(fmu_in_node)
    executor.add_node(fmu_out_node)
    executor.spin()
    fmu_in_node.destroy_node()
    fmu_out_node.destroy_node()
    rclpy.shutdown()
    # sys.stdout = original_stdout # R


if __name__ == '__main__':
    main()
