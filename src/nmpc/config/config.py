import numpy as np
from enum import Enum

class ControlConfig:
    control_time = 0.02
    control_frequency = 1/control_time
    nodes = 50
    T = control_time * nodes

     

class MpcConfig:
    """
    simple mpc configuration
    """
    mpc_timeout = 3 # [s  second]
    nodes = ControlConfig.nodes
    T_horizon = ControlConfig.T

    # TODO keep sync with MpcConfig
    lb_u = np.array([0.5, 0.5, 0.5, 0.5]) * 0.0
    ub_u = np.array([10., 10., 10., 10.]) * 0.45

    qPos = np.array([1e1, 1e1, 1e1]) * 3 
    qQuat = np.array([1e2, 1e2, 1e2, 5e1]) * 1e-1 * 0.0 
    qVel = np.array([1.0,1.0, 1.0]) * 1e-1 * 1
    qAngV = np.array([2.0,2.0, 2.0]) * 1e1 * 1.0 * 0.2
    qDynamicVel = np.zeros((1,))
    rU = np.array([1e1, 1e1, 1e1, 1e1]) * 1e-3

    Metrics = False
    
    class Contrainst:
        CoordinateFlight = False


class SimpleSimConfig:
    class quad:
        mass = 2.0
        J = np.array([0.0216, 0.0216, 0.04])
        length = 0.47/2
        c_torque = 0.016
        max_thrust = 16

    class Drag:
        linear_drag = 0.3 #0.3
        quadratic_drag = 0.08

    class SimProcess:
        noisy = False
        drag = False
        motor_noise = False
        update_dt = ControlConfig.control_time



class LoopTrajConfig:
    dt = ControlConfig.control_time
    radius =5
    z = 1.0
    lin_acc = 0.1
    clockwise = True
    yawing = False
    v_max = 5

class LemniscateConfig:
    dt = ControlConfig.control_time
    radius = 20
    z = 2
    lin_acc = 0.05
    clockwise = False
    yawing = True
    v_max = 5


class PX4Interface:
    stable_odom_frq = ControlConfig.control_frequency 
    use_sim_time = True

    class thrust_curve:
        a = 11.019786910197855
        b = 5.380213089802145


def main():

    # x,y = loop_trajectory(config=LoopTrajConfig)
    pass

    # simple_simulator.drag
    # print(simple_simulator.drag)
if __name__ == "__main__":
    main()
