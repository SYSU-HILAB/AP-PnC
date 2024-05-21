import numpy as np
from enum import Enum


class ControlConfig:
    control_time = 0.04
    control_frequency = 1 / control_time
    nodes = 20
    T = control_time * nodes


class MpcConfig:
    """
    simple mpc configuration
    """
    mpc_timeout = 3  # [s  second]
    nodes = ControlConfig.nodes
    T_horizon = ControlConfig.T

    cz = 0.0 * np.zeros((1,))
    # TODO keep sync with MpcConfig
    lb_u = np.array([0.5, 0.5, 0.5, 0.5]) * 1
    ub_u = np.array([10., 10., 10., 10.]) * 0.35
    # 1e1 15 15 20 OK
    qPos = np.array([ 12,  12,  20]) * 1 * 1.0 

    qYB = np.array([1e2, 1e2, 1e2]) * 1.0 * 0.1 
    # qYB = np.array([1e2, 1e2, 1e2]) * 1.0 * 0.0
    qvelh = 1.0 # 5.0
    qvelv = 1.0

    qVel = np.array([qvelh, qvelh, qvelv]) * 1 * 0.9
    wx=0.4
    wy=0.6
    wz=0.5
    qAngV = np.array([wx, wy, wz]) * 1 
    qDynamicVel = np.zeros((1,))
    rU = np.array([1e1, 1e1, 1e1, 1e1]) * 1e-3 * 5

    Metrics = False




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
