import numpy as np
import matplotlib.pyplot as plt
from casadi import SX, vertcat, sin, cos, Function, tan
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import math
from create_ocp_or_sim import *
# Define the bicycle model
L = 2.0  # wheelbase

# States: x, y, theta
ocp = ocp_nls()
ocp: AcadosOcp = ocp_nls()
ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
    ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
)

N = 100  # number of control intervals
Nodes = ocp.dims.N
# now the model states is 
# p 3D position
# v 3D velocity
# r 3D angular velocity
# q 4D quaternion
# u 4D thrust

# control variables is du
# du 4D du
init_pos = np.array([0.0, 0.0, 0.0])
init_vel = np.array([0.0, 0.0, 0.0])
init_rate = np.array([0.0, 0.0, 0.0])
init_quat = np.array([1.0, 0.0, 0.0, 0.0])
init_u = np.array([9.8/4, 9.8/4, 9.8/4, 9.8/4]) * 2
x0 = np.concatenate([init_pos, init_vel, init_rate, init_quat])
x0_aug = np.concatenate([x0, init_u])


tgt_pos = np.array([0.0, 0.0, 1.0])
tgt_vel = np.array([0.0, 0.0, 0.0])
tgt_rate = np.array([0.0, 0.0, 0.0])
# yaw 90 degree
tgt_yb = np.array([-1.0, 0.0, 0.0])
mass = 1.152
tgt_du = np.array([0.0, 0.0, 0.0, 0.0])

tgt_x = np.concatenate([tgt_pos, tgt_vel, tgt_rate, tgt_yb])
tgt_y = np.concatenate([tgt_pos, tgt_vel, tgt_rate, tgt_yb, tgt_du])

simX = np.zeros((N+1, ocp.model.x.size()[0]))
simU = np.zeros((N, ocp.model.u.size()[0]))

ocp_solver.set(0, "lbx", x0_aug)
ocp_solver.set(0, "ubx", x0_aug)
# Solve OCP
for i in range(N):
    for j in range(Nodes):
      ocp_solver.set(j, "yref", tgt_y)
    ocp_solver.set(Nodes, "yref", tgt_x)
    status = ocp_solver.solve()
    if status not in [0,2]:
        raise Exception(f'acados returned status {status}.')

    # Get the solution
    simX[i, :] = ocp_solver.get(0, "x")
    simU[i, :] = ocp_solver.get(0, "u")
    x0 = ocp_solver.get(1, "x")

    # Set initial state for next step
    ocp_solver.set(0, "lbx", x0)
    ocp_solver.set(0, "ubx", x0)
    
    
# Get final state
simX[N, :] = ocp_solver.get(1, "x")

# Plot the results
time = np.linspace(0, N*0.04, N+1)

plt.figure(figsize=(10, 6))

# Plot x and y positions
plt.subplot(3, 1, 1)
plt.plot(time, simX[:, 2], marker='.')
plt.xlabel('x position')
plt.ylabel('y position')
plt.title('Trajectory')

# Plot states over time
plt.subplot(3, 1, 2)
plt.plot(time, simX[:, 9], label='q1')
plt.plot(time, simX[:, 10], label='q2')
plt.plot(time, simX[:, 11], label='q3')
plt.plot(time, simX[:, 12], label='q4')
plt.xlabel('Time [s]')
plt.ylabel('States')
plt.legend()
plt.title('States over Time')

# Plot control inputs over time
plt.subplot(3, 1, 3)
plt.plot(time[:-1], simX[:N, 13], label='u1', marker='.')
plt.plot(time[:-1], simX[:N, 14], label='u2', marker='.')
plt.plot(time[:-1], simX[:N, 15], label='u3', marker='.')
plt.plot(time[:-1], simX[:N, 16], label='u4', marker='.')
plt.xlabel('Time [s]')
plt.ylabel('Control inputs')
plt.legend()
plt.title('Control Inputs over Time')

plt.tight_layout()
plt.show()