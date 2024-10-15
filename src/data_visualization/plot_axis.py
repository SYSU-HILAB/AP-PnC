import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Create the figure and axes object
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Define the 3D axis lines (as before)
x_axis = np.array([[0, 0], [0, 0], [0, 1]]) * 2
y_axis = np.array([[0, 0], [0, -1], [0, 0]]) * 2
z_axis = np.array([[0, 1], [0, 0], [0, 0]]) * 2

scale = 4
horizontal = z_axis * scale
vertical = x_axis

# Define rotation angles (in degrees)
yaw = 0  # Yaw angle
pitch = 70 # Pitch angle
roll = 0 # Roll angle


# Create rotation matrix using scipy's Rotation library
rotation = R.from_euler('xyz', [yaw, pitch, roll], degrees=True) #Order matters!

# Apply rotation to the axis lines
rotated_x = rotation.apply(x_axis.T).T
rotated_y = rotation.apply(y_axis.T).T
rotated_z = rotation.apply(z_axis.T).T

vv = horizontal + rotated_y * .5

vx = rotated_x * np.sin(pitch*np.pi/180) * scale
vz = rotated_z * np.cos(pitch*np.pi/180) * scale

# Plot the rotated axes
ax.plot(rotated_x[0, :], rotated_x[1, :], rotated_x[2, :], 'r', linewidth=2, label='X-axis')
ax.plot(rotated_y[0, :], rotated_y[1, :], rotated_y[2, :], 'g', linewidth=2, label='Y-axis')
ax.plot(rotated_z[0, :], rotated_z[1, :], rotated_z[2, :], 'b', linewidth=2, label='Z-axis')
ax.plot(horizontal[0, :], horizontal[1, :], horizontal[2, :], 'c:', linewidth=.5, label='horizontal')
ax.plot(vertical[0, :], vertical[1, :], vertical[2, :], 'c:', linewidth=.5, label='vertical')
ax.plot(vv[0, :], vv[1, :], vv[2, :], 'y', linewidth=2, label='vv')
ax.plot(vx[0, :], vx[1, :], vx[2, :], 'y:', linewidth=.5, label='vx')
ax.plot(vz[0, :], vz[1, :], vz[2, :], 'y:', linewidth=.5, label='vz')
ax.grid(False)
ax.set_axis_off() #This turns off the entire axis frame
# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Axes with Roll, Pitch, and Yaw')

# ax.set_xticks([])
# ax.set_yticks([])
# ax.set_zticks([])
# Set limits for better visualization (optional)
ax.set_xlim([-1.5, 8.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 2.5])
ax.view_init(azim=-129, elev=24) #azim is rotation around z-axis, elev is elevation
plt.show()