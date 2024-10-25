import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the environment variable and the CSV file
csv_file_path = os.path.join(os.getenv('TAILSITTER_PLANNING_DIR'), 'planning.csv')

df = pd.read_csv(csv_file_path, on_bad_lines='skip')

# Alternatively, for newer versions of pandas (1.3.0+), you can use:
# df = pd.read_csv(csv_file_path, on_bad_lines='skip')
columns_to_convert = ['time', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'thrust', 'alpha', 'aero_force_z']

# Apply pd.to_numeric to each column in the list
for column in columns_to_convert:
    df[column] = pd.to_numeric(df[column], errors='coerce')
# Convert alpha from radians to degrees
if 'alpha_deg' not in df.columns:
    df['alpha_deg'] = np.degrees(df['alpha'])

# Calculate the norm of the velocity
if 'velocity_norm' not in df.columns:
    df['velocity_norm'] = np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2)

print(df['velocity_norm'])
time = df['time'].values
velocity_norm = df['velocity_norm'].values

# First plot with two subplots: 3D trajectory and velocity norm vs time
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# 3D trajectory plot (pos_x, pos_y, pos_z)
ax_3d = fig.add_subplot(121, projection='3d')
ax_3d.plot(df['pos_x'].values, df['pos_y'].values, df['pos_z'].values)
ax_3d.set_xlabel('Position X')
ax_3d.set_ylabel('Position Y')
ax_3d.set_zlabel('Position Z')
ax_3d.set_title('3D Trajectory')

# Velocity norm vs time
axs[1].plot(time, velocity_norm, label='Velocity Norm')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Velocity Norm')
axs[1].set_title('Velocity Norm vs Time')
axs[1].grid(True)

plt.tight_layout()
plt.show()

# Second plot with two subplots: velocity norm and alpha vs aero_force_z with dual y-axis
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

# First subplot: velocity norm vs time
ax1.plot(time, velocity_norm, label='Velocity Norm')
ax1.set_xlabel('Time')
ax1.set_ylabel('Velocity Norm')
ax1.set_title('Velocity Norm vs Time')
ax1.grid(True)

# Second subplot: alpha (degrees) vs aero_force_z with dual y-axis
alpha_deg = df['alpha_deg'].values
aero_force_z = df['aero_force_z'].values

ax2.plot(time, alpha_deg, label='Alpha (degrees)', color='b')
ax2.set_xlabel('Time')
ax2.set_ylabel('Alpha (degrees)', color='b')
ax2.tick_params(axis='y', labelcolor='b')

# Create a second y-axis for aero_force_z
ax3 = ax2.twinx()
ax3.plot(time, aero_force_z, label='Aero Force Z', color='r')
ax3.set_ylabel('Aero Force Z', color='r')
ax3.tick_params(axis='y', labelcolor='r')

ax2.set_title('Alpha vs Aero Force Z with Dual Y-axis')

plt.tight_layout()
plt.show()