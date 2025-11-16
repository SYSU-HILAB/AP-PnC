#!/usr/bin/env python3
import os
from cProfile import label
from turtle import color

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("GTK3Agg")  # 或者 'QtAgg', 'GTK3Agg' 等

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the environment variable and the CSV file
csv_file_path = os.path.join(os.getenv("AP_PNC_DIR"), "temp_data", "planning.csv")

df = pd.read_csv(csv_file_path, on_bad_lines="skip")
print(df.head())
# Print column names to debug
print("Available columns:", df.columns.tolist())
# Alternatively, for newer versions of pandas (1.3.0+), you can use:
# df = pd.read_csv(csv_file_path, on_bad_lines='skip')
columns_to_convert = [
    "time",
    "pos_x",
    "pos_y",
    "pos_z",
    "vel_x",
    "vel_y",
    "vel_z",
    "thrust",
    "alpha",
    "aero_force_z",
    "ap_x",
    "ap_y",
    "ap_z",
    "S_ap_x",
    "S_ap_z",
    "B_Omega_b_y",
    "alpha_dot",
    "B_aeroForce_z",
]

# Apply pd.to_numeric to each column in the list
for column in columns_to_convert:
    if column in df.columns:  # Only convert if column exists
        df[column] = pd.to_numeric(df[column], errors="coerce")
    else:
        print(f"Warning: Column '{column}' not found in CSV file")

# Convert alpha from radians to degrees
if "alpha_deg" not in df.columns:
    df["alpha_deg"] = np.degrees(df["alpha"])

# Calculate the norm of the velocity
if "velocity_norm" not in df.columns:
    df["velocity_norm"] = np.sqrt(
        df["vel_x"] ** 2 + df["vel_y"] ** 2 + df["vel_z"] ** 2
    )

print(df["velocity_norm"])
time = df["time"].values
velocity_norm = df["velocity_norm"].values

# First plot with two subplots: 3D trajectory and velocity norm vs time
fig, axs = plt.subplots(1, 1, figsize=(14, 6))
ax_3d = axs
# 3D trajectory plot (pos_x, pos_y, pos_z)
ax_3d = fig.add_subplot(121, projection="3d")
ax_3d.plot(df["pos_x"].values, df["pos_y"].values, df["pos_z"].values)
ax_3d.set_xlabel("Position X")
ax_3d.set_ylabel("Position Y")
ax_3d.set_zlabel("Position Z")
ax_3d.set_title("3D Trajectory")
plt.tight_layout()
plt.show()

# Second plot with two subplots: velocity norm and alpha vs aero_force_z with dual y-axis
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

# First subplot: velocity norm vs time
ax1.plot(time, velocity_norm, label="Velocity Norm")
ax1.set_xlabel("Time")
ax1.set_ylabel("Velocity Norm")
ax1.set_title("Velocity Norm vs Time")
ax1.grid(True)

# Second subplot: alpha (degrees) vs aero_force_z with dual y-axis
alpha_deg = df["alpha_deg"].values
aero_force_z = df["aero_force_z"].values

ax2.plot(time, alpha_deg, label="Alpha (degrees)", color="b")
ax2.set_xlabel("Time")
ax2.set_ylabel("Alpha (degrees)", color="b")
ax2.tick_params(axis="y", labelcolor="b")

# Create a second y-axis for aero_force_z
ax3 = ax2.twinx()
ax3.plot(time, aero_force_z, label="Aero Force Z", color="r")
ax3.set_ylabel("Aero Force Z", color="r")
ax3.tick_params(axis="y", labelcolor="r")

ax2.set_title("Alpha vs Aero Force Z with Dual Y-axis")

plt.tight_layout()
plt.show()

# Only create the third plot if both columns exist
if "B_Omega_b_y" in df.columns and "alpha_dot" in df.columns:
    # Third plot: B_Omega_b_y and alpha_dot with dual y-axis
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot B_Omega_b_y on the first y-axis
    ax.plot(time, df["B_Omega_b_y"].values, label="B_Omega_b_y", color="b")
    ax.set_xlabel("Time")
    ax.set_ylabel("B_Omega_b_y (rad/s)", color="b")
    ax.tick_params(axis="y", labelcolor="b")

    # Create a second y-axis for alpha_dot
    ax2 = ax.twinx()
    ax2.plot(time, df["alpha_dot"].values, label="alpha_dot", color="r")
    ax2.set_ylabel("alpha_dot (rad/s)", color="r")
    ax2.tick_params(axis="y", labelcolor="r")

    # Add legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    ax.set_title("Body Angular Velocity and Alpha Rate vs Time")
    ax.grid(True)

    plt.tight_layout()
    plt.show()
else:
    print("Warning: B_Omega_b_y and/or alpha_dot columns not found in CSV file")

# # Fourth plot: Compare aero_force_z and B_aeroForce_z
# if 'aero_force_z' in df.columns and 'B_aeroForce_z' in df.columns:
#     fig, ax = plt.subplots(figsize=(10, 6))

#     # Plot both forces on the same axis
#     ax.plot(time, df['aero_force_z'].values, label='aero_force_z', color='b')
#     ax.plot(time, df['B_aeroForce_z'].values, label='B_aeroForce_z', color='r')
#     ax.set_xlabel('Time')
#     ax.set_ylabel('Force (N)')
#     ax.legend()
#     ax.grid(True)

#     ax.set_title('Aerodynamic Forces Comparison')

#     plt.tight_layout()
#     plt.show()
# else:
#     print("Warning: aero_force_z and/or B_aeroForce_z columns not found in CSV file")


# Fourth plot: Compare aero_force_z and B_aeroForce_z
if "S_ap_x" in df.columns and "S_ap_z" in df.columns:
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))

    ax[0].plot(time, velocity_norm, label="Velocity", color="g")

    # Plot both forces on the same axis
    ax[1].plot(time, df["S_ap_x"].values, label="S_ap_x", color="b")
    ax[1].plot(time, df["S_ap_z"].values, label="S_ap_z", color="r")
    ax[1].set_xlabel("Time")
    ax[1].set_ylabel("Force (N)")
    ax[1].legend()
    ax[1].grid(True)

    ax[1].set_title("Aerodynamic Forces Comparison")

    plt.tight_layout()
    plt.show()
else:
    print("Warning: S_ap_x and/or S_ap_z columns not found in CSV file")
