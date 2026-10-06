# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

import os
import re

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt

# plt.style.use('science')
import numpy as np

#!/usr/bin/python3
import pandas as pd
from scipy.spatial.transform import Rotation as R

from tooling.env import project_path


def getEulerAnglesZXY(rotation):
    """
    Convert a rotation matrix or a scipy.spatial.transform.Rotation object
    to the ZXY Euler angles (ψ, φ, θ) intrinsic sequence.

    Parameters:
    - rotation: Rotation object (scipy.spatial.transform.Rotation)
      or a 3x3 rotation matrix.

    Returns:
    - psi, phi, theta: Euler angles corresponding to the ZXY sequence.
    """
    # Ensure the input is a Rotation object
    if isinstance(rotation, R):
        rot_matrix = rotation.as_matrix()  # Convert to rotation matrix
    else:
        rot_matrix = np.array(rotation)  # Assume input is already a matrix

    # Extract the elements from the rotation matrix
    R11, R12, R13 = rot_matrix[0, :]
    R21, R22, R23 = rot_matrix[1, :]
    R31, R32, R33 = rot_matrix[2, :]

    # Calculate Euler angles (ψ, φ, θ) from the rotation matrix
    # For ZXY intrinsic, the following formulas are used:

    # theta (rotation about y-axis)
    theta = np.arctan2(-R31, np.sqrt(R32**2 + R33**2))

    psi = np.arctan2(-R12, R22)  # yaw
    phi = np.arcsin(R32)          # roll
    # phi (rotation about x-axis)
    # phi = np.arctan2(R32 / np.cos(theta), R33 / np.cos(theta))

    # # psi (rotation about z-axis)
    # psi = np.arctan2(R21 / np.cos(theta), R11 / np.cos(theta))

    # Return Euler angles (ψ, φ, θ)
    return psi, phi, theta

def quaternionsToAnglesZXY(quaternions):
    """
    Convert a set of quaternions (w, x, y, z) to the ZXY Euler angles (ψ, φ, θ) intrinsic sequence.
    Each quaternion is represented as [w, x, y, z].

    Parameters:
    - quaternions: A numpy array of shape (N, 4) representing N quaternions in (w, x, y, z) format.

    Returns:
    - A numpy array of shape (N, 3) containing the Euler angles (ψ, φ, θ) for each quaternion.
    """
    # Reorder quaternions from (w, x, y, z) to (x, y, z, w)
    quaternions_reordered = np.column_stack((quaternions[:, 1:], quaternions[:, 0]))

    # Convert the reordered quaternions into Rotation objects
    rotations = R.from_quat(quaternions_reordered)

    # Get Euler angles in the ZXY sequence for each quaternion
    euler_angles = np.array([getEulerAnglesZXY(rotation) for rotation in rotations])

    return euler_angles




def quaternion_to_axes(quaternions):
    """
    Converts a sequence of quaternions (N, 4) into separate sequences of rotated axes (xb, yb, zb).
    Each quaternion describes a rotation applied to the original basis vectors (x, y, z).

    Parameters:
    quaternions (numpy.ndarray): Array of quaternions with shape (N, 4),
                                  where each quaternion is [w, x, y, z].

    Returns:
    tuple: Three numpy arrays representing the rotated x, y, and z axes for each quaternion:
           (xb, yb, zb) where:
             - xb: The rotated x-axis for each quaternion
             - yb: The rotated y-axis for each quaternion
             - zb: The rotated z-axis for each quaternion
    """
    if quaternions is None or quaternions.shape[1] != 4:
        raise ValueError("Input quaternion array must have shape (N, 4)")

    N = quaternions.shape[0]

    # Prepare arrays to store the rotated axes for each quaternion
    xb = np.zeros((N, 3))
    yb = np.zeros((N, 3))
    zb = np.zeros((N, 3))

    for i in range(N):
        w, x, y, z = quaternions[i]

        # Compute the rotation matrix from the quaternion
        R = np.array([
            [1 - 2 * (y**2 + z**2),  2 * (x * y - z * w),  2 * (x * z + y * w)],
            [2 * (x * y + z * w),    1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),    2 * (y * z + x * w),   1 - 2 * (x**2 + y**2)]
        ])

        # The columns of the rotation matrix are the rotated axes
        xb[i] = R[:, 0]  # x-axis (first column)
        yb[i] = R[:, 1]  # y-axis (second column)
        zb[i] = R[:, 2]  # z-axis (third column)

    return xb, yb, zb


def add_yaw_and_right_multiply_frd(quaternions, yaw_angle):
    """
    Add a yaw rotation to all quaternions and right-multiply by the FRD quaternion [0, 1, 0, 0].

    Args:
    - quaternions (N x 4): Array of quaternions, each quaternion is of the form [w, x, y, z].
    - yaw_angle (float): Yaw angle in radians to be added to all quaternions.

    Returns:
    - new_quaternions (N x 4): Array of new quaternions after adding yaw and applying right-multiply by [0, 1, 0, 0].
    """
    # Normalize quaternions to ensure they are unit quaternions
    quaternions = quaternions / np.linalg.norm(quaternions, axis=1)[:, np.newaxis]

    # Create the yaw quaternion (rotation around the Z-axis)
    w_yaw = np.cos(yaw_angle / 2)
    z_yaw = np.sin(yaw_angle / 2)
    yaw_quaternion = np.array([w_yaw, 0, 0, z_yaw])  # Yaw quaternion: [cos(psi/2), 0, 0, sin(psi/2)]

    # Create the FRD quaternion [0, 1, 0, 0]
    frd_quaternion = np.array([1, 0, 0, 0])  # Right-multiply quaternion [0, 1, 0, 0] (rotation around X-axis)

    # Initialize array for new quaternions
    new_quaternions = np.zeros_like(quaternions)

    # Perform quaternion multiplication (yaw_quaternion * q)
    for i in range(quaternions.shape[0]):
        q = quaternions[i]

        # First apply the yaw rotation: yaw_quaternion * q
        w1, x1, y1, z1 = yaw_quaternion
        w2, x2, y2, z2 = q

        # Perform quaternion multiplication (yaw_quaternion * q)
        w_yaw_new = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x_yaw_new = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y_yaw_new = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z_yaw_new = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        # Now, right multiply by the FRD quaternion [0, 1, 0, 0]
        w_frd, x_frd, y_frd, z_frd = frd_quaternion

        # Perform right multiplication (yaw quaternion * FRD quaternion)
        w_final = w_yaw_new * w_frd - x_yaw_new * x_frd - y_yaw_new * y_frd - z_yaw_new * z_frd
        x_final = w_yaw_new * x_frd + x_yaw_new * w_frd + y_yaw_new * z_frd - z_yaw_new * y_frd
        y_final = w_yaw_new * y_frd - x_yaw_new * z_frd + y_yaw_new * w_frd + z_yaw_new * x_frd
        z_final = w_yaw_new * z_frd + x_yaw_new * y_frd - y_yaw_new * x_frd + z_yaw_new * w_frd

        # Store the final result
        new_quaternions[i] = np.array([w_final, x_final, y_final, z_final])

    return new_quaternions


def quaternion_to_ZXY_euler_batch(q):
    # Normalize the quaternion to ensure they are unit quaternions
    q = q / np.linalg.norm(q, axis=1)[:, np.newaxis]  # Normalize each quaternion

    # Extract components
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    # Compute ZXY Euler angles (phi, theta, psi)
    theta = np.arcsin(2 * (w * y - x * z))  # pitch
    phi = np.arctan2(2 * (w * y + x * z), w**2 + x**2 - y**2 - z**2)  # roll
    psi = np.arctan2(2 * (w * z + x * y), w**2 - x**2 - y**2 + z**2)  # yaw

    # Stack the Euler angles into a (N, 3) array
    euler_angles = np.column_stack((phi, theta, psi))

    return euler_angles

def normalize_angle_to_2pi(angles):
    """
    Normalize an angle to the range [0, 2π].

    Parameters:
        angle (float): The input angle in radians.

    Returns:
        float: The angle normalized to the range [0, 2π].
    """
    # Normalize to [0, 2π]
    normalized_angles = np.mod(angles, 2 * np.pi)
    return normalized_angles

def extract_yaw_roll_from_yb(yb):
    """
    Extract yaw (psi) and roll (phi) from the input 3D numpy array.

    Parameters:
        yb (numpy.ndarray): A 3D numpy array with dimensions (..., 3),
                            where each element along the last axis is a vector [yb_x, yb_y, yb_z].

    Returns:
        numpy.ndarray: A 2D numpy array with two columns:
                       - The first column contains yaw (psi).
                       - The second column contains roll (phi).
    """
    # Ensure the input array has the correct shape (..., 3)
    if yb.shape[-1] != 3:
        raise ValueError("The last dimension of the input array must be 3.")

    # Extract components of yb
    yb_x = yb[..., 0]
    yb_y = yb[..., 1]
    yb_z = yb[..., 2]

    # Compute yaw (psi) and roll (phi)
    psi = np.arctan2(-yb_x, yb_y)  # yaw
    phi = np.arcsin(yb_z)          # roll

    # Stack the results into a 2D array
    result = np.stack((psi, phi), axis=-1)
    # scaling to deg
    result = result
    return result



def read_topic_from_ulog_export_csv(topic_name:str):
    """
    Load the tracking information CSV file from the specified directory.

    Parameters:
    tracking_info_dir (str): The directory to search for the tracking info file.
                             Defaults to the environment variable 'AP_PNC_DIR'
                             or the current directory if the environment variable is not set.

    Returns:
    pd.DataFrame: The data from the tracking info CSV file as a DataFrame.
    """
    # Set directory to environment variable or current directory if not provided
    directory = str(project_path('.artifacts/paper/exp'))

    # Update the directory to the 'output_csv/' subdirectory
    directory = os.path.join(directory, 'output_csv/')

    # Compile regex pattern to search for 'tracking_info' in filenames
    pattern = re.compile(fr'{topic_name}')

    # Find the first CSV file that matches the pattern
    csv_file_name = None
    for file in os.listdir(directory):
        if pattern.search(file) and file.endswith('.csv'):
            csv_file_name = file
            break

    # If no matching file is found, return None or raise an error
    if csv_file_name is None:
        raise FileNotFoundError("No tracking_info CSV file found in the specified directory.")

    # Read and return the CSV file as a DataFrame
    return pd.read_csv(os.path.join(directory, csv_file_name), delimiter=',')

# Create the main figure

def main():
    plt.rcParams.update({
        'font.size': 9,                 # Font size for readability (between 8-10)
        'font.family': 'serif',         # Set font family to serif for Times New Roman
        'font.serif': ['Times New Roman'],  # Specify Times New Roman as the serif font
        'axes.labelsize': 9,            # Font size for axis labels
        'axes.titlesize': 10,           # Font size for titles
        'legend.fontsize': 8,           # Font size for legends
        'xtick.labelsize': 8,           # Font size for x-axis tick labels
        'ytick.labelsize': 8,           # Font size for y-axis tick labels
        'figure.dpi': 300               # High resolution for publication quality
    })

    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'

    plt.rcParams['xtick.minor.size'] = 2                   # Minor tick length on the x-axis
    plt.rcParams['ytick.minor.size'] = 2                   # Minor tick length on the y-axis
    plt.rcParams['xtick.minor.width'] = .5                  # Minor tick width on the x-axis
    plt.rcParams['ytick.minor.width'] = .5                 # Minor tick width on the y-axis
    plt.rcParams['xtick.direction'] = 'in'                 # Minor tick direction (inward)
    plt.rcParams['ytick.direction'] = 'in'                 # Minor tick direction (inward)
    plt.rcParams['grid.linestyle'] = '--'  # Dashed grid lines
    plt.rcParams['grid.linewidth'] = 0.5   # Line width for grid lines (IEEE style)
    plt.rcParams['lines.linewidth'] = 1  # Line thickness in points (1pt)

    a4_width, a4_height = 7.25, 9.25
    fig = plt.figure(figsize=(a4_width, .3*a4_height), constrained_layout=True)

    # Create the grid layout
    gs = gridspec.GridSpec(2, 10, figure=fig)  # 2 rows x 6 columns for better control
    gs.update(wspace=0.3, hspace=0.0)  # Reduce spacing between subplots
    # Subfigure 1: A 4x3 subplot (spanning rows and columns)
    ax1 = fig.add_subplot(gs[:, :4], projection='3d')  # Spans all rows, first 4 columns
    ax1.set_title("Trajectory Overall")

    first_plot_end = 4
    stride = 3
    # Subfigure 2: A 2x2 subplot (occupies a smaller section)
    ax2 = fig.add_subplot(gs[0, first_plot_end:first_plot_end+stride])  # Top row, last 2 columns
    ax2.set_title("Delta position")
    ax2.minorticks_on()
    #
    #
    #
    #
    #
    #
    # Subfigure 3: Another smaller subplot below Subfigure 2
    ax3 = fig.add_subplot(gs[1,  first_plot_end:first_plot_end+stride])  # Bottom row, last 2 columns
    ax3.set_title("Velocity")
    ax3.minorticks_on()

    ax4 = fig.add_subplot(gs[0,  first_plot_end+stride:first_plot_end+2*stride])  # Top row, last 2 columns
    ax4.set_title("yb tracking and pitch")
    ax4.minorticks_on()

    # Subfigure 3: Another smaller subplot below Subfigure 2
    ax5 = fig.add_subplot(gs[1, first_plot_end+stride:first_plot_end+2*stride])  # Bottom row, last 2 columns
    ax5.set_title("Angular Velocity")
    ax5.minorticks_on()




    # ax1 content
    df = read_topic_from_ulog_export_csv('tracking_info')

    end_time_point = df['timestamp'].values[-1]
    start_time_point = df['timestamp'].values[0]

    # Convert to numpy arrays
    time = (df['timestamp'].values - df['timestamp'].values[0]) * 1e-6
    ref_p = df[['ref_position[0]', 'ref_position[1]', 'ref_position[2]']].values
    p = df[['actual_position[0]', 'actual_position[1]', 'actual_position[2]']].values
    ref_v = df[['ref_velocity[0]', 'ref_velocity[1]', 'ref_velocity[2]']].values
    v = df[['actual_velocity[0]', 'actual_velocity[1]', 'actual_velocity[2]']].values
    # Calculate velocity norms
    ref_v_norm = np.linalg.norm(ref_v, axis=1)
    v_norm = np.linalg.norm(v, axis=1)

    ax1.plot3D(ref_p[:, 0], ref_p[:, 1], ref_p[:, 2], 'green', label='Reference')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], 'red', label='Measured')
    ax1.grid(True)
    # Set 3D view perspective
    ax1.view_init(elev=30, azim=65)  # Adjust elevation and azimuth as needed
    ax1.set_zlim(np.min(ref_p[:, 2])-5, np.max(ref_p[:, 2])+5)  # Adjust z limits based on your data
    # ax2 content
    position_error = ref_p - p
    ax2.plot(time, position_error[:, 0])
    ax2.plot(time, position_error[:, 1])
    ax2.plot(time, position_error[:, 2])
    ax2.grid(True)

    # ax3 content
    ax3.plot(time, ref_v_norm, color='blue')
    ax3.plot(time, v_norm, color='red')
    ax3.grid(True)



    # ANGLE PLOT


    ref_yb = df[['ref_yb[0]', 'ref_yb[1]', 'ref_yb[2]']].values
    actual_yb = df[['actual_yb[0]', 'actual_yb[1]', 'actual_yb[2]']].values

    old_timestamp = df['timestamp'].values

    df = read_topic_from_ulog_export_csv('vehicle_odometry')
    df = df[df['timestamp'] > start_time_point - .5*1e6]

    new_timestamp = df['timestamp'].values
    # df = df[df['timestamp'] < end_time_point]
    q = df[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values.reshape(-1, 4)

    q = add_yaw_and_right_multiply_frd(q, -np.pi/2)
    # the q is (w,x,y,z) reorder to (x,y,z,w)
    new_q = np.column_stack((q[:, 1:], q[:, 0]))

    rotation = R.from_quat(new_q)
    euler_angles = rotation.as_euler('ZXY', degrees=False)  # In radians by default

    quaternionsToAnglesZXY(q)
    (df['timestamp'].values - df['timestamp'].values[0]) * 1e-6

    max_pitch = np.max(euler_angles[:,2])
    min_pitch = np.min(euler_angles[:,2])
    max_roll = np.max(euler_angles[:,1])

    border_more = 0.5

    ax4.set_ylim(-np.pi-border_more, np.pi+border_more)
    ticks = [-np.pi,-np.pi/2,  0, np.pi/2, np.pi]
    # add ticks with max_pitch
    ticks.append(max_pitch)
    ticks.append(min_pitch)
    ticks.append(max_roll)
    # tick_labels = [r'$-\pi$', r'$0$', r'$\pi$']  # LaTeX formatting for π
    tick_labels = [r'$-\pi$', r'$-\frac{\pi}{2}$', r'$0$', r'$\frac{\pi}{2}$', r'$\pi$']  # LaTeX formatting for π
    tick_labels.append('\0')
    tick_labels.append(f'{min_pitch:.1f}')
    tick_labels.append(f'{max_roll:.1f}')
    ax4.set_yticks(ticks)
    ax4.set_yticklabels(tick_labels)

    ref_angles = extract_yaw_roll_from_yb(ref_yb)
    extract_yaw_roll_from_yb(actual_yb)
    ax4.plot(old_timestamp, -ref_angles[:, 0], label='Reference Yaw', color='blue')
    ax4.plot(old_timestamp, ref_angles[:, 1], label='Reference Roll', color='green')
    # euler_angles
    ax4.plot(new_timestamp, euler_angles[:, 0], label='Actual Yaw', color='red')
    ax4.plot(new_timestamp, euler_angles[:, 1], label='Actual Yaw', color='orange')
    ax4.plot(new_timestamp, euler_angles[:, 2], label='Actual Yaw', color='cyan')



    ax4.set_ylabel('Angles [rad]')
    ax4.grid()
    directory = os.getenv('AP_PNC_DIR')




    df = read_topic_from_ulog_export_csv('vehicle_angular_velocity')
    # filter timestamp > time[0]
    df = df[df['timestamp'] > start_time_point]
    df = df[df['timestamp'] < end_time_point]

    new_time = (df['timestamp'].values - df['timestamp'].values[0]) * 1e-6
    ang_vel = df[['xyz[0]', 'xyz[1]', 'xyz[2]']].values
    #ang_vel is rad/s to deg/s
    ang_vel = ang_vel
    # gc_acc_norm = np.linalg.norm(gc_acc, axis=1)
    ax5.plot(new_time, ang_vel[:,0], label='omg_x', color='red')
    ax5.plot(new_time, ang_vel[:,1], label='omg_y', color='blue')
    ax5.plot(new_time, ang_vel[:,2], label='omg_z', color='green')


    df = read_topic_from_ulog_export_csv('vehicle_rates_setpoint')
    # filter timestamp > start and < end_time_point
    df = df[df['timestamp'] > start_time_point - .5*1e6]
    df = df[df['timestamp'] < end_time_point]

    new_time = (df['timestamp'].values - df['timestamp'].values[0]) * 1e-6
    ang_vel_sp = df[['roll', 'pitch', 'yaw']].values
    ang_vel_sp = ang_vel_sp

    ax5.plot(new_time, ang_vel_sp[:,0], label='omg_x_sp', color='orange')
    ax5.plot(new_time, ang_vel_sp[:,1], label='omg_y_sp', color='purple')
    ax5.plot(new_time, ang_vel_sp[:,2], label='omg_z_sp', color='black')

    ax5.grid(True)



    plt.savefig(f"{directory}/paper_plots/traj-view.pdf", format='pdf')

    plt.show()

    print(fig.get_size_inches())

if __name__ == "__main__":
    main()

