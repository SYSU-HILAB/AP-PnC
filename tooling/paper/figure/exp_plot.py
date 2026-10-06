#!/usr/bin/python3
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
import numpy as np
import pandas as pd
from scipy.spatial import geometric_slerp
from scipy.spatial.transform import Rotation as R

from tooling.env import project_path

from .specify_figure_dirs import figure_path

colors = {
    "red": "#B43734",
    "cyan": "#397E77",
    "blue": "#00159D"
}

def calculate_distance(ref_p, p):
    return np.sqrt(np.sum((ref_p - p)**2, axis=1))



def slerp(t, q0, q1, t0, t1):
    """Perform Spherical Linear Interpolation (SLERP) between two quaternions.
    return: array (not R.quat)
    """
    fraction = (t - t0) / (t1 - t0)
    slerped_quat = geometric_slerp(q0, q1, fraction)

    return slerped_quat


def find_nearest_neighbors(t, original_timestamps):
    """
    Find the indices of the two nearest neighbors in original_timestamps for a given time t.
    :param t: The timestamp for which neighbors are to be found.
    :param original_timestamps: A sorted array of original timestamps.
    :return: Indices of the two nearest neighbors (idx1, idx2).
    """
    idx = (original_timestamps <= t).nonzero()[0].max()  # Find the largest timestamp <= t
    left_idx = max(idx, 0)  # Ensure left_idx is within bounds
    right_idx = min(idx + 1, len(original_timestamps) - 1)  # Ensure right_idx is within bounds
    return left_idx, right_idx

def set_plot_styles():
    """Set global matplotlib styles."""
    plt.rcParams.update({
        'font.size': 8,
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'legend.fontsize': 8,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'xtick.minor.size': 2,
        'ytick.minor.size': 2,
        'xtick.minor.width': .5,
        'ytick.minor.width': .5,
        'axes.labelpad': 0,
        'figure.dpi': 300,
        'grid.linestyle': '--',
        'grid.linewidth': 0.5,
        'lines.linewidth': 1,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'axes.prop_cycle': plt.cycler(color=[
            f'{colors["red"]}',
            f'{colors["blue"]}',
            f'{colors["cyan"]}'
        ]),
        'figure.facecolor': 'none',
        'axes.facecolor': 'none',
        'grid.color': 'gray',
        'grid.alpha': 0.5,
        'axes.edgecolor': 'black',
        'axes.grid': True,
    })
# Utility functions
def normalize_quaternions(quaternions):
    """Normalize quaternions to unit length."""
    return quaternions / np.linalg.norm(quaternions, axis=1)[:, np.newaxis]

def read_csv_by_topic(directory, topic_name, start_time=None, end_time=None):
    """Read a CSV file by topic name and filter by time range if specified."""
    # Use current directory's output_csv folder
    directory = str(project_path('.artifacts/paper/exp/output_csv'))
    pattern = re.compile(fr'{topic_name}')

    # Find the relevant CSV file
    csv_file_name = None
    for file in os.listdir(directory):
        if pattern.search(file) and file.endswith('.csv'):
            csv_file_name = file
            break

    if csv_file_name is None:
        raise FileNotFoundError(f"No CSV found for topic: {topic_name} in {directory}")

    # Read the CSV file
    df = pd.read_csv(os.path.join(directory, csv_file_name), delimiter=',')

    if start_time:
        df = df[df['timestamp'] > start_time]
    if end_time:
        df = df[df['timestamp'] < end_time]
    return df

def read_csv_by_topic_timeline_align(directory, topic_name, new_timestamps):
    """Read a CSV file by topic name and filter by time range if specified.
    timestamps is a vector that containing timestamps from timestamp[0] and timestamp[1].
    and it is uniformly spaced.
    """
    quaternion_columns = ['q[1]', 'q[2]', 'q[3]', 'q[0]']

    # Use current directory's output_csv folder
    directory = str(project_path('.artifacts/paper/exp/output_csv'))
    pattern = re.compile(fr'{topic_name}')

    # Convert new_timestamps to numpy array if it's a pandas Series
    if isinstance(new_timestamps, pd.Series):
        new_timestamps = new_timestamps.values

    # Find the relevant CSV file
    csv_file_name = None
    for file in os.listdir(directory):
        if pattern.search(file) and file.endswith('.csv'):
            csv_file_name = file
            break

    if csv_file_name is None:
        raise FileNotFoundError(f"No CSV found for topic: {topic_name} in {directory}")

    # Read the CSV file
    df = pd.read_csv(os.path.join(directory, csv_file_name), delimiter=',')

    if 'timestamp' not in df.columns:
        raise ValueError("CSV does not contain a 'timestamp' column.")

    # Sort both timestamp arrays
    original_timestamps = df['timestamp'].values

    try:
        if new_timestamps[0] < original_timestamps[0] or new_timestamps[-1] > original_timestamps[-1]:
            raise ValueError(f"New timestamps {new_timestamps[0]}-{new_timestamps[-1]} are outside the original range {original_timestamps[0]}-{original_timestamps[-1]}")
    except (IndexError, TypeError) as e:
        raise ValueError("Invalid timestamp format or empty timestamp array") from e

    df.set_index('timestamp', inplace=True)

    # Check if quaternion columns exist
    has_quaternions = all(col in df.columns for col in quaternion_columns)

    scalar_columns = [col for col in df.columns if (not has_quaternions or col not in quaternion_columns) and col != 'timestamp']

    # Interpolate scalar values linearly, but keep NaN values as NaN
    scalar_df_interpolated = pd.DataFrame(index=new_timestamps)
    for col in scalar_columns:
        # Only interpolate non-NaN values
        mask = ~df[col].isna()
        if mask.any():  # If there are any non-NaN values
            valid_data = df[col][mask]
            scalar_df_interpolated[col] = np.interp(
                new_timestamps,
                valid_data.index.values,
                valid_data.values,
                left=np.nan,  # Use NaN for extrapolation
                right=np.nan
            )
        else:  # If column is all NaN
            scalar_df_interpolated[col] = np.nan

    # Only do quaternion interpolation if the columns exist
    if has_quaternions:
        # Get quaternions and normalize them
        quaternions = df[quaternion_columns].values
        # Only normalize non-NaN quaternions
        valid_quat_mask = ~np.any(np.isnan(quaternions), axis=1)
        quaternions[valid_quat_mask] = quaternions[valid_quat_mask] / np.linalg.norm(quaternions[valid_quat_mask], axis=1, keepdims=True)

        # Interpolate quaternion columns with SLERP, but only if both neighbors are valid
        interpolated_quaternions = []
        for t in new_timestamps:
            idx1, idx2 = find_nearest_neighbors(t, original_timestamps)
            t0, t1 = original_timestamps[idx1], original_timestamps[idx2]
            q0 = quaternions[idx1]
            q1 = quaternions[idx2]

            # Check if either quaternion contains NaN
            if np.any(np.isnan(q0)) or np.any(np.isnan(q1)):
                interpolated_quaternions.append([np.nan] * 4)
            else:
                interpolated_quaternions.append(slerp(t, q0, q1, t0, t1))

        # Create quaternion DataFrame with proper index
        quat_df = pd.DataFrame(interpolated_quaternions,
                             index=new_timestamps,
                             columns=quaternion_columns)

        # Concatenate with matching indices
        df_interpolated = pd.concat([scalar_df_interpolated, quat_df], axis=1)
    else:
        df_interpolated = scalar_df_interpolated

    df_interpolated['timestamp'] = new_timestamps

    return df_interpolated




def extract_yaw_roll_from_yb(yb):
    """Extract yaw (psi) and roll (phi) from yb vectors."""
    yb_x, yb_y, yb_z = yb[:, 0], yb[:, 1], yb[:, 2]
    psi = np.arctan2(-yb_x, yb_y)
    phi = np.arcsin(yb_z)
    return np.column_stack((psi, phi))

def add_yaw_and_right_multiply_frd(quaternions, yaw_angle):
    """Add a yaw rotation and right-multiply by FRD quaternion."""
    quaternions = normalize_quaternions(quaternions)
    w_yaw = np.cos(yaw_angle / 2)
    z_yaw = np.sin(yaw_angle / 2)
    yaw_quaternion = np.array([w_yaw, 0, 0, z_yaw])
    frd_quaternion = np.array([1, 0, 0, 0])

    new_quaternions = np.zeros_like(quaternions)
    for i, q in enumerate(quaternions):
        # Apply yaw rotation
        w1, x1, y1, z1 = yaw_quaternion
        w2, x2, y2, z2 = q
        w_yaw_new = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x_yaw_new = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y_yaw_new = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z_yaw_new = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        # Right multiply by FRD quaternion
        w3, x3, y3, z3 = frd_quaternion
        new_quaternions[i] = [
            w_yaw_new * w3 - x_yaw_new * x3 - y_yaw_new * y3 - z_yaw_new * z3,
            w_yaw_new * x3 + x_yaw_new * w3 + y_yaw_new * z3 - z_yaw_new * y3,
            w_yaw_new * y3 - x_yaw_new * z3 + y_yaw_new * w3 + z_yaw_new * x3,
            w_yaw_new * z3 + x_yaw_new * y3 - y_yaw_new * x3 + z_yaw_new * w3
        ]
    return new_quaternions

# Plotting functions
def initialize_figure():
    """Initialize a multi-panel figure with a grid layout."""
    a4_width, a4_height = 7.25, 9
    fig = plt.figure(figsize=(a4_width, 0.3 * a4_height), constrained_layout=True)
    gs = gridspec.GridSpec(2, 10, height_ratios=[1, 1], hspace=0.4, figure=fig)
    gs.update(wspace=0.3, hspace=0.0)

    # Create subplots
    ax1 = fig.add_subplot(gs[:, :4], projection='3d')  # Main 3D plot
    ax2 = fig.add_subplot(gs[0, 4:7])  # Position error
    ax3 = fig.add_subplot(gs[1, 4:7])  # Velocity
    ax4 = fig.add_subplot(gs[0, 7:10])  # Angles
    ax5 = fig.add_subplot(gs[1, 7:10])  # Angular velocities
    return fig, [ax1, ax2, ax3, ax4, ax5]

def plot_trajectory(ax, ref_p, p, padding=5):
    """Plot 3D trajectory."""
    ax.plot3D(ref_p[:, 0], ref_p[:, 1], ref_p[:, 2], '--',
              label='Reference', color=f'{colors["red"]}', zorder=3)
    ax.plot3D(p[:, 0], p[:, 1], p[:, 2], label='Measured',
              color=f'{colors["blue"]}', zorder=2)
    ax.view_init(elev=30, azim=65)

    ax.set_zlim(np.min(ref_p[:, 2])-padding, np.max(ref_p[:, 2])+padding)  # Adjust z limits based on your data
    # ax2 content
        # Set tick lines opacity
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        for line in axis.get_ticklines():
            line.set_alpha(0.8)  # Set tick line transparency


def plot_position_error(ax, time, ref_p, p):
    """Plot position error over time."""
    position_error = ref_p - p
    ax.plot(time, position_error[:, 0], label='X Error')
    ax.plot(time, position_error[:, 1], label='Y Error')
    ax.plot(time, position_error[:, 2], label='Z Error')
    # ax.set_title("Delta Position")

    ax.set_ylabel(r'$\mathbf{p}_{e}$ [m]')
    ax.set_xlabel('Time [s]')
    ax.grid(True)
augment_dashed_line_width = 1.5
def plot_velocity(ax, time, ref_v, v):
    """Plot velocity over time."""
    ref_v_norm = np.linalg.norm(ref_v, axis=1)
    v_norm = np.linalg.norm(v, axis=1)
    ax.plot(time, ref_v_norm, '--', label='Reference Velocity',
            color=f'{colors["red"]}', zorder=3, linewidth=1.5)
    ax.plot(time, v_norm, label='Measured Velocity',
            color=f'{colors["blue"]}', zorder=2)
    # ax.set_title("Velocity")

    ax.set_ylabel(r'$\mathbf{v}$ [m/s]')
    ax.set_xlabel('Time [s]')
    ax.grid(True)

def limit_angle_array(angles, min_angle, max_angle):
    """Limit angles to be within [min_angle, max_angle].
    This function handles angle wrapping in a robust way without jumps.

    Args:
        angles (np.ndarray): Array of angles to be limited, shape [N,1] or [N,]
        min_angle (float): Minimum angle (e.g., -pi)
        max_angle (float): Maximum angle (e.g., pi)

    Returns:
        np.ndarray: Limited angles with same shape as input
    """
    range_size = max_angle - min_angle
    # Ensure the input is a numpy array
    angles = np.asarray(angles)

    # First bring angles into the range [min_angle, min_angle + 2π]
    normalized = angles - min_angle
    normalized = normalized % (2 * np.pi)
    angles_in_range = normalized + min_angle

    # Then handle any angles that might be slightly above max_angle
    mask = angles_in_range > max_angle
    angles_in_range[mask] -= range_size

    return angles_in_range

def plot_angles(ax, main_time, odometry_time, ref_yb, actual_euler_angles):
    """Plot yb tracking performance and automatically determined pitch angle."""
    # Extract yaw and roll from the reference yb
    ref_angles = extract_yaw_roll_from_yb(ref_yb)  # Shape: (N1, 2)

    # Define y-axis limits dynamically based on the data range
    pitch_min, _pitch_max = np.min(actual_euler_angles[:, 2]) * 180/np.pi , np.max(actual_euler_angles[:, 2]) * 180/np.pi
    _yaw_min, _yaw_max = np.min(actual_euler_angles[:, 0]), np.max(actual_euler_angles[:, 0])
    np.max(actual_euler_angles[:, 1])

    yaw_diff = ref_angles[0, 0] - actual_euler_angles[0, 0]
    actual_euler_angles[:, 0] = actual_euler_angles[:, 0] - yaw_diff
    # Limit ref yaw to be within -pi to pi
    ref_angles[:, 0] = limit_angle_array(ref_angles[:, 0], -np.pi, np.pi)



    # Limit actual yaw and pitch to be within -pi to pi
    actual_euler_angles[:, 0] = limit_angle_array(actual_euler_angles[:, 0], -np.pi, np.pi)
    actual_euler_angles[:, 2] = limit_angle_array(actual_euler_angles[:, 2], -np.pi, np.pi)

    # Plot reference yaw and roll from yb
    ax.plot(main_time, -ref_angles[:, 0] * 180/np.pi, '--', label='Reference Yaw',
            color=f'{colors["blue"]}', zorder=3, linewidth=augment_dashed_line_width)
    ax.plot(main_time, ref_angles[:, 1] * 180/np.pi, '--', label='Reference Roll',
            color=f'{colors["red"]}', zorder=3, linewidth=augment_dashed_line_width)


    # Plot actual yaw, roll, and pitch
    ax.plot(odometry_time, actual_euler_angles[:, 0] * 180/np.pi, label='Actual Yaw',
            color=f'{colors["blue"]}', zorder=2)
    ax.plot(odometry_time, actual_euler_angles[:, 1] * 180/np.pi, label='Actual Roll',
            color=f'{colors["red"]}', zorder=2)
    ax.plot(odometry_time, actual_euler_angles[:, 2] * 180/np.pi, label='Actual Pitch',
            color=f'{colors["cyan"]}', zorder=2)
    # tick that point on y-axis
    # set tick at the position pitch_min  not 0
    ax.set_yticks([pitch_min, 0, 180])
    ax.set_yticklabels([f'{pitch_min:.1f}', f'{0:.1f}', f'{180:.1f}'])
    # Configure axis labels, title, and grid
    ax.set_ylabel('Angles [rad]')
    ax.set_xlabel('Time [s]')
    # ax.set_title('yb Tracking and Pitch Angle Performance')
    ax.grid(True)





def plot_angular_velocity(ax, time, ang_vel=None, ang_vel_sp=None):
    """Plot angular velocity and setpoints."""
    if ang_vel_sp is not None:
        ax.plot(time, ang_vel_sp[:, 0], '--', color=f'{colors["red"]}', label='Roll Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
        ax.plot(time, ang_vel_sp[:, 1], '--', color=f'{colors["cyan"]}', label='Pitch Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
        ax.plot(time, ang_vel_sp[:, 2], '--', color=f'{colors["blue"]}', label='Yaw Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
    if ang_vel is not None:
        ax.plot(time, ang_vel[:, 0], color=f'{colors["red"]}', label='Measured Roll Rate',zorder=2)
        ax.plot(time, ang_vel[:, 1], color=f'{colors["cyan"]}', label='Measured Pitch Rate',zorder=2)
        ax.plot(time, ang_vel[:, 2], color=f'{colors["blue"]}', label='Measured Yaw Rate',zorder=2)
    # ax.set_title("Angular Velocities")

    #using latex
    ax.set_ylabel(r'$\Omega$ [rad/s]')
    ax.set_xlabel('Time [s]')
    ax.grid(True)

def save_plot(fig, save_path):
    """Save the figure to a specified path."""
    fig.savefig(save_path, format='pdf')

def create_df_export(df_odometry):
    """Create a DataFrame for exporting to CSV."""
    # create a new df_export, the header is time (starting from 0.0), px, py,pz, vx, vy, vz, qw, qx, qy, qz
    df_export = pd.DataFrame()



    df_export['time'] = df_odometry['timestamp']
    df_export['px'] = df_odometry['position[0]']
    df_export['py'] = df_odometry['position[1]']
    df_export['pz'] = df_odometry['position[2]']
    df_export['vx'] = df_odometry['velocity[0]']
    df_export['vy'] = df_odometry['velocity[1]']
    df_export['vz'] = df_odometry['velocity[2]']

    quat = df_odometry[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values
    # quat = add_yaw_and_right_multiply_frd(quat, np.pi/2)
    new_q = np.column_stack((quat[:, 1:], quat[:, 0]))
    RR = R.from_quat(new_q)
    euler_test = RR.as_euler('ZXY', degrees=False)
    euler_test[:, 0] =  euler_test[:, 0]
    # euler_test[:, 1] =
    euler_test[:, 1] =  -euler_test[:, 1]
    euler_test[:, 2] =  -euler_test[:, 2]

    # euler_test = np.zeros_like(euler_test)

    new_R = R.from_euler('ZXY', euler_test, degrees=False)
    new_q = new_R.as_quat()
    # to FLU  world also
    df_export['qw'] = new_q[:, 3]
    df_export['qx'] = new_q[:, 0]
    df_export['qy'] = new_q[:, 1]
    df_export['qz'] = new_q[:, 2]

    df_export['yaw'] = euler_test[:, 0]
    df_export['roll'] = euler_test[:, 1]
    df_export['pitch'] = euler_test[:, 2]

    return df_export

# Main processing pipeline
def main_plotting_pipeline(directory=None):
    """Main function to process data and generate plots."""
    set_plot_styles()

    # Load data & remove 2:inf circles, only keep the first circle
    df_tracking = read_csv_by_topic(directory, 'tracking_info')

    # Find and remove data before timestamp jumps > 1s
    timestamps = df_tracking['timestamp'].values
    time_diffs = np.diff(timestamps) * 1e-6  # Convert to seconds
    jump_indices = np.where(time_diffs > 1.0)[0]

    if len(jump_indices) > 0:
        start_idx = jump_indices[0] + 1
        df_tracking = df_tracking.iloc[start_idx:]
        print(f"Removed {start_idx} rows before first timestamp jump")

    # Now find the first circle
    ref_p = df_tracking[['ref_position[0]', 'ref_position[1]', 'ref_position[2]']].values
    find_result = np.where(ref_p[1:,0] == ref_p[0,0])[0]
    find_idx = find_result[0] if len(find_result) > 0 else -1

    df_tracking = df_tracking.iloc[0:find_idx]
    ref_p = df_tracking[['ref_position[0]', 'ref_position[1]', 'ref_position[2]']].values
    p = df_tracking[['actual_position[0]', 'actual_position[1]', 'actual_position[2]']].values
    ref_v = df_tracking[['ref_velocity[0]', 'ref_velocity[1]', 'ref_velocity[2]']].values
    v = df_tracking[['actual_velocity[0]', 'actual_velocity[1]', 'actual_velocity[2]']].values
    start_time, end_time = df_tracking['timestamp'].values[0], df_tracking['timestamp'].values[-1]
    main_time = (df_tracking['timestamp'].values - start_time) * 1e-6
    print(f"Time range: {start_time} to {end_time} microseconds")

    # calculate the RMSE of the position error
    position_error = ref_p - p
    RMSE_position_error = np.sqrt(np.sum(np.sum(position_error**2, axis=1), axis=0)/len(position_error))
    print(f"RMSE of position error: {RMSE_position_error:.4f} m")
    print(f"MAX of position error: {np.max(np.linalg.norm(position_error, axis=1)):.4f} m")



    main_time_line = df_tracking['timestamp'].values
    df_odometry = read_csv_by_topic_timeline_align(directory, 'odometry', main_time_line)

    odometry_time = (df_odometry['timestamp'].values - start_time) * 1e-6
    df_odometry['timestamp'] = odometry_time
    quat = df_odometry[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values
    # print(quat)
    quat = add_yaw_and_right_multiply_frd(quat, -np.pi/2)
    new_q = np.column_stack((quat[:, 1:], quat[:, 0]))

    rotation = R.from_quat(new_q)
    euler_angles = rotation.as_euler('ZXY', degrees=False)  # In radians by default

    # Initialize figure
    fig, axes = initialize_figure()
    ref_yb = df_tracking[['ref_yb[0]', 'ref_yb[1]', 'ref_yb[2]']].values
    # Plot content
    plot_trajectory(axes[0], ref_p, p)
    plot_position_error(axes[1], main_time, ref_p, p)
    plot_velocity(axes[2], main_time, ref_v, v)
    plot_angles(axes[3], main_time, odometry_time, ref_yb=ref_yb, actual_euler_angles=euler_angles)  # Replace with correct angle data


    df_ang_vel_sp = read_csv_by_topic_timeline_align(directory, 'vehicle_rates_setpoint',
                                                     main_time_line)
    ang_vel_sp = df_ang_vel_sp[['roll', 'pitch', 'yaw']].values
    ang_vel_sp_time = (df_ang_vel_sp['timestamp'].values - start_time) * 1e-6
    plot_angular_velocity(axes[4], ang_vel_sp_time,
                          ang_vel_sp=ang_vel_sp, ang_vel=None)  # Replace with correct angular velocity data



    df_ang_vel = read_csv_by_topic_timeline_align(directory, 'angular_velocity',
                                                  main_time_line)
    ang_vel = df_ang_vel[['xyz[0]', 'xyz[1]', 'xyz[2]']].values
    ang_vel_time = (df_ang_vel['timestamp'].values - start_time) * 1e-6
    plot_angular_velocity(axes[4], ang_vel_time,
                          ang_vel_sp=None, ang_vel=ang_vel)  # Replace with correct angular velocity data


    # Save figure
    save_plot(fig, figure_path("traj-view.pdf"))
    plt.show()

# Execute pipeline
if __name__ == "__main__":
    main_plotting_pipeline()
