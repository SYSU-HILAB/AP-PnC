#!/usr/bin/python3
# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
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

from .exp_data_loader import UlgDataLoader
from .specify_figure_dirs import root_dir, tracking_plots_dir

colors = {
    "red": "#B43734",
    "cyan": "#397E77",
    "blue": "#00159D"
}

LabelsConfig = {
    "TrajXlabel": r'$\boldsymbol{p}_x~\text{[m]}$',
    "TrajYlabel": r'$\boldsymbol{p}_y~\text{[m]}$',
    "OthersXlablel": r'Time [s]',
    "PosErrorYlabel": r'$\boldsymbol{p}_{z}~\text{[m]}$',
    "AnglesYlabel": r'Angles [rad]',
    "VelYlabel": r'$V~\text{[m/s]}$',
    "AngVelYlabel": r'$\prescript{\mathcal{B}}{}{\boldsymbol{\Omega}_b}~\mathrm{[rad/s]}$',
    "PosErrorLim": [-1.0, 1.0],
    "AnglesLim": [-np.pi-0.5, np.pi+0.5],
    "AngVelLim": [-3.0, 3.0],
}

if_chinese = True

# TestName = "ma_v_4"

TestName = "lemniscate-v-14"
# TestName = "sim_proposed_circle_v_4v2"

'''lyu_v_14
simulation_v_14_cz_0.15_yb
indoor_v_2
indoor_v_2_yaw
outdoor_v_10
outdoor_v_12
lyu_v_14
sim_baseline_v_14
sim_baseline_v_13
sim_baseline_v_10
sim_baseline_v_7
sim_baseline_v_4
sim_baseline_v_2
sim_baseline_lem_v_2
sim_baseline_lem_v_4
sim_baseline_lem_v_7
sim_baseline_lem_v_10
sim_baseline_lem_v_13
sim_yb_lem_v_13
sim_yb_lem_v_10
sim_yb_lem_v_7
sim_yb_lem_v_4
sim_yb_lem_v_2
sim_yb_circle_v_2
sim_yb_circle_v_4
sim_yb_circle_v_7
sim_yb_circle_v_10
sim_yb_circle_v_13
sim_cz_circle_v_13
sim_cz_circle_v_10
sim_cz_circle_v_7
sim_cz_circle_v_4
sim_cz_circle_v_2
sim_cz_lem_v_13
sim_cz_lem_v_10
sim_cz_lem_v_7
sim_cz_lem_v_4
sim_cz_lem_v_2
sim_proposed_circle_v_13
sim_proposed_circle_v_10
sim_proposed_circle_v_7
sim_proposed_circle_v_4
sim_proposed_circle_v_2
GC_v_12
GC_v_14
'''


ExtraConfig = {
    "fix_yaw_angle": -np.pi/2,
    # "fix_yaw_angle": 0.0,
    # "fix_yaw_angle": -np.pi/4,
    # "ulg_file": os.path.join(root_dir(), '.artifacts/benchmark/ulgs/log_4_UnknownDate.ulg'),
    "ulg_file": os.path.join(root_dir(), f'.artifacts/benchmark/ulgs/{TestName}.ulg'),
    # "ulg_file": None,
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



# Utility functions
def normalize_quaternions(quaternions):
    """Normalize quaternions to unit length."""
    return quaternions / np.linalg.norm(quaternions, axis=1)[:, np.newaxis]

def read_csv_by_topic(directory, topic_name, start_time=None, end_time=None):
    """Read a CSV file by topic name and filter by time range if specified."""
    directory = directory or root_dir()
    directory = os.path.join(directory, '.artifacts/paper/exp/output_csv/')
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

    # the timestamp of original df is containing [start_time, end_time]
    # use inteplate1 to create a new df according to the timestamp_specified

    if start_time:
        df = df[df['timestamp'] > start_time]
    if end_time:
        df = df[df['timestamp'] < end_time]
    return df



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
    fig = plt.figure(figsize=(7.5, 3.0))
    gs = gridspec.GridSpec(2, 10, height_ratios=[1, 1], hspace=0.4, figure=fig)
    gs.update(wspace=2.0, hspace=0.6)

    # Create subplots
    ax1 = fig.add_subplot(gs[:, :4])  # Main 3D plot
    ax2 = fig.add_subplot(gs[0, 4:7])  # Position error
    ax3 = fig.add_subplot(gs[1, 4:7])  # Velocity
    ax4 = fig.add_subplot(gs[0, 7:10])  # Angles
    ax5 = fig.add_subplot(gs[1, 7:10])  # Angular velocities
    return fig, [ax1, ax2, ax3, ax4, ax5]

def plot_trajectory(ax, ref_p, p):
    """Plot 3D trajectory."""
    if if_chinese:
        ax.plot(ref_p[:, 0], ref_p[:, 1], '--',
              label=r'Ref.', color=f'{colors["red"]}', zorder=3)
        ax.plot(p[:, 0], p[:, 1], label=r'Est.',
              color=f'{colors["blue"]}', zorder=2)
        ax.set_xlabel(LabelsConfig["TrajXlabel"])
        ax.set_ylabel(LabelsConfig["TrajYlabel"])
    else:
        ax.plot(ref_p[:, 0], ref_p[:, 1], '--',
              label='Ref.', color=f'{colors["red"]}', zorder=3)
        ax.plot(p[:, 0], p[:, 1], label='Meas.',
              color=f'{colors["blue"]}', zorder=2)
        ax.set_xlabel(LabelsConfig["TrajXlabel"])
        ax.set_ylabel(LabelsConfig["TrajYlabel"])
    ax.get_position()  # 获取 axes 位置
    ax.legend(bbox_to_anchor=(0.5, 1.02),
              ncol=2, columnspacing=1.0,
              borderaxespad=0, handletextpad=0.2,
              loc='lower center'
              )

def plot_position_error(ax, time, ref_p, p):
    """Plot position error over time."""
    ref_p - p
    ax.plot(time, ref_p[:, 2],'--', color=f'{colors["red"]}', label='Ref.')
    ax.plot(time, p[:, 2], '-', color=f'{colors["blue"]}', label='Est.')
    ax.set_ylabel(LabelsConfig["PosErrorYlabel"], labelpad=-4)
    ax.set_ylim(LabelsConfig["PosErrorLim"])
    ax.legend(bbox_to_anchor=(0.5, 1.05),
            ncol=3, columnspacing=1.0,
            borderaxespad=0, handletextpad=0.2,
            loc='lower center'
            )

def plot_velocity(ax, time, ref_v, v):
    """Plot velocity over time."""
    ref_v_norm = np.linalg.norm(ref_v, axis=1)
    v_norm = np.linalg.norm(v, axis=1)
    if if_chinese:
        ax.plot(time, ref_v_norm, '--', label='Ref.',
                color=f'{colors["red"]}', zorder=3, linewidth=1.5)
        ax.plot(time, v_norm, label='Est.',
            color=f'{colors["blue"]}', zorder=2)

    else:
        ax.plot(time, ref_v_norm, '--', label='Ref.',
                color=f'{colors["red"]}', zorder=3, linewidth=1.5)
        ax.plot(time, v_norm, label='Meas.',
                color=f'{colors["blue"]}', zorder=2)
    # ax.set_title("Velocity")

    ax.set_ylabel(LabelsConfig["VelYlabel"])
    ax.set_xlabel(LabelsConfig["OthersXlablel"])
    ax.legend(bbox_to_anchor=(0.5, 1.05),
            ncol=2, columnspacing=1.0,
            borderaxespad=0, handletextpad=0.2,
            loc='lower center'
            )

def plot_angles(ax, main_time, odometry_time, ref_yb, actual_euler_angles):
    """
    Plot yb tracking performance and automatically determined pitch angle.

    Parameters:
    - ax: Matplotlib axis object to plot on.
    - main_time: Time array of shape (N1,), corresponding to the timeline for ref_yb.
    - odometry_time: Time array of shape (N2,), corresponding to the timeline for actual_euler_angles.
    - ref_yb: Reference yb vector array of shape (N1, 3).
    - actual_euler_angles: Actual Euler angles (yaw, roll, pitch) array of shape (N2, 3).
    """
    # Extract yaw and roll from the reference yb
    ref_angles = extract_yaw_roll_from_yb(ref_yb)  # Shape: (N1, 2)

    max_ref_roll = np.max(np.abs(ref_angles[:, 1]))
    print(f"Max reference roll angle: {max_ref_roll:.4f} rad ({np.degrees(max_ref_roll):.4f} deg)")


    # Plot actual yaw, roll, and pitch
    ax.plot(odometry_time, actual_euler_angles[:, 0], label=r'$\psi$',
            color=f'{colors["blue"]}', zorder=2)
    # ax.plot(odometry_time, actual_euler_angles[:, 2], label=r'$\theta$',
    #         color=f'{colors["cyan"]}', zorder=2)
    ax.plot(odometry_time, actual_euler_angles[:, 1], label=r'$\phi$',
            color=f'{colors["red"]}', zorder=2)
    # Plot reference yaw and roll from yb
    ax.plot(main_time, -ref_angles[:, 0], '--',
            color=f'{colors["blue"]}', zorder=3)
    ax.plot(main_time, ref_angles[:, 1], '--',
            color=f'{colors["red"]}', zorder=3)

    # tick that point on y-axis
    # set tick at the position pitch_min  not 0
    ax.set_ylim(LabelsConfig['AnglesLim'])
    ax.set_yticks([np.pi, np.pi/2, 0, -np.pi/2, -np.pi])
    ax.set_yticklabels([r'$\pi$', r'$\frac{\pi}{2}$', r'$0$', r'$-\frac{\pi}{2}$', r'$-\pi$'])
    # Configure axis labels, title, and grid
    ax.set_ylabel(LabelsConfig["AnglesYlabel"], labelpad=-1)
    ax.legend(bbox_to_anchor=(0.5, 1.05),
            ncol=3, columnspacing=1.0,
            borderaxespad=0, handletextpad=0.2,
            loc='lower center'
            )




def plot_angular_velocity(ax, time, ang_vel=None, ang_vel_sp=None):
    """Plot angular velocity and setpoints."""
    if ang_vel_sp is not None:
        ax.plot(time, ang_vel_sp[:, 0], '--', color=f'{colors["red"]}',zorder=3)
        ax.plot(time, ang_vel_sp[:, 1], '--', color=f'{colors["cyan"]}',zorder=3)
        ax.plot(time, ang_vel_sp[:, 2], '--', color=f'{colors["blue"]}',zorder=3)
    if ang_vel is not None:
        ax.plot(time, ang_vel[:, 0], color=f'{colors["red"]}', label='X',zorder=2)
        ax.plot(time, ang_vel[:, 1], color=f'{colors["cyan"]}', label='Y',zorder=2)
        ax.plot(time, ang_vel[:, 2], color=f'{colors["blue"]}', label='Z',zorder=2)
    # ax.set_title("Angular Velocities")

    #using latex
    ax.set_ylabel(LabelsConfig["AngVelYlabel"], labelpad=-2)
    ax.set_xlabel(LabelsConfig["OthersXlablel"])
    ax.set_ylim(LabelsConfig["AngVelLim"])
    ax.legend(bbox_to_anchor=(0.5, 1.05),
            ncol=3, columnspacing=1.0,
            borderaxespad=0, handletextpad=0.2,
            loc='lower center'
            )


# Reserved for export blender animation.
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
def main_plotting_pipeline():
    """Main function to process data and generate plots."""

    # tracking_plots_dir = tracking_plots_dir()
    ulg_loader = UlgDataLoader(ExtraConfig["ulg_file"])
    ulg_loader.load_data()

    # First, interpolate all topics to common timeline
    print("Interpolating all topics to common timeline...")
    ulg_loader.interpolate_topics_to_common_timeline()

    # Get the shifted tracking_info data
    df_tracking = ulg_loader.get_topic_data('tracking_info')

    ref_p = df_tracking[['ref_position[0]', 'ref_position[1]', 'ref_position[2]']].values
    p = df_tracking[['actual_position[0]', 'actual_position[1]', 'actual_position[2]']].values
    ref_v = df_tracking[['ref_velocity[0]', 'ref_velocity[1]', 'ref_velocity[2]']].values
    v = df_tracking[['actual_velocity[0]', 'actual_velocity[1]', 'actual_velocity[2]']].values
    main_time = df_tracking['time_from_start'].values

    # calculate the RMSE of the position error
    position_error = ref_p - p
    RMSE_position_error = np.sqrt(np.sum(np.sum(position_error**2, axis=1), axis=0)/len(position_error))
    print(f"RMSE of position error: {RMSE_position_error:.4f} m")

    # Calculate max height
    max_height = np.max(np.abs(position_error[:, 2]))
    print(f"Max height Error: {max_height:.4f} m")

    # Calculate max position
    max_pos = np.max(np.sqrt(np.sum(position_error**2, axis=1)))
    print(f"Max position Error magnitude: {max_pos:.4f} m")

    # Calculate velocity ratios
    ref_vel_mag = np.sqrt(np.sum(ref_v**2, axis=1))
    meas_vel_mag = np.sqrt(np.sum(v**2, axis=1))
    max_ref_vel = np.max(ref_vel_mag)
    max_meas_vel = np.max(meas_vel_mag)
    print(f"Max reference velocity: {max_ref_vel:.4f} m/s")
    print(f"Max measured velocity: {max_meas_vel:.4f} m/s")
    print(f"Velocity ratio (ref/meas): {max_ref_vel/max_meas_vel:.4f}")

    # Get the shifted vehicle_odometry data
    df_odometry = ulg_loader.get_topic_data('vehicle_odometry')
    odometry_time = df_odometry['time_from_start'].values
    quat = df_odometry[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values
    quat = add_yaw_and_right_multiply_frd(quat, ExtraConfig["fix_yaw_angle"])
    new_q = np.column_stack((quat[:, 1:], quat[:, 0]))

    rotation = R.from_quat(new_q)
    euler_angles = rotation.as_euler('ZXY', degrees=False)  # In radians by default

    # Calculate max roll
    max_roll = np.max(np.abs(euler_angles[:, 1]))  # X axis for roll in ZXY order
    print(f"Max roll angle: {max_roll:.4f} rad ({np.degrees(max_roll):.4f} deg)")

    max_pitch = np.max(np.abs(euler_angles[:, 2]))
    print(f"Max pitch angle: {max_pitch:.4f} rad ({np.degrees(max_pitch):.4f} deg)")

    # Initialize figure
    with plt.style.context("default"):
        fig, axes = initialize_figure()
        ref_yb = df_tracking[['ref_yb[0]', 'ref_yb[1]', 'ref_yb[2]']].values
        # Plot content
        plot_trajectory(axes[0], ref_p, p)
        plot_position_error(axes[1], main_time, ref_p, p)
        plot_velocity(axes[2], main_time, ref_v, v)
        plot_angles(axes[3], main_time, odometry_time, ref_yb=ref_yb, actual_euler_angles=euler_angles)

        df_ang_vel_sp = ulg_loader.get_topic_data('vehicle_rates_setpoint')
        ang_vel_sp = df_ang_vel_sp[['roll', 'pitch', 'yaw']].values
        ang_vel_sp_time = df_ang_vel_sp['time_from_start'].values
        plot_angular_velocity(axes[4], ang_vel_sp_time,
                            ang_vel_sp=ang_vel_sp, ang_vel=None)

        df_ang_vel = ulg_loader.get_topic_data('vehicle_angular_velocity')
        ang_vel = df_ang_vel[['xyz[0]', 'xyz[1]', 'xyz[2]']].values
        ang_vel_time = df_ang_vel['time_from_start'].values
        plot_angular_velocity(axes[4], ang_vel_time,
                            ang_vel_sp=None, ang_vel=ang_vel)

        # Save figure
        fig.savefig(f"{tracking_plots_dir()}/traj-view{TestName}.pdf", backend='pgf')

    fig1 = plt.figure(figsize=(7.5, 4.5))
    # Save figure
    fig1.savefig(f"{tracking_plots_dir()}/ang-view{TestName}.pdf", backend='pgf')

# Execute pipeline
if __name__ == "__main__":
    main_plotting_pipeline()
