import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import matplotlib.gridspec as gridspec
from scipy.spatial import geometric_slerp

colors = {
  "dark_blue": "#004488",
  "dark_red": "#BB5566",
  "dark_yellow": "#DDAA33",
  "light_blue": "#004488",
  "light_red": "#BB5566",
  "light_yellow": "#DDAA33"
}


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
# colors = {
#   "dark_blue": "#004488",
#   "dark_red": "#994455",
#   "dark_yellow": "#997700",
#   "light_blue": "#6699CC",
#   "light_red": "#EE99AA",
#   "light_yellow": "#EECC66"
# }
# Set matplotlib styles
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
        # High-contrast color cycle
        'axes.prop_cycle': plt.cycler(color=[
            # f'{colors["light_yellow"]}',  # Light Yellow
            # f'{colors["light_red"]}',   # Light Red
            # f'{colors["light_blue"]}',  # Light Blue
            f'{colors["dark_blue"]}',  # Dark Blue
            f'{colors["dark_red"]}',   # Dark Red
            f'{colors["dark_yellow"]}',  # Dark Yellow
        ]),
        # Set axes and figure background to transparent
        'figure.facecolor': 'none',  # You can also set this to 'none' if desired
        'axes.facecolor': 'none',     # Transparent axes background
        # Grid settings for high contrast
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
    directory = directory or os.getenv('TAILSITTER_PLANNING_DIR') or '.'
    directory = os.path.join(directory, 'ulgs/output_csv/')
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

def read_csv_by_topic_timeline_align(directory, topic_name, new_timestamps):
    """Read a CSV file by topic name and filter by time range if specified.
    timestamps is a vector that containing timestamps from timestamp[0] and timestamp[1].
    and it is uniformly spaced.
    """
    quaternion_columns = ['q[1]', 'q[2]', 'q[3]', 'q[0]']
    
    directory = directory or os.getenv('TAILSITTER_PLANNING_DIR') or '.'
    directory = os.path.join(directory, 'ulgs/output_csv/')
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
    
    if 'timestamp' not in df.columns:
        raise ValueError("CSV does not contain a 'timestamp' column.")
    original_timestamps = df['timestamp'].values
    
    df.set_index('timestamp', inplace=True)
    
    
    scalar_columns = [col for col in df.columns if col not in 
                      quaternion_columns and col != 'timestamp']

# Interpolate scalar values linearly
    scalar_df_interpolated = df[scalar_columns].reindex(new_timestamps).interpolate(method='linear')

    # Interpolate quaternion columns with SLERP
    quaternions = df[quaternion_columns].values
    quaternions = quaternions/np.linalg.norm(quaternions,axis=1, keepdims=True)
    interpolated_quaternions = []
    for t in new_timestamps:
        idx1, idx2 = find_nearest_neighbors(t, original_timestamps=original_timestamps) # Find nearest neighbors for t
        t0, t1 = original_timestamps[idx1], original_timestamps[idx2]
        q0, q1 = quaternions[idx1], quaternions[idx2]
        interpolated_quaternions.append(slerp(t, q0, q1, t0, t1))

    # Combine results
    df_interpolated = pd.concat([scalar_df_interpolated, pd.DataFrame(interpolated_quaternions, columns=quaternion_columns)], axis=1)
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
              label='Reference', color = f'{colors["dark_red"]}',zorder=3)
    ax.plot3D(p[:, 0], p[:, 1], p[:, 2], label='Measured', color = f'{colors["dark_blue"]}',zorder=2)
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
            color = f'{colors["dark_red"]}',zorder=3, linewidth=1.5)
    ax.plot(time, v_norm, label='Measured Velocity', color = f'{colors["dark_blue"]}',zorder=2)
    # ax.set_title("Velocity")
    
    ax.set_ylabel(r'$\mathbf{v}$ [m/s]')
    ax.set_xlabel('Time [s]')
    ax.grid(True)

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

    # Define y-axis limits dynamically based on the data range
    pitch_min, pitch_max = np.min(actual_euler_angles[:, 2]), np.max(actual_euler_angles[:, 2])
    yaw_min, yaw_max = np.min(actual_euler_angles[:, 0]), np.max(actual_euler_angles[:, 0])
    roll_max = np.max(actual_euler_angles[:, 2])
    border_more = 0.5  # Add buffer to axis limits

    ax.set_ylim(min(-np.pi, pitch_min) - border_more, max(np.pi, pitch_max) + border_more)

    # Set y-axis ticks and labels, including dynamic limits
    ticks = [-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi, pitch_min, pitch_max, roll_max]
    tick_labels = [
        r'$-\pi$', r'$-\frac{\pi}{2}$', r'$0$', r'$\frac{\pi}{2}$', r'$\pi$',
        f'{pitch_min:.1f}', f'{pitch_max:.1f}', f'{roll_max:.1f}'
    ]
    ax.set_yticks(ticks)
    ax.set_yticklabels(tick_labels)

    # Plot reference yaw and roll from yb
    ax.plot(main_time, -ref_angles[:, 0], '--', label='Reference Yaw', 
            color = f'{colors["light_yellow"]}',zorder=3, linewidth=augment_dashed_line_width)
    ax.plot(main_time, ref_angles[:, 1], '--', label='Reference Roll', 
            color = f'{colors["light_blue"]}',zorder=3, linewidth=augment_dashed_line_width)

    
    # Plot actual yaw, roll, and pitch
    ax.plot(odometry_time, actual_euler_angles[:, 0], label='Actual Yaw',
            color = f'{colors["dark_yellow"]}',zorder=2)
    ax.plot(odometry_time, actual_euler_angles[:, 1], label='Actual Roll', 
            color = f'{colors["dark_blue"]}',zorder=2)
    ax.plot(odometry_time, actual_euler_angles[:, 2], label='Actual Pitch', 
            color = f'{colors["dark_red"]}',zorder=2)

    # Configure axis labels, title, and grid
    ax.set_ylabel('Angles [rad]')
    ax.set_xlabel('Time [s]')
    # ax.set_title('yb Tracking and Pitch Angle Performance')
    ax.grid(True)
    




def plot_angular_velocity(ax, time, ang_vel=None, ang_vel_sp=None):
    """Plot angular velocity and setpoints."""
    if ang_vel_sp is not None:
        ax.plot(time, ang_vel_sp[:, 0], '--', label='Roll Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
        ax.plot(time, ang_vel_sp[:, 1], '--', label='Pitch Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
        ax.plot(time, ang_vel_sp[:, 2], '--', label='Yaw Rate Setpoint',zorder=3, linewidth=augment_dashed_line_width)
    if ang_vel is not None:
        ax.plot(time, ang_vel[:, 0], label='Measured Roll Rate',zorder=2)
        ax.plot(time, ang_vel[:, 1], label='Measured Pitch Rate',zorder=2)
        ax.plot(time, ang_vel[:, 2], label='Measured Yaw Rate',zorder=2)    
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
    directory = directory or os.getenv('TAILSITTER_PLANNING_DIR')
    
    # Load data
    df_tracking = read_csv_by_topic(directory, 'tracking_info')
    start_time, end_time = df_tracking['timestamp'].values[0], df_tracking['timestamp'].values[-1]
    main_time = (df_tracking['timestamp'].values - start_time) * 1e-6
    ref_p = df_tracking[['ref_position[0]', 'ref_position[1]', 'ref_position[2]']].values
    p = df_tracking[['actual_position[0]', 'actual_position[1]', 'actual_position[2]']].values
    ref_v = df_tracking[['ref_velocity[0]', 'ref_velocity[1]', 'ref_velocity[2]']].values
    v = df_tracking[['actual_velocity[0]', 'actual_velocity[1]', 'actual_velocity[2]']].values
    
    df_odometry = read_csv_by_topic(directory, 'odometry', start_time, end_time)
    
    
    
    odometry_time = (df_odometry['timestamp'].values - start_time) * 1e-6    
    df_odometry['timestamp'] = odometry_time
    quat = df_odometry[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values
    quat = add_yaw_and_right_multiply_frd(quat, -np.pi/2)
    new_q = np.column_stack((quat[:, 1:], quat[:, 0]))
    

    
    
    df_export = read_csv_by_topic_timeline_align(directory, 'odometry', df_tracking['timestamp'])
    print(df_export)
    df_export = create_df_export(df_odometry)
    
    
    
    df_export.to_csv(f"{directory}/odometry.csv", index=False)
    
    new_qq = df_export[['qx', 'qy', 'qz', 'qw']].values
    RR = R.from_quat(new_qq)
    euler_test = RR.as_euler('ZXY', degrees=False)
    test_time = df_export['time'].values
    fig, axes = initialize_figure()
    
    axes[1].plot(euler_test[:, 0]*180/np.pi, label='roll')
    axes[2].plot(euler_test[:, 1]*180/np.pi, label='pitch')
    axes[4].plot(euler_test[:, 2]*180/np.pi, label='pitch')

    plt.show()
    # rotation = R.from_quat(new_q)
    # euler_angles = rotation.as_euler('ZXY', degrees=False)  # In radians by default

    # # Initialize figure
    # fig, axes = initialize_figure()
    # ref_yb = df_tracking[['ref_yb[0]', 'ref_yb[1]', 'ref_yb[2]']].values
    # # Plot content
    # plot_trajectory(axes[0], ref_p, p)
    # plot_position_error(axes[1], main_time, ref_p, p)
    # plot_velocity(axes[2], main_time, ref_v, v)
    # plot_angles(axes[3], main_time, odometry_time, ref_yb=ref_yb, actual_euler_angles=euler_angles)  # Replace with correct angle data


    # df_ang_vel_sp = read_csv_by_topic(directory, 'vehicle_rates_setpoint', start_time, end_time)
    # ang_vel_sp = df_ang_vel_sp[['roll', 'pitch', 'yaw']].values
    # ang_vel_sp_time = (df_ang_vel_sp['timestamp'].values - start_time) * 1e-6
    # plot_angular_velocity(axes[4], ang_vel_sp_time, 
    #                       ang_vel_sp=ang_vel_sp, ang_vel=None)  # Replace with correct angular velocity data
       

 
    # df_ang_vel = read_csv_by_topic(directory, 'angular_velocity', start_time, end_time)
    # ang_vel = df_ang_vel[['xyz[0]', 'xyz[1]', 'xyz[2]']].values
    # ang_vel_time = (df_ang_vel['timestamp'].values - start_time) * 1e-6
    # plot_angular_velocity(axes[4], ang_vel_time, 
    #                       ang_vel_sp=None, ang_vel=ang_vel)  # Replace with correct angular velocity data
    

    # # Save figure
    # save_plot(fig, f"{directory}/traj-view.pdf")
    # plt.show()

# Execute pipeline
if __name__ == "__main__":
    main_plotting_pipeline()
