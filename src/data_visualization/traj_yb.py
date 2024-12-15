import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the environment variable and the CSV file
csv_file_path = os.path.join(os.getenv('TAILSITTER_PLANNING_DIR'), 'planning.csv')

df = pd.read_csv(csv_file_path, on_bad_lines='skip')

# Define the columns to be converted to numeric
columns_to_convert = ['time', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'thrust', 'alpha', 'aero_force_z', "yb_x", "yb_y", "yb_z"]

# Convert columns to numeric, handling errors with NaN
for column in columns_to_convert:
    df[column] = pd.to_numeric(df[column], errors='coerce')

# Drop any rows with NaN values in position or direction columns to avoid plotting errors
df.dropna(subset=['pos_x', 'pos_y', 'pos_z', 'yb_x', 'yb_y', 'yb_z', "av_x", "av_y", "av_z"], inplace=True)

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Remove grids, background, and axis labels
ax.grid(False)
ax.set_facecolor((1, 1, 1, 0))  # Transparent background
ax.xaxis.pane.set_visible(False)
ax.yaxis.pane.set_visible(False)
ax.zaxis.pane.set_visible(False)

def get_orthonormal_vectors(direction):
    # Normalize the direction vector
    direction = direction / np.linalg.norm(direction)
    # Find a vector that is not collinear with the direction
    if abs(direction[0]) < 1:
        arbitrary = np.array([1, 0, 0])
    else:
        arbitrary = np.array([0, 1, 0])
    # First orthogonal vector
    orthogonal1 = np.cross(direction, arbitrary)
    orthogonal1 /= np.linalg.norm(orthogonal1)
    # Second orthogonal vector
    orthogonal2 = np.cross(direction, orthogonal1)
    orthogonal2 /= np.linalg.norm(orthogonal2)
    return orthogonal1, orthogonal2

# Extract the position and direction components
positions = df[['pos_x', 'pos_y', 'pos_z']].values
directions = df[['yb_x', 'yb_y', 'yb_z']].values
avs = df[['av_x', 'av_y', 'av_z']].values

last = 100
positions = positions[0:last]
directions = directions[0:last]
avs = avs[0:last]
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color='b', linestyle='-', linewidth=1.5)

# Plot arrows and orthonormal planes at the start and end
for i, point, direction, av, color in zip(
    [0, -1], [positions[0], positions[-1]], [directions[0], directions[-1]],
    [avs[0], avs[-1]], ['g', 'r']
):
    # Plot direction arrow
    ax.quiver(
        point[0], point[1], point[2],
        direction[0], direction[1], direction[2],
        length=.5, color=color, linewidth=2, normalize=True, 
        arrow_length_ratio=0.0
    )
    ax.quiver(
        point[0], point[1], point[2],
        av[0], av[1], -av[2],
        length=.5, color='y', linewidth=2, normalize=True,
        arrow_length_ratio=0.0
    )
    # Generate the orthonormal vectors
    ortho1, ortho2 = get_orthonormal_vectors(direction)

    # Define a grid for the plane
    grid_size = 1.0
    u = np.linspace(-grid_size, grid_size, 10)
    v = np.linspace(-grid_size, grid_size, 10)
    u, v = np.meshgrid(u, v)

    # Calculate the plane points
    plane_x = point[0] + u * ortho1[0] + v * ortho2[0]
    plane_y = point[1] + u * ortho1[1] + v * ortho2[1]
    plane_z = point[2] + u * ortho1[2] + v * ortho2[2]

    # Plot the plane
    ax.plot_surface(plane_x, plane_y, plane_z, color=color, alpha=0.3, edgecolor='none')
ax.view_init(elev=0, azim=-70)
ax.grid(False)
ax.dist = 7.5
ax.set_facecolor((1, 1, 1, 0))  # 透明背景
ax.xaxis.pane.set_visible(False)
ax.yaxis.pane.set_visible(False)
ax.zaxis.pane.set_visible(False)
ax.xaxis.line.set_visible(False)  # 移除 X 轴线
ax.yaxis.line.set_visible(False)  # 移除 Y 轴线
ax.zaxis.line.set_visible(False)  # 移除 Z 轴线
ax.set_xticks([])  # 移除 X 轴刻度
ax.set_yticks([])  # 移除 Y 轴刻度
ax.set_zticks([])  # 移除 Z 轴刻度
plt.show()
