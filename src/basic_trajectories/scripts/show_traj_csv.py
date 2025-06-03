import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_trajectory(csv_file):
    # Read CSV
    df = pd.read_csv(csv_file)
    
    # Create figure
    fig = plt.figure(figsize=(15, 5))
    
    # 3D Trajectory Plot
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(df['x'], df['y'], df['z'])
    ax1.set_title('3D Trajectory')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Velocity Magnitude Plot
    ax2 = fig.add_subplot(122)
    velocity_mag = (df['vx']**2 + df['vy']**2 + df['vz']**2)**0.5
  
    ax2.plot(df['time'], velocity_mag)
    ax2.set_title('Velocity Magnitude')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Velocity')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
  visualize_trajectory("vertical_loop_trajectory.csv")