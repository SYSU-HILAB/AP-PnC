import numpy as np
import matplotlib.pyplot as plt

def cartesian_to_polar(x, y, z):
    """Converts Cartesian coordinates to azimuthal and polar angles.  Handles scalar and array inputs."""
    try:
        r = np.sqrt(x**2 + y**2 + z**2)
        if np.all(r == 0): #Check if all values in r are zero (handles both scalar and array cases)
            return None, None
        theta = np.arccos(z / r)
        phi = np.arctan2(y, x)
        return phi, theta
    except TypeError:  #Handles cases where x, y, z are not numpy arrays or numbers
        print("Error: Input coordinates must be numbers or numpy arrays.")
        return None, None
    except ZeroDivisionError:
        print("Error: Division by zero encountered. Check your input coordinates.")
        return None, None


# Example circle path points (replace with your actual data)  This is corrected to create arrays.
radius = 1
num_points = 100
phi_angles = np.linspace(0, 2 * np.pi, num_points)
theta = np.pi / 2

x = radius * np.sin(theta) * np.cos(phi_angles)
y = radius * np.sin(theta) * np.sin(phi_angles)
z = radius * np.cos(theta) * np.ones(num_points) #Ensure z is an array of the same length


# Convert to polar coordinates
phi_converted = []
theta_converted = []
for i in range(len(x)):
    phi, theta_val = cartesian_to_polar(x[i], y[i], z[i])
    if phi is not None and theta_val is not None:
        phi_converted.append(phi)
        theta_converted.append(theta_val)

#Visualization (same as before)
plt.figure(figsize=(10, 5))

# Plot Azimuthal Angle (phi)
plt.subplot(1, 2, 1)
plt.plot(phi_angles, phi_converted)
plt.xlabel("Original Azimuth Angle (phi)")
plt.ylabel("Converted Azimuth Angle (phi)")
plt.title("Azimuthal Angle (phi)")

# Plot Polar Angle (theta)
plt.subplot(1, 2, 2)
plt.plot(phi_angles, theta_converted)
plt.xlabel("Original Azimuth Angle (phi)")
plt.ylabel("Polar Angle (theta)")
plt.title("Polar Angle (theta)")

plt.tight_layout()
plt.show()