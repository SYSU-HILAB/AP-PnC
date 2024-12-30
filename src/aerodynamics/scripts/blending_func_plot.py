import numpy as np
import matplotlib.pyplot as plt

# Convert 15 degrees to radians
alpha_stall_deg = 30
alpha_stall = np.deg2rad(alpha_stall_deg)  # Convert degrees to radians

# Define the blending function sigma
def blending_function(alpha, alpha_stall, M):
    term1 = np.exp(-M * (alpha - alpha_stall))
    term2 = np.exp(M * (alpha + alpha_stall))
    numerator = 1 + term1 + term2
    denominator = (1 + term1) * (1 + term2)
    return numerator / denominator

alpha_values = np.linspace(-np.pi / 2, np.pi / 2, 400)  # from -30 to 30 degrees in radians

# Plot the blending function for different values of M
plt.figure(figsize=(8, 6))
for M in [0, 2, 5, 10, 15, 20]:
    sigma_values = blending_function(alpha_values, alpha_stall, M)
    plt.plot(alpha_values, sigma_values, label=f'M = {M}')

# Customize the plot
plt.title("Blending Function value vs Angle of Attack ", fontsize=14)
plt.xlabel("Angle of Attack (rad)", fontsize=12)
plt.ylabel("value", fontsize=12)
plt.axvline(x=alpha_stall, color='r', linestyle='--', label="Stall Angle ")
plt.legend()
plt.grid(True)
plt.show()
