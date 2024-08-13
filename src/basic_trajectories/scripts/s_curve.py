import numpy as np
import matplotlib.pyplot as plt

# Parameters for the sine wave
A = 3          # Amplitude
lambda_ = 20   # Wavelength
# phi = np.pi/4  # Phase shift (in radians)

# Generate x values
x = np.linspace(-20, 20, 500)

# Calculate y values
y = A * np.sin(2 * np.pi / lambda_ * x )

# Plotting the sine wave
plt.figure(figsize=(10, 6))
plt.plot(x, y, label=f'Sine wave: $A\\sin(\\frac{{2\\pi}}{{\\lambda}} x + \\phi)$')

# Adding labels and title
plt.title('Sine Wave with Phase Shift')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
plt.xlim(-5, 5)
plt.ylim(-3, 3)
# Show phase shift and amplitude
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.legend()

# Show the plot
plt.show()
