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

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


def create_custom_blues_1():
    """Method 1: Create a reversed Blues colormap."""
    blues = plt.cm.Blues
    colors = blues(np.linspace(0, 1, 256))
    colors = np.flipud(colors)  # Reverse the colors
    custom_blues = LinearSegmentedColormap.from_list('CustomBlues1', colors)
    plt.cm.register_cmap(name='CustomBlues1', cmap=custom_blues)
    return custom_blues

def create_custom_blues_2():
    """Method 2: Create a nonlinear Blues colormap (fast change in low values, slow in high values)."""
    blues = plt.cm.Blues
    # Create nonlinear spacing using power law (adjust power value to control nonlinearity)
    power = 0.5  # Values < 1 give desired effect (fast at low, slow at high)
    nonlinear_space = np.power(np.linspace(.1, 1, 256), power)
    colors = blues(nonlinear_space)
    colors = np.flipud(colors)
    custom_blues = LinearSegmentedColormap.from_list('CustomBlues2', colors)
    plt.cm.register_cmap(name='CustomBlues2', cmap=custom_blues)
    return custom_blues

def demonstrate_colormaps():
    """Show examples of all custom colormaps."""
    # Create data for demonstration
    data = np.linspace(0, 1, 256).reshape(1, -1)
    data = np.vstack((data, data))

    # Create and register all colormaps
    cmaps = [
        create_custom_blues_1(),
        create_custom_blues_2()
    ]

    # Plot all colormaps
    fig, axes = plt.subplots(len(cmaps), 1, figsize=(8, 4))
    fig.suptitle('Custom Blues Colormap Variants')

    for ax, cmap in zip(axes, cmaps, strict=False):
        im = ax.imshow(data, aspect='auto', cmap=cmap)
        plt.colorbar(im, ax=ax, orientation='horizontal')
        ax.set_title(cmap.name)
        ax.set_xticks([])
        ax.set_yticks([])

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    demonstrate_colormaps()

# Example usage in other scripts:
"""
import matplotlib.pyplot as plt
from .custom_colormaps import create_custom_blues_1

# Register the custom colormap
create_custom_blues_1()

# Now you can use it by name
plt.imshow(data, cmap='CustomBlues1')
# or
plt.imshow(data, cmap=plt.cm.get_cmap('CustomBlues1'))
"""
