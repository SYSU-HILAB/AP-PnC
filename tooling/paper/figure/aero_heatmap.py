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

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from matplotlib.colors import LinearSegmentedColormap


def set_plot_styles():
    """Set global matplotlib styles for publication quality plots."""
    plt.rcParams.update({
        'font.size': 8,
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'legend.fontsize': 8,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'axes.labelpad': 0,
        'figure.dpi': 300,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'figure.facecolor': 'none',
        'axes.facecolor': 'none',
        'axes.edgecolor': 'black',
    })

def create_nonlinear_blues():
    """Method 2: Create a nonlinear Blues colormap (fast change in low values, slow in high values)."""
    blues = plt.cm.Blues
    # Create nonlinear spacing using power law (adjust power value to control nonlinearity)
    power = 0.3  # Values < 1 give desired effect (fast at low, slow at high)
    nonlinear_space = np.power(np.linspace(0.1, 1, 256), power)
    colors = blues(nonlinear_space)
    colors = np.flipud(colors)
    custom_blues = LinearSegmentedColormap.from_list('non-linear-blues', colors)
    plt.cm.register_cmap(name='non-linear-blues', cmap=custom_blues)
    return custom_blues

def create_heatmap():
    # Read the data
    directory = os.getenv('AP_PNC_DIR')
    if not directory:
        raise ValueError("AP_PNC_DIR environment variable not set")

    df = pd.read_csv(os.path.join(directory, 'src',
                                  'aerodynamics', 'data', 'aero_diff_tracking.csv'))

    # Convert data to matrix format
    matrix_data = df.set_index('Type').values

    # Create figure with specific size
    fig, ax = plt.subplots(figsize=(5, 3))

    # Create velocity labels
    velocity_labels = [f'{v} m/s' for v in range(4, 15, 2)]

    # Create heatmap
    hm = sns.heatmap(matrix_data,
                annot=True,  # Show values in cells
                fmt='.2f',   # Format for the values (2 decimal places)
                cmap=create_nonlinear_blues(),  # Use custom reversed Blues colormap
                xticklabels=velocity_labels,  # Custom velocity labels
                yticklabels=df['Type'],
                cbar_kws={'label': 'RMSE [m]'},
                ax=ax)

    # Explicitly remove tick marks while keeping labels
    ax.tick_params(axis='both', length=0)

    # Remove colorbar tick marks but keep labels and adjust label padding
    colorbar = hm.collections[0].colorbar
    colorbar.ax.tick_params(length=0)
    colorbar.ax.set_ylabel('RMSE [m]', labelpad=5)

    # Rotate x-axis labels for better readability
    plt.xticks(rotation=0)
    plt.yticks(rotation=0)

    # Adjust layout
    plt.tight_layout()

    # Save the plot using the same directory handling as paper_exp_plot.py
    save_path = os.path.join(directory, 'paper_plots', 'aero-heatmap.pdf')
    plt.savefig(save_path, format='pdf', bbox_inches='tight', dpi=300)
    plt.show()

if __name__ == "__main__":
    set_plot_styles()
    create_heatmap()
