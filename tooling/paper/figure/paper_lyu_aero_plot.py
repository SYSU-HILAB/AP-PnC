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

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import MultipleLocator

colors = dict(enumerate(plt.get_cmap("tab10").colors))

def read_aero_data(filename):
    """Read aerodynamic data from CSV file."""
    return pd.read_csv(filename)

def initialize_figure():
    """Initialize a multi-panel figure with a grid layout."""
    fig_height = 2.36
    letter_column_width = 3.5
    fig = plt.figure(figsize=(letter_column_width, fig_height), constrained_layout=False)
    gs = gridspec.GridSpec(2, 1, figure=fig)
    gs.update(wspace=0.3, hspace=0.25)

    ax1 = fig.add_subplot(gs[0, 0])  # Cx plot
    ax2 = fig.add_subplot(gs[1, 0])  # Cz plot

    # Add more padding at the bottom for x-axis labels
    plt.subplots_adjust(bottom=0.15)

    return fig, [ax1, ax2]

def plot_aero_coefficients(data_files):
    """Plot aerodynamic coefficients with publication quality."""
    # Get baseline data for scaling
    # Initialize figure
    fig, axes = initialize_figure()

    data = read_aero_data(data_files['ZhangLyu'])
    cz_30_baseline = data[data['alpha_deg'] == 30]['cz'].values[0]
    print(f"Cz at alpha = 30 degrees: cz = {cz_30_baseline}")

    for ax in axes:
      ax.minorticks_on()
      # Make ticks point inward
      ax.tick_params(which='both', direction='in')
      ax.tick_params(which='both', top=True, right=True)
      # Adjust tick parameters for closer spacing
      ax.tick_params(axis='x', pad=2)  # Reduced from 8 to 2
      ax.tick_params(axis='y', pad=2)  # Also adjust y-axis for consistency
    with plt.style.context("default"):

        for _i, (name, file) in enumerate(data_files.items()):
            data = read_aero_data(file)
            cz_30 = data[data['alpha_deg'] == 30]['cz'].values[0]
            scale_factor = cz_30_baseline / cz_30
            print(f"Scale factor for {name}: {scale_factor}")

            # Convert to numpy arrays
            alpha = data['alpha_deg'].to_numpy()
            cx = data['cx'].to_numpy() * scale_factor
            cz = data['cz'].to_numpy() * scale_factor

            # Get color for this dataset
            color = colors['blue']

            # Plot Cx
            axes[0].plot(alpha, cx, label=name, color=color)
            axes[0].set_ylabel(r'$\prescript{\mathcal{B}}{}{\boldsymbol{c}_x}~\mathrm{[kg/m]}$')
            axes[0].grid(True)


            # set ticks points for first plot (cx)
            axes[0].set_yticks([-0.04, -0.02, 0, 0.02, 0.04])
            # Set minor ticks for y-axis (4 between each major tick)
            major_spacing = 0.02  # Distance between major ticks
            minor_spacing = major_spacing/5  # 4 minor ticks between majors
            axes[0].yaxis.set_minor_locator(MultipleLocator(minor_spacing))

            # Plot Cz
            if name == 'ZhangLyu':
                axes[1].plot(alpha, cz, label="Lyu et al.", color=color)

            axes[1].set_xlabel(r'$\alpha~[\mathrm{deg}]$', labelpad=4)
            axes[1].grid(True)

            axes[1].set_ylabel(r'$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}~\mathrm{[kg/m]}$')

            # set ticks points for second plot (cz)
            axes[1].set_yticks([-0.3, -0.15, 0, 0.15, 0.3])
            # Set minor ticks for y-axis (4 between each major tick)
            major_spacing = 0.15  # Distance between major ticks



            minor_spacing = major_spacing/5  # 4 minor ticks between majors
            axes[1].yaxis.set_minor_locator(MultipleLocator(minor_spacing))
        # # Set legend with specified line length and reduced text spacing
        # axes[1].legend(handlelength=1.2, handletextpad=0.2, ncol=2, columnspacing=0.4,
        #               bbox_to_anchor=(1.0, 0.95), loc='upper right')

        plt.subplots_adjust(left=0.2)

        directory = None
        directory = directory or os.getenv('AP_PNC_DIR')

        fig.savefig(f"{directory}/scripts/paper_plots_figures/aero-lyu.pdf", backend='pgf')
        # Set scientific style for both axes


def save_plot(fig, save_path):
    """Save the figure to a specified path."""
    fig.savefig(save_path, format='pdf', bbox_inches='tight', dpi=300)

def main_plotting_pipeline(directory=None):
    """Main function to process data and generate plots."""
    directory = directory or os.getenv('AP_PNC_DIR')

    if not directory:
        raise ValueError("AP_PNC_DIR environment variable not set")

    # Define the data files
    data_files = {
        'ZhangLyu': os.path.join(directory, 'zZhangLyuAero.csv'),
    }



    # Plot content
    plot_aero_coefficients(data_files)




if __name__ == "__main__":
    main_plotting_pipeline()
