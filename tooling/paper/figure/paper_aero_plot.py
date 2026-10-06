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

from tooling.env import artifact_path, project_path

from .specify_figure_dirs import figure_path

colors = dict(enumerate(plt.get_cmap("tab10").colors))

def read_aero_data(filename):
    """Read aerodynamic data from CSV file."""
    return pd.read_csv(project_path(filename))

def initialize_figure():
    """Initialize a multi-panel figure with a grid layout."""
    _letter_width, _a4_height = 7.0, 9.5
    letter_column_width = 3.5
    fig = plt.figure(figsize=(letter_column_width, 2.3622), constrained_layout=False)
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
        for i, (name, file) in enumerate(data_files.items()):
            data = read_aero_data(file)
            cz_30 = data[data['alpha_deg'] == 30]['cz'].values[0]
            scale_factor = cz_30_baseline / cz_30
            print(f"Scale factor for {name}: {scale_factor}")

        # Convert to numpy arrays
            alpha = data['alpha_deg'].to_numpy()
            cx = data['cx'].to_numpy() * scale_factor
            cz = data['cz'].to_numpy() * scale_factor

            # Get color for this dataset
            color = list(colors.values())[i]

            # Plot Cx
            axes[0].plot(alpha, cx, label=name, color=color)
            axes[0].set_ylabel(r'$\prescript{\mathcal{B}}{}{\boldsymbol{c}_x}~\mathrm{[kg/m]}$', \
                               labelpad=-2)
            axes[0].grid(True)

            # set ticks points for first plot (cx)
            axes[0].set_yticks([-0.08, -0.04, 0, 0.04, 0.08])
            # Set minor ticks for y-axis (4 between each major tick)
            major_spacing = 0.04  # Distance between major ticks
            minor_spacing = major_spacing/5  # 4 minor ticks between majors
            axes[0].yaxis.set_minor_locator(MultipleLocator(minor_spacing))

            # Plot Cz
            if name == 'ZhangLyu':
                axes[1].plot(alpha, cz, label=r"Lyu et al.", color=color)
            elif name == 'BSpline':
                axes[1].plot(alpha, cz, label=r"Ma et al.", color=color)
            elif name == 'Phi':
                axes[1].plot(alpha, cz, label=r"$\phi$-theory", color=color)
            else:
                axes[1].plot(alpha, cz, label="Gazebo", color=color)

            axes[1].set_xlabel(r'$\alpha~[\mathrm{deg}]$', labelpad=4)
            axes[1].set_ylabel(r'$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}~\mathrm{[kg/m]}$',\
                               labelpad=-2)
            axes[1].grid(True)

            # set ticks points for second plot (cz)
            axes[1].set_yticks([-0.4, -0.2, 0, 0.2, 0.4])
            # Set minor ticks for y-axis (4 between each major tick)
            major_spacing = 0.2  # Distance between major ticks



            minor_spacing = major_spacing/5  # 4 minor ticks between majors
            axes[1].yaxis.set_minor_locator(MultipleLocator(minor_spacing))
            # # Set legend with specified line length and reduced text spacing
            axes[1].legend(handlelength=1.2, handletextpad=0.2, ncol=4, columnspacing=0.4,
                        bbox_to_anchor=(0.5, 2.3), loc='lower center')

        plt.subplots_adjust(left=0.2)
        fig.savefig(figure_path("aero-comparison.pdf"), backend='pgf')
        # Set scientific style for both axes




def main_plotting_pipeline(directory=None):

    # Define the data files

    directory = project_path(directory) if directory is not None else artifact_path(".artifacts/aerodynamics")

    data_files = {
        'BSpline': os.path.join(directory, 'zBSplineAero.csv'),
        'ZhangLyu': os.path.join(directory, 'zZhangLyuAero.csv'),
        'Phi': os.path.join(directory, 'zPhiAero.csv'),
        'Advanced': os.path.join(directory, 'zAdvancedLDAero.csv'),
    }

    # Plot content
    plot_aero_coefficients(data_files)




if __name__ == "__main__":
    main_plotting_pipeline()
