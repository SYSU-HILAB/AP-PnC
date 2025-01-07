import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import MultipleLocator

colors = {
    "red": "#B43734",
    "blue": "#00159D",
    "cyan": "#397E77",
    "yellow": "#E9CC49"
}

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
        'axes.prop_cycle': plt.cycler(color=[
            f'{colors["red"]}',
            f'{colors["blue"]}',
            f'{colors["cyan"]}',
            f'{colors["yellow"]}'
        ]),
        'figure.facecolor': 'none',
        'axes.facecolor': 'none',
        'grid.color': 'gray',
        'grid.alpha': 0.5,
        'axes.edgecolor': 'black',
        'axes.grid': True,
    })

def read_aero_data(filename):
    """Read aerodynamic data from CSV file."""
    return pd.read_csv(filename)

def initialize_figure():
    """Initialize a multi-panel figure with a grid layout."""
    fig_height = 3
    letter_column_width = 3.25
    fig = plt.figure(figsize=(letter_column_width, fig_height), constrained_layout=False)
    gs = gridspec.GridSpec(2, 1, figure=fig)
    gs.update(wspace=0.3, hspace=0.25)
    
    ax1 = fig.add_subplot(gs[0, 0])  # Cx plot
    ax2 = fig.add_subplot(gs[1, 0])  # Cz plot
    
    # Add more padding at the bottom for x-axis labels
    plt.subplots_adjust(bottom=0.15)
    
    return fig, [ax1, ax2]

def plot_aero_coefficients(data_files, axes):
    """Plot aerodynamic coefficients with publication quality."""
    # Get baseline data for scaling
    data = read_aero_data(data_files['ZhangLyu'])
    cz_30_baseline = data[data['alpha_deg'] == 30]['cz'].values[0]
    print(f"Cz at alpha = 30 degrees: cz = {cz_30_baseline}")
    for ax in axes:
      ax.minorticks_on()
      ax.grid(True, which='major', linestyle='--', alpha=0.7)
      ax.grid(True, which='minor', linestyle=':', alpha=0.4)
      # Make ticks point inward
      ax.tick_params(which='both', direction='in')
      ax.tick_params(which='both', top=True, right=True)
      # Adjust tick parameters for closer spacing
      ax.tick_params(axis='x', pad=2)  # Reduced from 8 to 2
      ax.tick_params(axis='y', pad=2)  # Also adjust y-axis for consistency
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
        color = colors['blue']
        
        # Plot Cx
        axes[0].plot(alpha, cx, label=name, color=color)
        axes[0].set_ylabel('$C_x$ [kg/m]')
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

        axes[1].set_xlabel('Angle of Attack [deg]', labelpad=4)
        axes[1].set_ylabel('$C_z$ [kg/m]')
        axes[1].grid(True)
        
        # set ticks points for second plot (cz)
        axes[1].set_yticks([-0.3, -0.15, 0, 0.15, 0.3])
        # Set minor ticks for y-axis (4 between each major tick)
        major_spacing = 0.15  # Distance between major ticks


                 
        minor_spacing = major_spacing/5  # 4 minor ticks between majors
        axes[1].yaxis.set_minor_locator(MultipleLocator(minor_spacing))   
        # # Set legend with specified line length and reduced text spacing
        # axes[1].legend(handlelength=1.2, handletextpad=0.2, ncol=2, columnspacing=0.4,
        #               bbox_to_anchor=(1.0, 0.95), loc='upper right')
        
        # Set scientific style for both axes


def save_plot(fig, save_path):
    """Save the figure to a specified path."""
    fig.savefig(save_path, format='pdf', bbox_inches='tight', dpi=300)

def main_plotting_pipeline(directory=None):
    """Main function to process data and generate plots."""
    set_plot_styles()
    directory = directory or os.getenv('TAILSITTER_PLANNING_DIR')
    
    if not directory:
        raise ValueError("TAILSITTER_PLANNING_DIR environment variable not set")
    
    # Define the data files
    data_files = {
        'ZhangLyu': os.path.join(directory, 'zZhangLyuAero.csv'),
    }
    
    # Initialize figure
    fig, axes = initialize_figure()
    
    # Plot content
    plot_aero_coefficients(data_files, axes)
    
    # Save figure
    save_plot(fig, f"{directory}/paper_plots/aero-lyu.pdf")
    plt.show()
    


if __name__ == "__main__":
    main_plotting_pipeline() 