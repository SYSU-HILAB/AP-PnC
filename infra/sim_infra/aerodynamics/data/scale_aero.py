#!/usr/bin/env python3

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

import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

def read_aero_data(filename):
    """Read aerodynamic data from CSV file."""
    return pd.read_csv(filename)

plt.rcParams['axes.prop_cycle'] = (
    plt.cycler('color', ["#0d49fb", "#e6091c", "#26eb47", "#8936df", "#fec32d", "#25d7fd"])
)

def plot_aero_coefficients(data_files):
    """Plot Cx and Cz vs alpha for multiple data files."""
    plt.figure(figsize=(12, 6))

    # Create two subplots side by side
    ax1 = plt.subplot(121)  # For Cx
    ax2 = plt.subplot(122)  # For Cz

    # Colors for different datasets
    data = read_aero_data(data_files['ZhangLyu'])
    cz_60_baseline = data[data['alpha_deg'] == 60]['cz'].values[0]
    print(f"Cz at alpha = 60 degrees: cz = {cz_60_baseline}")
    for i, (name, file) in enumerate(data_files.items()):
        data = read_aero_data(file)

        cz_60 = data[data['alpha_deg'] == 60]['cz'].values[0]
        scale_factor = cz_60_baseline / cz_60
        print(f"Scale factor for {name}: {scale_factor}")
        # Convert pandas Series to numpy arrays
        alpha = data['alpha_deg'].to_numpy()
        cx = data['cx'].to_numpy() * scale_factor
        cz = data['cz'].to_numpy() * scale_factor

        # Plot Cx
        ax1.plot(alpha, cx, label=name)
        ax1.set_xlabel('Alpha (degrees)')
        ax1.set_ylabel('cx')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title('cx vs Alpha')

        # Plot Cz
        ax2.plot(alpha, cz, label=name)
        ax2.set_xlabel('Alpha (degrees)')
        ax2.set_ylabel('cz')
        ax2.grid(True)
        ax2.legend()
        ax2.set_title('cz vs Alpha')

    plt.tight_layout()
    plt.show()

def main():
    # Define the data files
    DIR = os.environ['AP_PNC_DIR']
    data_files = {
        'Advanced': os.path.join(DIR, 'zAdvancedLDAero.csv'),
        'BSpline': os.path.join(DIR, 'zBSplineAero.csv'),
        # 'NonAero': os.path.join(DIR, 'zNonAero.csv'),
        'Phi': os.path.join(DIR, 'zPhiAero.csv'),
        'ZhangLyu': os.path.join(DIR, 'zZhangLyuAero.csv')
    }

    plot_aero_coefficients(data_files)

if __name__ == '__main__':
    main()
