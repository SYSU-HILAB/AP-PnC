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


def analyze_solve_times(csv_file, column_name='solve_time_ms'):
    """
    Analyze solve times from a CSV file by calculating RMSE, MAX, and plotting distribution.

    Parameters:
    - csv_file: Path to the CSV file containing solve times
    - column_name: Name of the column containing solve times (default: 'solve_time_ms')
    """
    # Read data from CSV file
    try:
        df = pd.read_csv(csv_file)
        solve_times = df[column_name].values
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Calculate statistics
    rmse = np.sqrt(np.mean(np.square(solve_times)))
    max_time = np.max(solve_times)
    mean_time = np.mean(solve_times)
    median_time = np.median(solve_times)

    print(f"Number of samples: {len(solve_times)}")
    print(f"Mean solve time: {mean_time:.4f} ms")
    print(f"Median solve time: {median_time:.4f} ms")
    print(f"MAX solve time: {max_time:.4f} ms")
    print(f"RMSE of solve times: {rmse:.4f} ms")

    # Plot distribution
    plt.figure(figsize=(12, 6))

    # Histogram with KDE
    plt.subplot(1, 2, 1)

    solve_times_array = np.array(solve_times, dtype=np.float64).ravel()
    # sns.histplot(solve_times_array, kde=True, bins=50)
    # plt.title('Distribution of Solve Times')
    # plt.xlabel('Solve Time (ms)')
    # plt.ylabel('Frequency')

    # Boxplot
    plt.subplot(1, 2, 2)
    sns.boxplot(x=solve_times_array)
    plt.title('Boxplot of Solve Times')
    plt.xlabel('Solve Time (ms)')

    plt.tight_layout()
    plt.show()

    # Print percentile information
    percentiles = np.percentile(solve_times_array, [25, 50, 75, 90, 95, 99])
    print("\nPercentiles:")
    print(f"25th: {percentiles[0]:.4f} ms")
    print(f"50th (median): {percentiles[1]:.4f} ms")
    print(f"75th: {percentiles[2]:.4f} ms")
    print(f"90th: {percentiles[3]:.4f} ms")
    print(f"95th: {percentiles[4]:.4f} ms")
    print(f"99th: {percentiles[5]:.4f} ms")

# Example usage
if __name__ == "__main__":
    BaseDir = os.getenv('AP_PNC_DIR')
    # Replace 'solve_times.csv' with your actual CSV file path
    csv_file_path = os.path.join(BaseDir, 'record_nmpc.csv')
    analyze_solve_times(csv_file_path)
