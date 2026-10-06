#!/usr/bin/env python3

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

import argparse
import os

import numpy as np
import pandas as pd

from tooling.env import project_path

from .specify_figure_dirs import root_dir

RequiredTopics = [
    'tracking_info',
    'vehicle_odometry',  # 'odometry' is typically 'vehicle_odometry' in PX4 logs
    'vehicle_rates_setpoint',
    'vehicle_angular_velocity'
]

ulg_file = os.path.join(root_dir(), '.artifacts/benchmark/ulgs/log_3_UnknownDate.ulg')


class UlgDataLoader:
    """
    Class to load and process ULG files containing specific topics.
    """
    def __init__(self, file_path=None):
        """
        Initialize the ULG data loader.

        Args:
            file_path (str, optional): Path to a specific ULG file. If None, the latest ULG file will be loaded.
        """
        self._required_topics = RequiredTopics

        if file_path:
            self._ulg_path = str(project_path(file_path))
        else:
            self._ulg_path = self.__find_latest_ulg()

        print(f"Loading ULG file: {self._ulg_path}")
        self._ulog_data = None
        self.__dataframes = {}
        self.__time_start = None
        self.__time_end_v1 = None
        self.__time_end_v2 = None
    def __find_latest_ulg(self):
        """
        Find the latest ULG file in the directory structure.

        Returns:
            str: Path to the latest ULG file.
        """
        ulg_dir = os.path.join(root_dir(), '.artifacts/benchmark/ulgs')
        ulg_files = []

        # Recursively find all .ulg files
        for root, _, files in os.walk(ulg_dir):
            for file in files:
                if file.endswith(".ulg"):
                    ulg_files.append(os.path.join(root, file))

        if not ulg_files:
            raise FileNotFoundError("No ULG files found in the directory structure.")

        # Sort by modification time (newest first)
        ulg_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)

        return ulg_files[0]

    def load_data(self):
        """
        Load the ULG file and extract the required topics.

        Returns:
            dict: Dictionary containing DataFrames for each required topic.
        """
        try:
            import pyulog  # optional, only needed to read .ulg logs

            self._ulog_data = pyulog.ULog(self._ulg_path)

            # Check if all required topics are available
            available_topics = [d.name for d in self._ulog_data.data_list]
            missing_topics = [topic for topic in self._required_topics if topic not in available_topics]

            if missing_topics:
                raise ValueError(f"Warning: The following required topics are missing: {missing_topics}")


            self.time_range_specification()
            # Extract data for each available required topic
            for topic in self._required_topics:
                if topic in available_topics:
                    data = self._ulog_data.get_dataset(topic)
                    self.__dataframes[topic] = self.__convert_to_dataframe(data)

            return self.__dataframes

        except Exception as e:
            print(f"Error loading ULG file: {e}")
            return None

    def time_range_specification(self):
        """
        Get the time range specification.
        """
        tracking_data = self._ulog_data.get_dataset('tracking_info')
        #locate the first timestamp that  ref_vel_norm < 0.1
        ref_vel_norm = np.sqrt(tracking_data.data['ref_velocity[0]']**2 + tracking_data.data['ref_velocity[1]']**2 + tracking_data.data['ref_velocity[2]']**2)

        # We assume that

        start_index = np.where((ref_vel_norm > 0.1) & (ref_vel_norm < 3))[0][0]
        print(f"start index is {start_index}")

        self.__time_start = tracking_data.data['timestamp'][start_index]
        print(f"start index is {start_index}")


        self.__time_end_v1 = (tracking_data.data['timestamp'][-1] - self.__time_start)/1e6

        # get the index of the second row that ref_position[0] == ref_position[0][0]
        ref_position_0 = tracking_data.data['ref_position[0]']
        ref_position_1 = tracking_data.data['ref_position[1]']
        ref_position_2 = tracking_data.data['ref_position[2]']

        end_index = np.where(ref_vel_norm < 0.1)
        print(f"end_index: {end_index}")
        a_cirle_completed_index = -1
        if end_index[0][0] > start_index + 100:
            a_cirle_completed_index = end_index[0][0]
        else :
            a_cirle_completed_index = np.where((ref_position_0 == ref_position_0[start_index]) & (ref_position_1 == ref_position_1[start_index]) & (ref_position_2 == ref_position_2[start_index]))[0][1]

        print(f"a_cirlce_completed_index end is {a_cirle_completed_index}")
        # a_cirle_completed_index = np.where((ref_position_0 == ref_position_0[start_index]) & (ref_position_1 == ref_position_1[start_index]) & (ref_position_2 == ref_position_2[start_index]))[0][1]
        self.__time_end_v2 = (tracking_data.data['timestamp'][a_cirle_completed_index] - self.__time_start)/1e6



    def __convert_to_dataframe(self, dataset):
        """
        Convert a ULog dataset to a pandas DataFrame.

        Args:
            dataset: ULog dataset object.

        Returns:
            pd.DataFrame: DataFrame containing the dataset data.
        """
        data_dict = {}
        # Convert timestamp to seconds
        timestamp = dataset.data['timestamp']
        data_dict['time_from_start'] = (timestamp - self.__time_start) / 1e6  # Convert to seconds from start

        # Add all other fields
        for field_name in dataset.data.keys():
            if not (field_name == 'timestamp'):
                data_dict[field_name] = dataset.data[field_name]

        df = pd.DataFrame(data_dict)

        # time_from_start must greater than 0

        # convert all data to numeric
        df = df.loc[lambda df: (df['time_from_start'] >=  0) & (df['time_from_start'] <= self.__time_end_v2), :]
        df = df.apply(pd.to_numeric, errors='raise')

        return df

    def get_topic_data(self, topic_name):
        """
        Get data for a specific topic.

        Args:
            topic_name (str): Name of the topic.

        Returns:
            pd.DataFrame: DataFrame containing the topic data.
        """
        if not self.__dataframes:
            self.load_data()

        if topic_name in self.__dataframes:
            return self.__dataframes[topic_name]
        else:
            print(f"Topic '{topic_name}' not found in the loaded data.")
            return None

    def get_available_topics(self):
        """
        Get a list of available topics in the loaded ULG file.

        Returns:
            list: List of available topic names.
        """
        if not self._ulog_data:
            self.load_data()

        return [d.name for d in self._ulog_data.data_list]

    def interpolate_topics_to_common_timeline(self):
        """
        Interpolate all topics to align with the timeline of tracking_info.

        This method resamples all topics to have the same timestamps as tracking_info,
        using linear interpolation for numeric values.

        Returns:
            dict: Dictionary of interpolated DataFrames for all topics.
        """
        if not self.__dataframes or 'tracking_info' not in self.__dataframes:
            self.load_data()
        df = self.__dataframes
        # Get the reference timeline from tracking_info
        ref_timeline = self.__dataframes['tracking_info']['time_from_start'].values

        # Dictionary to store interpolated dataframes
        interpolated_dataframes = {}
        interpolated_dataframes['tracking_info'] = self.__dataframes['tracking_info'].copy()

        # Process each topic except tracking_info
        for topic, df in self.__dataframes.items():
            if topic == 'tracking_info':
                continue

            print(f"Interpolating topic: {topic} to match tracking_info timeline")

            # Create a new dataframe with the reference timeline
            interp_df = pd.DataFrame({'time_from_start': ref_timeline})

            # Get the original timeline
            orig_timeline = df['time_from_start'].values

            # Interpolate each column except time_from_start
            for col in df.columns:
                if col == 'time_from_start':
                    continue

                # Use pandas interpolation
                interp_df[col] = np.interp(
                    ref_timeline,
                    orig_timeline,
                    df[col].values,
                    left=np.nan,
                    right=np.nan
                )

            # Drop rows with NaN values (outside the original time range)
            interp_df = interp_df.dropna()

            # Store the interpolated dataframe
            interpolated_dataframes[topic] = interp_df

            print(f"  Original shape: {df.shape}, Interpolated shape: {interp_df.shape}")

        # Update the dataframes in the class
        self.__dataframes = interpolated_dataframes

        print("Successfully interpolated all topics to common timeline.")
        return self.__dataframes


def main():
    """
    Main function to parse arguments and load ULG data.
    """
    parser = argparse.ArgumentParser(description='Load and process ULG files.')
    parser.add_argument('--file', '-f', type=str, help='Path to a specific ULG file to load.')
    parser.add_argument('--shift-zero-vel', '-s', action='store_true',
                        help='Shift zero-velocity data from the end to the beginning.')
    parser.add_argument('--vel-threshold', '-t', type=float, default=0.1,
                        help='Velocity threshold for zero-velocity detection (default: 0.1 m/s).')
    args = parser.parse_args()

    # Create ULG data loader
    loader = UlgDataLoader(ulg_file)

    # Load data
    data = loader.load_data()

    if data:
        print("Successfully loaded ULG data.")
        print(f"Available topics: {loader.get_available_topics()}")

        # Print a summary of the loaded data
        for topic, df in data.items():
            print(f"\nTopic: {topic}")
            print(f"  Shape: {df.shape}")
            print(f"  Time range: {df['time_from_start'].min():.2f}s to {df['time_from_start'].max():.2f}s")
            print(f"  Columns: {', '.join(df.columns)}")

        # Apply zero-velocity data shifting if requested
        if args.shift_zero_vel and 'tracking_info' in data:
            print("\nShifting zero-velocity data...")
            shifted_df = loader.shift_zero_velocity_data(velocity_threshold=args.vel_threshold)

            # Print summary of the shifted data
            print("\nShifted tracking_info data:")
            print(f"  Shape: {shifted_df['tracking_info'].shape}")
            print(f"  Time range: {shifted_df['tracking_info']['time_from_start'].min():.2f}s to {shifted_df['tracking_info']['time_from_start'].max():.2f}s")

            # Calculate velocity magnitude for visualization
            vel_cols = ['actual_velocity[0]', 'actual_velocity[1]', 'actual_velocity[2]']
            vel_mag = np.sqrt(
                shifted_df['tracking_info'][vel_cols[0]]**2 + shifted_df['tracking_info'][vel_cols[1]]**2 + shifted_df['tracking_info'][vel_cols[2]]**2
            )

            print(f"  Velocity range: {vel_mag.min():.2f} to {vel_mag.max():.2f} m/s")
            print(f"  Zero-velocity samples: {(vel_mag < args.vel_threshold).sum()} out of {len(vel_mag)}")
    else:
        print("Failed to load ULG data.")

if __name__ == "__main__":
    main()
