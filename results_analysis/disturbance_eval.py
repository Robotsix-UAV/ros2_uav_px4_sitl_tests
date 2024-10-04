import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def load_data(directory):
    # Define file paths
    acceleration_path = os.path.join(directory, 'disturbance_observer_acceleration.csv')
    velocity_path = os.path.join(directory, 'disturbance_observer_velocity.csv')
    coefficients_path = os.path.join(directory, 'disturbance_observer_coefficients.csv')

    # Read the CSV files
    acceleration_df = pd.read_csv(acceleration_path)
    velocity_df = pd.read_csv(velocity_path)
    coefficients_df = pd.read_csv(coefficients_path)

    return acceleration_df, velocity_df, coefficients_df

def compute_obtained_acceleration(velocity, timestamp):
    # Compute the time differences in seconds
    time_diff = np.diff(timestamp) / 1e9  # Convert nanoseconds to seconds

    # Compute acceleration as the derivative of velocity
    obtained_acceleration = np.diff(velocity) / time_diff

    # Pad the result to maintain the same length by inserting 0 at the beginning
    return np.insert(obtained_acceleration, 0, 0)

def compute_disturbance(velocity_df, acceleration_df, coefficients_df):
    # Merge dataframes based on the timestamp
    merged_df = pd.merge_asof(velocity_df.sort_values('timestamp'), acceleration_df.sort_values('timestamp'), on='timestamp')
    merged_df = pd.merge_asof(merged_df.sort_values('timestamp'), coefficients_df.sort_values('timestamp'), on='timestamp')

    # Sort data by timestamp
    merged_df = merged_df.sort_values('timestamp')

    # Compute the obtained acceleration using the actual time differences
    merged_df['obtained_acceleration_x'] = compute_obtained_acceleration(merged_df['velocity_x'].values, merged_df['timestamp'].values)
    merged_df['obtained_acceleration_y'] = compute_obtained_acceleration(merged_df['velocity_y'].values, merged_df['timestamp'].values)
    merged_df['obtained_acceleration_z'] = compute_obtained_acceleration(merged_df['velocity_z'].values, merged_df['timestamp'].values)

    # Compute the disturbance for each axis
    merged_df['disturbance_x'] = merged_df['disturbance_coefficients_constant_x'] + merged_df['velocity_x'] * merged_df['disturbance_coefficients_proportional_x']
    merged_df['disturbance_y'] = merged_df['disturbance_coefficients_constant_y'] + merged_df['velocity_y'] * merged_df['disturbance_coefficients_proportional_y']
    merged_df['disturbance_z'] = merged_df['disturbance_coefficients_constant_z'] + merged_df['velocity_z'] * merged_df['disturbance_coefficients_proportional_z']

    # Limit timestamps to the disturbance coefficients timestamps
    merged_df = merged_df[merged_df['timestamp'] >= coefficients_df['timestamp'].min()]
    merged_df = merged_df[merged_df['timestamp'] <= coefficients_df['timestamp'].max()]

    # Compute the quality indicator as the difference between the obtained acceleration and the expected + disturbance
    merged_df['quality_x'] = (merged_df['obtained_acceleration_x'] - (merged_df['acceleration_x'] + merged_df['disturbance_x'])) ** 2
    merged_df['quality_y'] = (merged_df['obtained_acceleration_y'] - (merged_df['acceleration_y'] + merged_df['disturbance_y'])) ** 2
    merged_df['quality_z'] = (merged_df['obtained_acceleration_z'] - (merged_df['acceleration_z'] + merged_df['disturbance_z'])) ** 2

    # Quality indicator as the mean squared error (MSE) for each axis
    quality_x = np.mean(merged_df['quality_x'])
    quality_y = np.mean(merged_df['quality_y'])
    quality_z = np.mean(merged_df['quality_z'])

    return merged_df, quality_x, quality_y, quality_z

def plot_comparison(merged_df, axis):
    plt.figure(figsize=(10,6))
    plt.plot(merged_df['timestamp'].values, merged_df[f'obtained_acceleration_{axis}'].values, label=f'Obtained Acceleration ({axis}-axis)')
    plt.plot(merged_df['timestamp'].values, merged_df[f'acceleration_{axis}'].values, label=f'Expected Acceleration ({axis}-axis)', linestyle=':')
    plt.plot(merged_df['timestamp'].values, (merged_df[f'acceleration_{axis}'] + merged_df[f'disturbance_{axis}']).values, label=f'Expected + Disturbance ({axis}-axis)', linestyle='--')
    plt.xlabel('Timestamp')
    plt.ylabel('Acceleration')
    plt.title(f'Comparison of Obtained, Real, and Expected + Disturbance Accelerations ({axis}-axis)')
    plt.legend()
    plt.show()

def main():
    # Argument parsing
    parser = argparse.ArgumentParser(description='Disturbance Computation from CSV Files.')
    parser.add_argument('directory', type=str, help='Directory containing the CSV files')
    parser.add_argument('--plot', action='store_true', help='Display the plot for the comparison')
    args = parser.parse_args()

    # Load data
    acceleration_df, velocity_df, coefficients_df = load_data(args.directory)

    # Compute disturbance and quality indicators
    merged_df, quality_x, quality_y, quality_z = compute_disturbance(velocity_df, acceleration_df, coefficients_df)

    # Print quality indicators
    print(f"Quality Indicator (X-axis): {quality_x}")
    print(f"Quality Indicator (Y-axis): {quality_y}")
    print(f"Quality Indicator (Z-axis): {quality_z}")

    # If --plot option is passed, display the comparison plot for each axis
    if args.plot:
        plot_comparison(merged_df, 'x')
        plot_comparison(merged_df, 'y')
        plot_comparison(merged_df, 'z')

if __name__ == '__main__':
    main()
