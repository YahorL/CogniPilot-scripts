#!/usr/bin/env python3
"""
Script to plot IMU and magnetometer data from CSV files.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_imu_magnetometer_data(imu_file: str, mag_file: str) -> None:
    """Plot IMU and magnetometer data from CSV files.
    
    Args:
        imu_file: Path to IMU data CSV file
        mag_file: Path to magnetometer data CSV file
    """
    # Load data
    print(f"Loading IMU data from: {imu_file}")
    imu_df = pd.read_csv(imu_file)
    
    print(f"Loading magnetometer data from: {mag_file}")
    mag_df = pd.read_csv(mag_file)
    
    # Extract data into arrays
    imu_timestamps = imu_df['timestamp'].values
    gyro_x = imu_df['gyro_x'].values
    gyro_y = imu_df['gyro_y'].values
    gyro_z = imu_df['gyro_z'].values
    accel_x = imu_df['accel_x'].values
    accel_y = imu_df['accel_y'].values
    accel_z = imu_df['accel_z'].values
    
    mag_timestamps = mag_df['timestamp'].values
    mag_x = mag_df['mag_x'].values
    mag_y = mag_df['mag_y'].values
    mag_z = mag_df['mag_z'].values
    
    # Plot acceleration data
    plt.figure(figsize=(12, 8))
    plt.plot(imu_timestamps, accel_x, label='X', linewidth=1.5)
    plt.plot(imu_timestamps, accel_y, label='Y', linewidth=1.5)
    plt.plot(imu_timestamps, accel_z, label='Z', linewidth=1.5)
    plt.title('Linear Acceleration (Accelerometer)')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Plot gyroscope data
    plt.figure(figsize=(12, 8))
    plt.plot(imu_timestamps, gyro_x, label='X', linewidth=1.5)
    plt.plot(imu_timestamps, gyro_y, label='Y', linewidth=1.5)
    plt.plot(imu_timestamps, gyro_z, label='Z', linewidth=1.5)
    plt.title('Angular Velocity (Gyroscope)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Plot magnetometer data
    plt.figure(figsize=(12, 8))
    plt.plot(mag_timestamps, mag_x, label='X', linewidth=1.5)
    plt.plot(mag_timestamps, mag_y, label='Y', linewidth=1.5)
    plt.plot(mag_timestamps, mag_z, label='Z', linewidth=1.5)
    plt.title('Magnetic Field (Magnetometer)')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnetic Field (Gauss)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Print basic statistics
    print(f"\nData Statistics:")
    print(f"IMU duration: {imu_timestamps[-1] - imu_timestamps[0]:.2f} seconds")
    print(f"Magnetometer duration: {mag_timestamps[-1] - mag_timestamps[0]:.2f} seconds")
    print(f"IMU data points: {len(imu_timestamps)}")
    print(f"Magnetometer data points: {len(mag_timestamps)}")

if __name__ == "__main__":
    # Example usage
    imu_file = "output/output_data_long_stnr/imu_data.csv"
    mag_file = "output/output_data_long_stnr/magnetometer_data.csv"
    
    plot_imu_magnetometer_data(imu_file, mag_file)
    plt.show() 