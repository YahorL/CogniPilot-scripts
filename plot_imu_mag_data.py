#!/usr/bin/env python3
"""
Script to plot IMU and magnetometer data from CSV files.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def filter_data_by_time(timestamps, data_arrays, start_time=None, end_time=None):
    """Filter data arrays by time range.
    
    Args:
        timestamps: Array of timestamps
        data_arrays: List of data arrays to filter
        start_time: Start time in seconds (optional)
        end_time: End time in seconds (optional)
    
    Returns:
        Filtered timestamps and data arrays
    """
    if start_time is None and end_time is None:
        return timestamps, data_arrays
    
    # Create mask for time filtering
    mask = np.ones(len(timestamps), dtype=bool)
    
    if start_time is not None:
        mask &= (timestamps >= start_time)
    
    if end_time is not None:
        mask &= (timestamps <= end_time)
    
    # Apply mask to timestamps and all data arrays
    filtered_timestamps = timestamps[mask]
    filtered_data_arrays = [data[mask] for data in data_arrays]
    
    return filtered_timestamps, filtered_data_arrays

def plot_imu_magnetometer_data(imu_file: str, mag_file: str, start_time: float = None, end_time: float = None) -> None:
    """Plot IMU and magnetometer data from CSV files.
    
    Args:
        imu_file: Path to IMU data CSV file
        mag_file: Path to magnetometer data CSV file
        start_time: Start time in seconds for statistics calculation (optional)
        end_time: End time in seconds for statistics calculation (optional)
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
    
    # Filter data for statistics if time range is specified
    if start_time is not None or end_time is not None:
        print(f"\nFiltering data for statistics:")
        if start_time is not None:
            print(f"Start time: {start_time:.2f} seconds")
        if end_time is not None:
            print(f"End time: {end_time:.2f} seconds")
        
        # Filter IMU data
        imu_stats_timestamps, [gyro_x_stats, gyro_y_stats, gyro_z_stats, 
                              accel_x_stats, accel_y_stats, accel_z_stats] = filter_data_by_time(
            imu_timestamps, [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z], 
            start_time, end_time
        )
        
        # Filter magnetometer data
        mag_stats_timestamps, [mag_x_stats, mag_y_stats, mag_z_stats] = filter_data_by_time(
            mag_timestamps, [mag_x, mag_y, mag_z], start_time, end_time
        )
        
        print(f"Data points for statistics: IMU={len(imu_stats_timestamps)}, Mag={len(mag_stats_timestamps)}")
    else:
        # Use all data for statistics
        gyro_x_stats, gyro_y_stats, gyro_z_stats = gyro_x, gyro_y, gyro_z
        accel_x_stats, accel_y_stats, accel_z_stats = accel_x, accel_y, accel_z
        mag_x_stats, mag_y_stats, mag_z_stats = mag_x, mag_y, mag_z
    
    # Compute statistics
    accel_dt = (imu_timestamps[-1] - imu_timestamps[0]) / len(imu_timestamps)
    accel_mean_x = np.mean(accel_x_stats)
    accel_mean_y = np.mean(accel_y_stats)
    accel_mean_z = np.mean(accel_z_stats)
    accel_std_x = np.std(accel_x_stats)
    accel_std_y = np.std(accel_y_stats)
    accel_std_z = np.std(accel_z_stats)
    accel_noise_x = np.sqrt(accel_std_x ** 2 * accel_dt)  # output noise density
    accel_noise_y = np.sqrt(accel_std_y ** 2 * accel_dt)  # output noise density
    accel_noise_z = np.sqrt(accel_std_z ** 2 * accel_dt)  # output noise density
    accel_covariance_x = np.var(accel_x_stats)
    accel_covariance_y = np.var(accel_y_stats)
    accel_covariance_z = np.var(accel_z_stats)
    
    gyro_dt = (imu_timestamps[-1] - imu_timestamps[0]) / len(imu_timestamps)
    gyro_mean_x = np.mean(gyro_x_stats)
    gyro_mean_y = np.mean(gyro_y_stats)
    gyro_mean_z = np.mean(gyro_z_stats)
    gyro_std_x = np.std(gyro_x_stats)
    gyro_std_y = np.std(gyro_y_stats)
    gyro_std_z = np.std(gyro_z_stats)
    gyro_noise_x = np.sqrt(gyro_std_x ** 2) / np.pi * 180  # output noise density
    gyro_noise_y = np.sqrt(gyro_std_y ** 2) / np.pi * 180  # output noise density
    gyro_noise_z = np.sqrt(gyro_std_z ** 2) / np.pi * 180  # output noise density
    gyro_covariance_x = np.var(gyro_x_stats)
    gyro_covariance_y = np.var(gyro_y_stats)
    gyro_covariance_z = np.var(gyro_z_stats)
    
    # Print statistics
    print(f"\nAccelerometer Statistics:")   
    print(f"X-axis - Mean: {accel_mean_x:.6f} m/s², Std Dev: {accel_std_x:.6f} m/s², Noise: {accel_noise_x:.8f} m/s^(3/2), Covariance: {accel_covariance_x:.8f} m²/s^4, dt: {accel_dt:.8f} s")
    print(f"Y-axis - Mean: {accel_mean_y:.6f} m/s², Std Dev: {accel_std_y:.6f} m/s², Noise: {accel_noise_y:.8f} m/s^(3/2), Covariance: {accel_covariance_y:.8f} m²/s^4, dt: {accel_dt:.8f} s")
    print(f"Z-axis - Mean: {accel_mean_z:.6f} m/s², Std Dev: {accel_std_z:.6f} m/s², Noise: {accel_noise_z:.8f} m/s^(3/2), Covariance: {accel_covariance_z:.8f} m²/s^4, dt: {accel_dt:.8f} s")
    
    print(f"\nGyroscope Statistics:")
    print(f"X-axis - Mean: {gyro_mean_x:.6f} rad/s, Std Dev: {gyro_std_x:.6f} rad/s, Noise: {gyro_noise_x:.8f} deg/s, Covariance: {gyro_covariance_x:.8f} rad²/s²")
    print(f"Y-axis - Mean: {gyro_mean_y:.6f} rad/s, Std Dev: {gyro_std_y:.6f} rad/s, Noise: {gyro_noise_y:.8f} deg/s, Covariance: {gyro_covariance_y:.8f} rad²/s²")
    print(f"Z-axis - Mean: {gyro_mean_z:.6f} rad/s, Std Dev: {gyro_std_z:.6f} rad/s, Noise: {gyro_noise_z:.8f} deg/s, Covariance: {gyro_covariance_z:.8f} rad²/s²")
    
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
    
    # You can specify time range for statistics (in seconds)
    # For example, to analyze data from 10 to 30 seconds:
    # plot_imu_magnetometer_data(imu_file, mag_file, start_time=10.0, end_time=30.0)
    
    # Or analyze all data:
    plot_imu_magnetometer_data(imu_file, mag_file, start_time=7500, end_time=10000)
    plt.show() 