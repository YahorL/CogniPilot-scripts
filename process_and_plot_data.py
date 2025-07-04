#!/usr/bin/env python3
"""
Script to process protobuf data and plot yaw, pitch, roll, and positions.
Based on SD_logging_plotting.ipynb and plot_rosbag.py functionality.
"""

import synapse_pb.frame_pb2
import delimited_protobuf
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import argparse
import os
from typing import Dict, List, Tuple, Any

def q31_to_double(q31_value: int, shift: int) -> float:
    """Convert Q31 fixed-point format to double.
    
    Args:
        q31_value: Q31 fixed-point value
        shift: Bit shift value
        
    Returns:
        Converted double value
    """
    if q31_value > 1 << 31 or q31_value < -(1 << 31):
        raise IOError("out of range")
    return q31_value / (1 << (31 - shift))

def quaternion_to_euler321(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to Euler angles (yaw-pitch-roll, ZYX order).
    
    Args:
        qx: Quaternion x component
        qy: Quaternion y component
        qz: Quaternion z component
        qw: Quaternion w component
        
    Returns:
        Euler angles as [yaw, pitch, roll] in radians
    """
    rotation = R.from_quat([qx, qy, qz, qw])
    return rotation.as_euler('zyx', degrees=False)

def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi] range."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def read_protobuf_data(file_path: str) -> Dict[str, np.ndarray]:
    """Read protobuf data from file and extract relevant information.
    
    Args:
        file_path: Path to the protobuf file
        
    Returns:
        Dictionary containing processed data arrays
    """
    data = {
        'imu': [],
        'imu_q31_array': [],
        'pwm': [],
        'mag': [],
        'odom': [],
        'input': [],
    }
    
    print(f"Reading data from {file_path}...")
    
    with open(file_path, 'rb') as file:
        frame_count = 0
        while True:
            try:
                frame = delimited_protobuf.read(file, synapse_pb.frame_pb2.Frame)
                if frame is None:
                    break
                    
                frame_count += 1
                if frame_count % 1000 == 0:
                    print(f"Processed {frame_count} frames...")
                    
                match frame.WhichOneof('msg'):
                    case 'imu':
                        msg = frame.imu
                        t = msg.stamp.seconds + msg.stamp.nanos * 1e-9
                        gx = msg.angular_velocity.x
                        gy = msg.angular_velocity.y
                        gz = msg.angular_velocity.z
                        ax = msg.linear_acceleration.x
                        ay = msg.linear_acceleration.y
                        az = msg.linear_acceleration.z
                        data['imu'].append((t, gx, gy, gz, ax, ay, az))
                        
                    case 'pwm':
                        msg = frame.pwm
                        data['pwm'].append((0, msg.channel[0], msg.channel[1], msg.channel[2], msg.channel[3]))
                        
                    case 'odometry':
                        msg = frame.odometry
                        t = msg.stamp.seconds + msg.stamp.nanos * 1e-9
                        odom_x = msg.pose.position.x
                        odom_y = msg.pose.position.y
                        odom_z = msg.pose.position.z
                        odom_q0 = msg.pose.orientation.x
                        odom_q1 = msg.pose.orientation.y
                        odom_q2 = msg.pose.orientation.z
                        odom_q3 = msg.pose.orientation.w
                        data['odom'].append((t, odom_x, odom_y, odom_z, odom_q0, odom_q1, odom_q2, odom_q3))
                        
                    case 'magnetic_field':
                        msg = frame.magnetic_field
                        t = msg.stamp.seconds + msg.stamp.nanos * 1e-9
                        mag_x = msg.magnetic_field.x
                        mag_y = msg.magnetic_field.y
                        mag_z = msg.magnetic_field.z
                        data['mag'].append((t, mag_x, mag_y, mag_z))
                        
                    case 'input':
                        msg = frame.input
                        data['input'].append((0, msg.channel[0], msg.channel[1], msg.channel[2], msg.channel[3]))
                        
                    case 'imu_q31_array':
                        msg = frame.imu_q31_array
                        t = msg.stamp.seconds + msg.stamp.nanos * 1e-9
                        for f in msg.frame:
                            dt = f.delta_nanos * 1e-9
                            data['imu_q31_array'].append((t + dt,
                                q31_to_double(f.gyro_x, msg.gyro_shift),
                                q31_to_double(f.gyro_y, msg.gyro_shift),
                                q31_to_double(f.gyro_z, msg.gyro_shift),
                                q31_to_double(f.accel_x, msg.accel_shift),
                                q31_to_double(f.accel_y, msg.accel_shift),
                                q31_to_double(f.accel_z, msg.accel_shift),
                                q31_to_double(f.temp, msg.temp_shift)
                            ))
                    case _:
                        pass
                        
            except Exception as e:
                print(f"Error reading frame {frame_count}: {e}")
                break
    
    print(f"Total frames processed: {frame_count}")
    
    # Convert to numpy arrays with proper dtypes
    if data['imu']:
        data['imu'] = np.array(
            object=data['imu'],
            dtype=[('t', 'f8'), ('gx', 'f8'), ('gy', 'f8'), ('gz', 'f8'), ('ax', 'f8'), ('ay', 'f8'), ('az', 'f8')]
        )
    
    if data['odom']:
        data['odom'] = np.array(
            object=data['odom'],
            dtype=[('t', 'f8'), ('odom_x', 'f8'), ('odom_y', 'f8'), ('odom_z', 'f8'), 
                   ('odom_q0', 'f8'), ('odom_q1', 'f8'), ('odom_q2', 'f8'), ('odom_q3', 'f8')]
        )
    
    if data['mag']:
        data['mag'] = np.array(
            object=data['mag'],
            dtype=[('t', 'f8'), ('mag_x', 'f8'), ('mag_y', 'f8'), ('mag_z', 'f8')]
        )
    
    if data['imu_q31_array']:
        data['imu_q31_array'] = np.array(
            object=data['imu_q31_array'],
            dtype=[('t', 'f8'), ('gx', 'f8'), ('gy', 'f8'), ('gz', 'f8'), 
                   ('ax', 'f8'), ('ay', 'f8'), ('az', 'f8'), ('temp', 'f8')]
        )
    
    if data['pwm']:
        data['pwm'] = np.array(
            object=data['pwm'],
            dtype=[('t', 'f8'), ('c0', 'f8'), ('c1', 'f8'), ('c2', 'f8'), ('c3', 'f8')]
        )
    
    if data['input']:
        data['input'] = np.array(
            object=data['input'],
            dtype=[('t', 'f8'), ('c0', 'f8'), ('c1', 'f8'), ('c2', 'f8'), ('c3', 'f8')]
        )
    
    return data

def calculate_euler_angles(data: Dict[str, np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    """Calculate Euler angles from quaternions in odometry data.
    
    Args:
        data: Dictionary containing processed data
        
    Returns:
        Tuple of (timestamps, euler_angles) where euler_angles is [yaw, pitch, roll]
    """
    if 'odom' not in data or len(data['odom']) == 0:
        print("No odometry data found!")
        return np.array([]), np.array([])
    
    odom_data = data['odom']
    timestamps = odom_data['t']
    euler_angles = []
    
    print("Calculating Euler angles from quaternions...")
    
    for i in range(len(odom_data)):
        qx = odom_data['odom_q0'][i]  # x component
        qy = odom_data['odom_q1'][i]  # y component
        qz = odom_data['odom_q2'][i]  # z component
        qw = odom_data['odom_q3'][i]  # w component
        
        euler = quaternion_to_euler321(qx, qy, qz, qw)
        euler_angles.append(euler)
    
    return timestamps, np.array(euler_angles)

def plot_data(data: Dict[str, np.ndarray], output_dir: str = "plots") -> None:
    """Create comprehensive plots of the processed data.
    
    Args:
        data: Dictionary containing processed data
        output_dir: Directory to save plots
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Calculate Euler angles
    timestamps, euler_angles = calculate_euler_angles(data)
    
    # Set up the plotting style
    plt.style.use('default')
    plt.rcParams['figure.figsize'] = (12, 8)
    plt.rcParams['font.size'] = 10
    
    # 1. Plot Euler Angles (Yaw, Pitch, Roll)
    if len(euler_angles) > 0:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Euler Angles (ZYX Convention)', fontsize=14, fontweight='bold')
        
        # Yaw (Z-axis rotation)
        axes[0].plot(timestamps, np.rad2deg(euler_angles[:, 0]), 'b-', linewidth=1.5, label='Yaw')
        axes[0].set_ylabel('Yaw (degrees)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Pitch (Y-axis rotation)
        axes[1].plot(timestamps, np.rad2deg(euler_angles[:, 1]), 'g-', linewidth=1.5, label='Pitch')
        axes[1].set_ylabel('Pitch (degrees)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Roll (X-axis rotation)
        axes[2].plot(timestamps, np.rad2deg(euler_angles[:, 2]), 'r-', linewidth=1.5, label='Roll')
        axes[2].set_ylabel('Roll (degrees)')
        axes[2].set_xlabel('Time (seconds)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'euler_angles.png'), dpi=300, bbox_inches='tight')
    
    # 2. Plot Position
    if 'odom' in data and len(data['odom']) > 0:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Position', fontsize=14, fontweight='bold')
        
        odom_data = data['odom']
        
        # X position
        axes[0].plot(odom_data['t'], odom_data['odom_x'], 'b-', linewidth=1.5, label='X')
        axes[0].set_ylabel('X Position (m)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Y position
        axes[1].plot(odom_data['t'], odom_data['odom_y'], 'g-', linewidth=1.5, label='Y')
        axes[1].set_ylabel('Y Position (m)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Z position
        axes[2].plot(odom_data['t'], odom_data['odom_z'], 'r-', linewidth=1.5, label='Z')
        axes[2].set_ylabel('Z Position (m)')
        axes[2].set_xlabel('Time (seconds)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'position.png'), dpi=300, bbox_inches='tight')
    
    # 3. Plot IMU Data (Angular Velocity)
    if 'imu' in data and len(data['imu']) > 0:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('IMU Angular Velocity', fontsize=14, fontweight='bold')
        
        imu_data = data['imu']
        
        # Gyro X
        axes[0].plot(imu_data['t'], imu_data['gx'], 'b-', linewidth=1.5, label='ωx')
        axes[0].set_ylabel('Angular Velocity X (rad/s)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Gyro Y
        axes[1].plot(imu_data['t'], imu_data['gy'], 'g-', linewidth=1.5, label='ωy')
        axes[1].set_ylabel('Angular Velocity Y (rad/s)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Gyro Z
        axes[2].plot(imu_data['t'], imu_data['gz'], 'r-', linewidth=1.5, label='ωz')
        axes[2].set_ylabel('Angular Velocity Z (rad/s)')
        axes[2].set_xlabel('Time (seconds)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'angular_velocity.png'), dpi=300, bbox_inches='tight')
    
    # 4. Plot IMU Data (Linear Acceleration)
    if 'imu' in data and len(data['imu']) > 0:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('IMU Linear Acceleration', fontsize=14, fontweight='bold')
        
        imu_data = data['imu']
        
        # Accel X
        axes[0].plot(imu_data['t'], imu_data['ax'], 'b-', linewidth=1.5, label='ax')
        axes[0].set_ylabel('Acceleration X (m/s²)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Accel Y
        axes[1].plot(imu_data['t'], imu_data['ay'], 'g-', linewidth=1.5, label='ay')
        axes[1].set_ylabel('Acceleration Y (m/s²)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Accel Z
        axes[2].plot(imu_data['t'], imu_data['az'], 'r-', linewidth=1.5, label='az')
        axes[2].set_ylabel('Acceleration Z (m/s²)')
        axes[2].set_xlabel('Time (seconds)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'linear_acceleration.png'), dpi=300, bbox_inches='tight')
    
    # 5. Plot Magnetic Field Data
    if 'mag' in data and len(data['mag']) > 0:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Magnetic Field', fontsize=14, fontweight='bold')
        
        mag_data = data['mag']
        
        # Mag X
        axes[0].plot(mag_data['t'], mag_data['mag_x'], 'b-', linewidth=1.5, label='Bx')
        axes[0].set_ylabel('Magnetic Field X (μT)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Mag Y
        axes[1].plot(mag_data['t'], mag_data['mag_y'], 'g-', linewidth=1.5, label='By')
        axes[1].set_ylabel('Magnetic Field Y (μT)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Mag Z
        axes[2].plot(mag_data['t'], mag_data['mag_z'], 'r-', linewidth=1.5, label='Bz')
        axes[2].set_ylabel('Magnetic Field Z (μT)')
        axes[2].set_xlabel('Time (seconds)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'magnetic_field.png'), dpi=300, bbox_inches='tight')
    
    # 6. Combined Overview Plot
    if len(euler_angles) > 0 and 'odom' in data and len(data['odom']) > 0:
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Flight Data Overview', fontsize=16, fontweight='bold')
        
        odom_data = data['odom']
        
        # Yaw
        axes[0, 0].plot(timestamps, np.rad2deg(euler_angles[:, 0]), 'b-', linewidth=1.5)
        axes[0, 0].set_title('Yaw')
        axes[0, 0].set_ylabel('Yaw (degrees)')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Pitch
        axes[0, 1].plot(timestamps, np.rad2deg(euler_angles[:, 1]), 'g-', linewidth=1.5)
        axes[0, 1].set_title('Pitch')
        axes[0, 1].set_ylabel('Pitch (degrees)')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Roll
        axes[1, 0].plot(timestamps, np.rad2deg(euler_angles[:, 2]), 'r-', linewidth=1.5)
        axes[1, 0].set_title('Roll')
        axes[1, 0].set_ylabel('Roll (degrees)')
        axes[1, 0].set_xlabel('Time (seconds)')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Position (XY trajectory)
        axes[1, 1].plot(odom_data['odom_x'], odom_data['odom_y'], 'k-', linewidth=1.5)
        axes[1, 1].set_title('XY Trajectory')
        axes[1, 1].set_xlabel('X Position (m)')
        axes[1, 1].set_ylabel('Y Position (m)')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'overview.png'), dpi=300, bbox_inches='tight')
    
    print(f"Plots saved to {output_dir}/ directory")
    
    # Show all plots at the end
    plt.show()

def print_data_summary(data: Dict[str, np.ndarray]) -> None:
    """Print a summary of the processed data.
    
    Args:
        data: Dictionary containing processed data
    """
    print("\n" + "="*50)
    print("DATA SUMMARY")
    print("="*50)
    
    for key, value in data.items():
        if isinstance(value, np.ndarray) and len(value) > 0:
            print(f"{key.upper()}: {len(value)} samples")
            if 't' in value.dtype.names:
                duration = value['t'][-1] - value['t'][0]
                print(f"  Duration: {duration:.2f} seconds")
                print(f"  Time range: {value['t'][0]:.2f} to {value['t'][-1]:.2f}")
        else:
            print(f"{key.upper()}: No data")
    
    print("="*50)

def main():
    """Main function to process and plot data."""
    parser = argparse.ArgumentParser(description='Process protobuf data and plot yaw, pitch, roll, and positions')
    parser.add_argument('--input', '-i', type=str, default='log_data/data_flight.pb',
                       help='Input protobuf file path (default: log_data/data_flight.pb)')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output directory for plots (default: auto-generated from input filename)')
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input):
        print(f"Error: Input file '{args.input}' not found!")
        return
    
    # Generate output directory name from input filename if not specified
    if args.output is None:
        # Get the base filename without extension
        base_name = os.path.splitext(os.path.basename(args.input))[0]
        output_dir = f"plots/plots_{base_name}"
    else:
        output_dir = args.output
    
    try:
        # Read and process data
        data = read_protobuf_data(args.input)
        
        # Print summary
        print_data_summary(data)
        
        # Create plots
        plot_data(data, output_dir=output_dir)
            
    except Exception as e:
        print(f"Error processing data: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 