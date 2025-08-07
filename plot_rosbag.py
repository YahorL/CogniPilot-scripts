import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from geometry_msgs.msg import Quaternion
import tf_transformations
import os

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

def quaternion_difference(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Compute relative rotation from q2 to q1.
    
    Args:
        q1: Target quaternion(s) [x, y, z, w] format, shape (..., 4)
        q2: Reference quaternion(s) [x, y, z, w] format, shape (..., 4)
    
    Returns:
        Relative rotation quaternion q_diff = q1 * q2^(-1)
    """
    # Handle both single quaternions and arrays
    q1 = np.atleast_2d(q1)
    q2 = np.atleast_2d(q2)
    
    # Convert to scipy Rotation objects
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    
    # Compute relative rotation: r_diff = r1 * r2^(-1)
    r_diff = r1 * r2.inv()
    
    return r_diff.as_quat()

def read_single_odometry_topic(bag_path: str, topic: str) -> list:
    """Read odometry data from a single topic in a rosbag.
    
    Args:
        bag_path: Path to the rosbag file
        topic: Topic name to read
        
    Returns:
        List of tuples containing (position, euler_angles, quaternion, timestamp)
    """
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = []

    while reader.has_next():
        current_topic, msg_data, _ = reader.read_next()
        if current_topic == topic:
            msg = deserialize_message(msg_data, Odometry)
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            position = np.array([pos.x, pos.y, pos.z])
            euler = quaternion_to_euler321(ori.x, ori.y, ori.z, ori.w)
            quaternion = np.array([ori.x, ori.y, ori.z, ori.w])

            data.append((position, euler, quaternion, timestamp))

    return data

def read_two_odometry_topics(bag_path: str, topic1: str, topic2: str) -> dict:
    """Read odometry data from two topics in a rosbag.
    
    Args:
        bag_path: Path to the rosbag file
        topic1: First topic name
        topic2: Second topic name
        
    Returns:
        Dictionary with topic data containing position, euler angles, quaternions, and timestamps
    """
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = {topic1: [], topic2: []}

    while reader.has_next():
        topic, msg_data, _ = reader.read_next()
        if topic in data:
            msg = deserialize_message(msg_data, Odometry)
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            position = np.array([pos.x, pos.y, pos.z])
            euler = quaternion_to_euler321(ori.x, ori.y, ori.z, ori.w)
            quaternion = np.array([ori.x, ori.y, ori.z, ori.w])

            data[topic].append((position, euler, quaternion, timestamp))

    return data

def interpolate_rotations(times_source: np.ndarray, quats_source: np.ndarray, times_target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Interpolate quaternions from source timestamps to target timestamps using SLERP.

    Args:
        times_source: Source timestamps array (shape: [N,])
        quats_source: Source quaternions array (shape: [N, 4]) in [x, y, z, w] format
        times_target: Target timestamps for interpolation (shape: [M,])

    Returns:
        Tuple of (valid_times, interpolated_quaternions) where:
        - valid_times: Target times that were within interpolation range (shape: [K,])
        - interpolated_quaternions: Corresponding interpolated quaternions (shape: [K, 4])
    """
    # Sort source data by timestamp to ensure strictly increasing order
    sort_indices = np.argsort(times_source)
    times_source_sorted = times_source[sort_indices]
    quats_source_sorted = quats_source[sort_indices]
    
    # Remove duplicate timestamps (keep first occurrence)
    unique_indices = np.append([0], np.where(np.diff(times_source_sorted) > 0)[0] + 1)
    times_source_unique = times_source_sorted[unique_indices]
    quats_source_unique = quats_source_sorted[unique_indices]
    
    # Find overlapping time range
    min_source_time = np.min(times_source_unique)
    max_source_time = np.max(times_source_unique)
    
    # Filter target times to be within source range
    valid_mask = (times_target >= min_source_time) & (times_target <= max_source_time)
    valid_target_times = times_target[valid_mask]
    
    if len(valid_target_times) == 0:
        raise ValueError(f"No overlapping time range between source [{min_source_time:.3f}, {max_source_time:.3f}] and target times")
    
    # Convert quaternions to scipy Rotation object
    rotations_source = R.from_quat(quats_source_unique)
    
    # Create SLERP interpolator with sorted, unique timestamps
    print(f"Source times range: [{times_source_unique[0]:.3f}, {times_source_unique[-1]:.3f}], count: {len(times_source_unique)}")
    slerp = Slerp(times_source_unique, rotations_source)
    
    # Interpolate to valid target times
    interpolated_rotations = slerp(valid_target_times)
    
    # Convert back to quaternion array
    return valid_target_times, interpolated_rotations.as_quat()

def interp_to_time1(arr2: np.ndarray, time2: np.ndarray, time1: np.ndarray) -> np.ndarray:
    """Interpolate topic2's data to topic1's timestamps for difference plots.
    
    Args:
        arr2: Array to interpolate (N, D)
        time2: Source timestamps (N,)
        time1: Target timestamps (M,)
        
    Returns:
        Interpolated array (M, D)
    """
    return np.stack([
        np.interp(time1, time2, arr2[:, i]) for i in range(arr2.shape[1])
    ], axis=1)

def plot_single_topic_odometry(filename: str, topic: str, rosbag_dir: str = "rosbag") -> None:
    """Plot attitude and position data from a single odometry topic.
    
    Args:
        filename: Name of the rosbag file (without full path)
        topic: Topic name to plot
        rosbag_dir: Directory containing rosbag files (default: "rosbag")
    """
    # Construct full path to rosbag
    bag_path = os.path.join(rosbag_dir, filename)
    
    # Read data from the topic
    poses = read_single_odometry_topic(bag_path, topic)
    
    if not poses:
        print(f"No data found for topic {topic} in {filename}")
        return
    
    # Extract data
    positions = np.array([p[0] for p in poses])
    euler_angles = np.array([p[1] for p in poses])
    timestamps = np.array([p[3] for p in poses])
    
    # Convert to degrees for plotting
    euler_angles_deg = np.rad2deg(euler_angles)
    
    # Create subplots for position and attitude
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'{topic} - {filename}', fontsize=16)
    
    # Position plots
    ax1.plot(timestamps, positions[:, 0], label='X', linewidth=2)
    ax1.plot(timestamps, positions[:, 1], label='Y', linewidth=2)
    ax1.plot(timestamps, positions[:, 2], label='Z', linewidth=2)
    ax1.set_title('Position')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)')
    ax1.legend()
    ax1.grid(True)
    
    # 3D trajectory
    ax2.plot(positions[:, 0], positions[:, 1], linewidth=2)
    ax2.set_title('2D Trajectory (X-Y)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.grid(True)
    ax2.axis('equal')
    
    # Attitude plots
    ax3.plot(timestamps, euler_angles_deg[:, 0], label='Yaw', linewidth=2)
    ax3.plot(timestamps, euler_angles_deg[:, 1], label='Pitch', linewidth=2)
    ax3.plot(timestamps, euler_angles_deg[:, 2], label='Roll', linewidth=2)
    ax3.set_title('Euler Angles')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (deg)')
    ax3.legend()
    ax3.grid(True)
    
    # Altitude plot
    ax4.plot(timestamps, positions[:, 2], label='Altitude', linewidth=2, color='red')
    ax4.set_title('Altitude')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Altitude (m)')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Print some statistics
    print(f"\nStatistics for {topic} in {filename}:")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
    print(f"Position range - X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
    print(f"Position range - Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
    print(f"Position range - Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
    print(f"Attitude range - Yaw: [{euler_angles_deg[:, 0].min():.1f}, {euler_angles_deg[:, 0].max():.1f}] deg")
    print(f"Attitude range - Pitch: [{euler_angles_deg[:, 1].min():.1f}, {euler_angles_deg[:, 1].max():.1f}] deg")
    print(f"Attitude range - Roll: [{euler_angles_deg[:, 2].min():.1f}, {euler_angles_deg[:, 2].max():.1f}] deg")

def compare_and_plot(filename: str, topic1: str, topic2: str, rosbag_dir: str = "rosbag") -> None:
    """Compare and plot odometry data from two topics in a rosbag file.
    
    Args:
        filename: Name of the rosbag file (without full path)
        topic1: First topic name (typically ground truth)
        topic2: Second topic name (typically estimated)
        rosbag_dir: Directory containing rosbag files (default: "rosbag")
    """
    # Construct full path to rosbag
    bag_path = os.path.join(rosbag_dir, filename)
    
    # Read data from both topics
    data = read_two_odometry_topics(bag_path, topic1, topic2)
    
    poses1 = data[topic1]  # ground truth (has more data)
    poses2 = data[topic2]  # estimated (has less data)

    pos1 = np.array([p[0] for p in poses1])
    eul1 = np.array([p[1] for p in poses1])
    quat1 = np.array([p[2] for p in poses1])
    time1 = np.array([p[3] for p in poses1])
    pos2 = np.array([p[0] for p in poses2])
    eul2 = np.array([p[1] for p in poses2])
    quat2 = np.array([p[2] for p in poses2])
    time2 = np.array([p[3] for p in poses2])

    pos2_interp = interp_to_time1(pos2, time2, time1)
    pos_diff = pos1 - pos2_interp

    # Get interpolated quaternions for overlapping time range
    # Interpolates estimated quaternion to ground truth time and return valid time for both arrays)
    valid_times, quat2_interp_slerp = interpolate_rotations(time2, quat2, time1)

    # Find indices in time1 that correspond to valid_times
    valid_indices = np.searchsorted(time1, valid_times)
    
    # Ensure indices are within bounds
    valid_indices = valid_indices[valid_indices < len(time1)]
    valid_times = valid_times[:len(valid_indices)]
    quat2_interp_slerp = quat2_interp_slerp[:len(valid_indices)]

    # Extract corresponding data from topic1 (ground truth) to match interpolated data
    quat1_valid = quat1[valid_indices]

    # Now compute quaternion differences with matching array sizes
    quat_diff_array = quaternion_difference(quat1_valid, quat2_interp_slerp)
    eul_diff_quat = np.array([quaternion_to_euler321(q[0], q[1], q[2], q[3]) for q in quat_diff_array])
    eul_diff_quat_deg = np.rad2deg(eul_diff_quat)

    # --- Difference plots (aligned to topic1's time) ---
    # Position difference plot
    plt.figure()
    plt.plot(time1, pos_diff[:, 0], label='Δx')
    plt.plot(time1, pos_diff[:, 1], label='Δy')
    plt.plot(time1, pos_diff[:, 2], label='Δz')
    plt.title(f"Position Difference - {filename}")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔPosition (m)")
    plt.legend()
    plt.grid()

    # Orientation difference plot
    plt.figure()
    plt.plot(valid_times, eul_diff_quat_deg[:, 0], label='ΔYaw')
    plt.plot(valid_times, eul_diff_quat_deg[:, 1], label='ΔPitch')
    plt.plot(valid_times, eul_diff_quat_deg[:, 2], label='ΔRoll')
    plt.title(f"Orientation Difference (Quaternion Method) - {filename}")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔAngle (deg)")
    plt.legend()
    plt.ylim(-10, 10)
    plt.grid()

if __name__ == "__main__":
    rclpy.init()
    
    # Define topics
    topic1 = "/odom"
    topic2 = "/cerebri/out/odometry"
    
    # Usage
    #compare_and_plot("rosbag2_2025_06_18-20_33_24", topic1, topic2)  # fixed attitude estimator
    #compare_and_plot("rosbag2_2025_06_17-11_06_48", topic1, topic2)  # old attitude estimator
    #compare_and_plot("rosbag2_2025_07_03-13_33_45", topic1, topic2)  # estimator pre flight
    #compare_and_plot("rosbag2_2025_07_09-19_10_57", topic1, topic2)  # fixed roll/pitch estimation
    #compare_and_plot("rosbag2_2025_07_09-19_13_24", topic1, topic2)  # fixed roll/pitch estimation, small gains
    #compare_and_plot("rosbag2_2025_07_15-20_54_12", topic1, topic2)  # 
    #compare_and_plot("rosbag2_2025_07_29-14_31_12", topic1, topic2)  # without correction
    compare_and_plot("rosbag2_2025_08_04-17_18_05", topic1, topic2)
    #compare_and_plot("output", topic1, topic2)

    # Add single topic plotting for cerebri/out/odometry
    #plot_single_topic_odometry("rosbag2_2025_07_04-17_53_47", topic2)  # gazebo stationary on the ground
    #plot_single_topic_odometry("rosbag2_2025_07_07-13_58_58", topic2) # gazebo stationary in flight
    #plot_single_topic_odometry("rosbag2_2025_07_07-18_57_57", topic2) # gazebo stationary in flight with noise
    #plot_single_topic_odometry("rosbag2_2025_07_07-23_57_40", topic2) # gazebo stationary in flight with noise
    #plot_single_topic_odometry("rosbag2_2025_07_09-17_46_20", topic2) # gazebo stationary, increased noise
    #plot_single_topic_odometry("rosbag2_2025_07_09-17_56_59", topic2) # gazebo stationary, increased noise, increased gyro bias
    # plot_single_topic_odometry("rosbag2_2025_07_09-20_34_20", topic1)
    # plot_single_topic_odometry("rosbag2_2025_07_09-20_34_20", topic2)

    rclpy.shutdown()
    plt.show()
