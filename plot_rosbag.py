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

def quaternion_to_euler321(qx, qy, qz, qw):
    """Convert quaternion to Euler angles (yaw-pitch-roll, ZYX order)."""
    rotation = R.from_quat([qx, qy, qz, qw])
    return rotation.as_euler('zyx', degrees=False)

def wrap_to_pi(angle):
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

def read_two_odometry_topics(bag_path, topic1, topic2):
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
    # Find overlapping time range
    min_source_time = np.min(times_source)
    max_source_time = np.max(times_source)
    
    # Filter target times to be within source range
    valid_mask = (times_target >= min_source_time) & (times_target <= max_source_time)
    valid_target_times = times_target[valid_mask]
    
    if len(valid_target_times) == 0:
        raise ValueError(f"No overlapping time range between source [{min_source_time:.3f}, {max_source_time:.3f}] and target times")
    
    # Convert quaternions to scipy Rotation object
    rotations_source = R.from_quat(quats_source)
    
    # Create SLERP interpolator
    slerp = Slerp(times_source, rotations_source)
    
    # Interpolate to valid target times
    interpolated_rotations = slerp(valid_target_times)
    
    # Convert back to quaternion array
    return valid_target_times, interpolated_rotations.as_quat()

# Interpolate topic2's data to topic1's timestamps for difference plots
def interp_to_time1(arr2, time2, time1):
    # arr2: (N, D), time2: (N,), time1: (M,)
    return np.stack([
        np.interp(time1, time2, arr2[:, i]) for i in range(arr2.shape[1])
    ], axis=1)


def compare_and_plot(data, topic1, topic2):
    poses1 = data[topic1] # ground truth (has more data)
    poses2 = data[topic2] # estimated (has less data)

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
    # Interpolates estimated quaternion to ground truth time and return valid time for both arrays
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
    plt.title("Position Difference")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔPosition (m)")
    plt.legend()
    plt.grid()

    # Orientation difference plot
    plt.figure()
    plt.plot(valid_times, eul_diff_quat_deg[:, 0], label='ΔYaw')
    plt.plot(valid_times, eul_diff_quat_deg[:, 1], label='ΔPitch')
    plt.plot(valid_times, eul_diff_quat_deg[:, 2], label='ΔRoll')
    plt.title("Orientation Difference (Quaternion Method)")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔAngle (deg)")
    plt.legend()
    plt.ylim(-5, 5)
    plt.grid()

    # # --- Raw plots for each topic (vs their own time) ---
    # # Position elements for both topics (x, y, z)
    # plt.figure()
    # plt.plot(time1, pos1[:, 0], label=f'x {topic1}')
    # plt.plot(time2, pos2[:, 0], label=f'x {topic2}')
    # plt.title("Position X Comparison")
    # plt.xlabel("Time (s)")
    # plt.ylabel("X (m)")
    # plt.legend()
    # plt.grid()

    # plt.figure()
    # plt.plot(time1, pos1[:, 1], label=f'y {topic1}')
    # plt.plot(time2, pos2[:, 1], label=f'y {topic2}')
    # plt.title("Position Y Comparison")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Y (m)")
    # plt.legend()
    # plt.grid()

    # plt.figure()
    # plt.plot(time1, pos1[:, 2], label=f'z {topic1}')
    # plt.plot(time2, pos2[:, 2], label=f'z {topic2}')
    # plt.title("Position Z Comparison")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Z (m)")
    # plt.legend()
    # plt.grid()



if __name__ == "__main__":
    rclpy.init()
    bag_path27 = "rosbag/rosbag2_2025_06_16-20_39_43"
    bag_path28 = "rosbag/rosbag2_2025_06_16-20_43_02"
    bag_path29 = "rosbag/rosbag2_2025_06_16-20_50_25"
    bag_path30 = "rosbag/rosbag2_2025_06_17-10_05_44"  
    bag_path31 = "rosbag/rosbag2_2025_06_17-10_09_25"
    bag_path32 = "rosbag/rosbag2_2025_06_17-10_29_24"  
    bag_path33 = "rosbag/rosbag2_2025_06_17-10_34_01"
    bag_path34 = "rosbag/rosbag2_2025_06_17-10_36_04"
    bag_path35 = "rosbag/rosbag2_2025_06_17-10_41_34"
    bag_path36 = "rosbag/rosbag2_2025_06_17-10_43_39"
    bag_path37 = "rosbag/rosbag2_2025_06_17-10_47_16"
    bag_path38 = "rosbag/rosbag2_2025_06_17-11_02_48"
    bag_path39 = "rosbag/rosbag2_2025_06_17-11_06_48"
    bag_path40 = "rosbag/rosbag2_2025_06_17-14_07_30"
    bag_path41 = "rosbag/rosbag2_2025_06_18-20_33_24"
    bag_path42 = "rosbag/rosbag2_2025_06_18-20_36_03"
    bag_path43 = "rosbag/rosbag2_2025_06_18-20_38_13"
    bag_path44 = "rosbag/rosbag2_2025_06_18-20_40_35"
    topic1 = "/odom"
    topic2 = "/cerebri/out/odometry"

    data27 = read_two_odometry_topics(bag_path27, topic1, topic2)
    data28 = read_two_odometry_topics(bag_path28, topic1, topic2)
    data29 = read_two_odometry_topics(bag_path29, topic1, topic2)
    data30 = read_two_odometry_topics(bag_path30, topic1, topic2)   
    data31 = read_two_odometry_topics(bag_path31, topic1, topic2)
    data32 = read_two_odometry_topics(bag_path32, topic1, topic2)
    data33 = read_two_odometry_topics(bag_path33, topic1, topic2)
    data34 = read_two_odometry_topics(bag_path34, topic1, topic2)
    data35 = read_two_odometry_topics(bag_path35, topic1, topic2)
    data36 = read_two_odometry_topics(bag_path36, topic1, topic2)
    data37 = read_two_odometry_topics(bag_path37, topic1, topic2)
    data38 = read_two_odometry_topics(bag_path38, topic1, topic2)
    data39 = read_two_odometry_topics(bag_path39, topic1, topic2)
    data40 = read_two_odometry_topics(bag_path40, topic1, topic2)
    data41 = read_two_odometry_topics(bag_path41, topic1, topic2)
    data42 = read_two_odometry_topics(bag_path42, topic1, topic2)
    data43 = read_two_odometry_topics(bag_path43, topic1, topic2)
    data44 = read_two_odometry_topics(bag_path44, topic1, topic2)
    rclpy.shutdown()
 
    #compare_and_plot(data27, topic1, topic2)   
    #compare_and_plot(data28, topic1, topic2)   
    #compare_and_plot(data29, topic1, topic2)   # no correction
    #compare_and_plot(data30, topic1, topic2)
    #compare_and_plot(data31, topic1, topic2)
    #compare_and_plot(data32, topic1, topic2)
    #compare_and_plot(data33, topic1, topic2)
    #compare_and_plot(data34, topic1, topic2)
    #compare_and_plot(data35, topic1, topic2)
    #compare_and_plot(data36, topic1, topic2)  # only with mag correction   
    #compare_and_plot(data37, topic1, topic2)
    #compare_and_plot(data38, topic1, topic2) # only with mag correction
    #compare_and_plot(data39, topic1, topic2) # with all corrections
    #compare_and_plot(data40, topic1, topic2) # only mag correction forward and stop
    compare_and_plot(data41, topic1, topic2) # fixed attitude estimator
    #compare_and_plot(data42, topic1, topic2) # fixed attitude estimator, changing yaw only
    #compare_and_plot(data43, topic1, topic2) # fixed attitude estimator, enabled spin rate
    #compare_and_plot(data44, topic1, topic2) # fixed attitude estimator, one spin CW
    plt.show()
