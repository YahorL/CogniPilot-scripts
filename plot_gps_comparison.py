#!/usr/bin/env python3

"""Script to plot position data from /pose, /odom, and /gps_local topics."""

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path


def read_position_topics(mcap_path: str) -> dict[str, list]:
    """Read position data from /pose, /odom, and /gps_local topics.
    
    Args:
        mcap_path: Path to the MCAP file or directory containing MCAP files
        
    Returns:
        Dictionary with keys 'pose', 'odom', 'gps_local', each containing
        lists of tuples (position, timestamp) where position is [x, y, z]
    """
    # Handle both directory and file paths
    if os.path.isdir(mcap_path):
        # Find the .mcap file in the directory
        mcap_files = list(Path(mcap_path).glob("*.mcap"))
        if not mcap_files:
            raise ValueError(f"No .mcap files found in {mcap_path}")
        mcap_file = str(mcap_files[0])
    else:
        mcap_file = mcap_path
    
    storage_options = StorageOptions(uri=mcap_file, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    data = {
        '/pose': [],
        '/odom': [],
        '/gps_local': []
    }
    
    print(f"Reading MCAP file: {mcap_file}")
    
    while reader.has_next():
        topic, msg_data, _ = reader.read_next()
        
        if topic == '/pose':
            msg = deserialize_message(msg_data, PoseWithCovarianceStamped)
            pos = msg.pose.pose.position
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            position = np.array([pos.x, pos.y, pos.z])
            data['/pose'].append((position, timestamp))
            
        elif topic == '/odom':
            msg = deserialize_message(msg_data, Odometry)
            pos = msg.pose.pose.position
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            position = np.array([pos.x, pos.y, pos.z])
            data['/odom'].append((position, timestamp))
            
        elif topic == '/gps_local':
            msg = deserialize_message(msg_data, PoseStamped)
            pos = msg.pose.position
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            position = np.array([pos.x, pos.y, pos.z])
            data['/gps_local'].append((position, timestamp))
    
    reader.close()
    
    # Print statistics
    for topic, values in data.items():
        if values:
            timestamps = np.array([v[1] for v in values])
            print(f"{topic}: {len(values)} messages, "
                  f"duration: {timestamps[-1] - timestamps[0]:.2f} s")
        else:
            print(f"{topic}: No messages found")
    
    return data


def plot_positions(data: dict[str, list], output_file: str | None = None) -> None:
    """Plot position data from all three topics on the same plot.
    
    Args:
        data: Dictionary containing position data for each topic
        output_file: Optional path to save the plot
    """
    # Extract data for each topic
    topics_data = {}
    for topic in ['/pose', '/odom', '/gps_local']:
        if data[topic]:
            positions = np.array([v[0] for v in data[topic]])
            timestamps = np.array([v[1] for v in data[topic]])
            # Normalize timestamps to start from 0
            timestamps = timestamps - timestamps[0]
            topics_data[topic] = {
                'positions': positions,
                'timestamps': timestamps
            }
    
    if not topics_data:
        print("No data to plot!")
        return
    
    # Create figure with subplots for X, Y, Z
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Position Comparison: /pose (ground truth), /odom (estimated), /gps_local (GPS)', 
                 fontsize=14, fontweight='bold')
    
    labels = ['X', 'Y', 'Z']
    colors = {'/pose': 'blue', '/odom': 'green', '/gps_local': 'red'}
    linestyles = {'/pose': '-', '/odom': '--', '/gps_local': '-.'}
    legend_labels = {
        '/pose': 'Ground Truth',
        '/odom': 'Estimated',
        '/gps_local': 'GPS Local'
    }
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        for topic, topic_data in topics_data.items():
            ax.plot(
                topic_data['timestamps'],
                topic_data['positions'][:, i],
                label=legend_labels.get(topic, topic),
                color=colors.get(topic, 'black'),
                linestyle=linestyles.get(topic, '-'),
                linewidth=1.5,
                alpha=0.8
            )
        
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel(f'{label} Position (m)', fontsize=11)
        ax.set_title(f'{label} Position vs Time', fontsize=12)
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {output_file}")
    
    # Create zoomed X position plot (0-40 seconds)
    fig_zoom, ax_zoom = plt.subplots(1, 1, figsize=(12, 6))
    for topic, topic_data in topics_data.items():
        # Filter data to 0-40 seconds
        mask = (topic_data['timestamps'] >= 0) & (topic_data['timestamps'] <= 40)
        if np.any(mask):
            ax_zoom.plot(
                topic_data['timestamps'][mask],
                topic_data['positions'][mask, 0],
                label=legend_labels.get(topic, topic),
                color=colors.get(topic, 'black'),
                linestyle=linestyles.get(topic, '-'),
                linewidth=1.5,
                alpha=0.8
            )
    
    ax_zoom.set_xlabel('Time (s)', fontsize=11)
    ax_zoom.set_ylabel('X Position (m)', fontsize=11)
    ax_zoom.set_title('X Position vs Time', fontsize=12)
    ax_zoom.set_xlim(0, 40)
    ax_zoom.legend(loc='best', fontsize=10)
    ax_zoom.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        zoom_file = output_file.replace('.png', '_x_zoom.png')
        plt.savefig(zoom_file, dpi=300, bbox_inches='tight')
        print(f"Zoomed X plot saved to {zoom_file}")
    
    # Also create a 2D trajectory plot (X-Y)
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 8))
    for topic, topic_data in topics_data.items():
        ax2.plot(
            topic_data['positions'][:, 0],
            topic_data['positions'][:, 1],
            label=legend_labels.get(topic, topic),
            color=colors.get(topic, 'black'),
            linestyle=linestyles.get(topic, '-'),
            linewidth=1.5,
            alpha=0.8
        )
    
    ax2.set_xlabel('X Position (m)', fontsize=11)
    ax2.set_ylabel('Y Position (m)', fontsize=11)
    ax2.set_title('2D Trajectory (X-Y)', fontsize=12)
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    
    if output_file:
        traj_file = output_file.replace('.png', '_trajectory.png')
        plt.savefig(traj_file, dpi=300, bbox_inches='tight')
        print(f"Trajectory plot saved to {traj_file}")
    
    # Create zoomed 2D trajectory plot (X-Y) for first 40 seconds
    fig2_zoom, ax2_zoom = plt.subplots(1, 1, figsize=(10, 8))
    for topic, topic_data in topics_data.items():
        # Filter data to 0-40 seconds
        mask = (topic_data['timestamps'] >= 0) & (topic_data['timestamps'] <= 40)
        if np.any(mask):
            ax2_zoom.plot(
                topic_data['positions'][mask, 0],
                topic_data['positions'][mask, 1],
                label=legend_labels.get(topic, topic),
                color=colors.get(topic, 'black'),
                linestyle=linestyles.get(topic, '-'),
                linewidth=1.5,
                alpha=0.8
            )
    
    ax2_zoom.set_xlabel('X Position (m)', fontsize=11)
    ax2_zoom.set_ylabel('Y Position (m)', fontsize=11)
    ax2_zoom.set_title('2D Trajectory (X-Y) - First 40 seconds', fontsize=12)
    ax2_zoom.legend(loc='best', fontsize=10)
    ax2_zoom.grid(True, alpha=0.3)
    ax2_zoom.axis('equal')
    
    plt.tight_layout()
    
    if output_file:
        traj_zoom_file = output_file.replace('.png', '_trajectory_zoom.png')
        plt.savefig(traj_zoom_file, dpi=300, bbox_inches='tight')
        print(f"Zoomed trajectory plot saved to {traj_zoom_file}")
    
    plt.show()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Plot position data from /pose, /odom, and /gps_local topics'
    )
    parser.add_argument(
        'mcap_path',
        type=str,
        help='Path to MCAP file or directory containing MCAP files'
    )
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='Output file path for saving the plot (optional)'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.mcap_path):
        print(f"Error: Path does not exist: {args.mcap_path}")
        return
    
    rclpy.init()
    
    try:
        data = read_position_topics(args.mcap_path)
        plot_positions(data, args.output)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

