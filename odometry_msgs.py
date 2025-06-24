# This script is used to publish position data to the flight controller.
# It is used to test the odometry data from the flight controller.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
import numpy as np

class PositionPublisher(Node):
    """Publishes position data to synapse_ros for remote flight controller."""
    
    def __init__(self):
        super().__init__('position_publisher')
        
        # Publisher for odometry data
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/cerebri/in/odometry', 
            10
        )
        
        # Timer to publish position updates
        self.timer = self.create_timer(0.1, self.publish_position)  # 10Hz
        
        # Example position data
        self.x = 5.0
        self.y = 5.0
        self.z = 5.0  # 1 meter altitude
        
    def publish_position(self):
        """Publish current position to the flight controller."""
        
        # Create odometry message
        odom = Odometry()
        
        # Header with timestamp and frame
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Position (x, y, z in meters)
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=self.z)
        
        # Orientation (quaternion - facing forward)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Velocity (zero for stationary)
        odom.twist.twist = Twist()
        
        # Covariance matrices (identity for simplicity)
        odom.pose.covariance = [0.1] * 36  # Position uncertainty
        odom.twist.covariance = [0.1] * 36  # Velocity uncertainty
        
        # Publish the message
        self.odom_pub.publish(odom)

def main(args=None):
    """Main function to run the position publisher."""
    rclpy.init(args=args)
    
    node = PositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()