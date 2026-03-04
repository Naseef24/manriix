#!/usr/bin/env python3
"""
Set initial pose for AMCL after launch
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Declare parameters for pose
        self.declare_parameter('initial_x', 1.109)
        self.declare_parameter('initial_y', -0.881)
        self.declare_parameter('initial_yaw', 1.56)
        self.declare_parameter('delay', 3.0)
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Delay to wait for AMCL
        delay = self.get_parameter('delay').value
        self.get_logger().info(f'Waiting {delay}s for AMCL to be ready...')
        time.sleep(delay)
        
        self.set_initial_pose()
        
    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Get parameters
        x = self.get_parameter('initial_x').value
        y = self.get_parameter('initial_y').value
        yaw = self.get_parameter('initial_yaw').value
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance (6x6 matrix flattened - REQUIRED!)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'✅ Initial pose set: x={x:.3f}, y={y:.3f}, yaw={yaw:.2f} rad')
        
def main():
    rclpy.init()
    node = InitialPoseSetter()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
