#!/usr/bin/env python3
"""Republishes point cloud with current ROS time to fix ZED timestamp sync."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PointCloudTimestampFixer(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_fixer')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        self.publisher = self.create_publisher(
            PointCloud2, '/zedx_front/point_cloud/fixed', qos)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zedx_front/zed_node/point_cloud/cloud_registered',
            self.callback, qos)
        
        self.get_logger().info('Timestamp fixer running...')

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
