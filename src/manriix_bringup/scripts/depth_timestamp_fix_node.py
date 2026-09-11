#!/usr/bin/env python3
"""
depth_timestamp_fix_node.py
Republishes D455 depth image and camera_info with current ROS time.
Fixes D455 hardware timestamp offset that causes TF extrapolation errors.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class DepthTimestampFix(Node):

    def __init__(self):
        super().__init__('depth_timestamp_fix')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers — original D455 topics
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            qos)

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.info_callback,
            qos)

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers — fixed timestamp topics (RELIABLE to match depthimage_to_laserscan)
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_rect_raw_fixed',
            pub_qos)

        self.info_pub = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info_fixed',
            pub_qos)

        self.get_logger().info(
            '[DepthFix] Republishing D455 depth with ROS system time')

    def depth_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(msg)

    def info_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthTimestampFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()