#!/usr/bin/env python3
"""
============================================================================
qos_bridge.py — Republishes /oak/points (BEST_EFFORT) as /oak/points_reliable (RELIABLE)

PURPOSE:
  depth_image_proc::PointCloudXyziNode publishes /oak/points with BEST_EFFORT QoS
  (standard sensor_data convention).
  ground_segmentation_ros2 subscribes with default RELIABLE QoS.
  These are incompatible — ROS2 won't connect them.

  This bridge subscribes BEST_EFFORT, republishes RELIABLE.
  This matches the pattern used in zedx_multi_camera_node for STVL compatibility.

USAGE:
  Launched as part of oak_pointcloud_bridged.launch.py
  Then in another terminal:
    ros2 launch ground_segmentation_ros2 ground_segmentation.launch.py \
        pointcloud_topic:=/oak/points_reliable

File: ~/manriix2_ws/src/manriix_description/scripts/qos_bridge.py
============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2


class QosBridge(Node):
    def __init__(self):
        super().__init__('oak_points_qos_bridge')

        # Declare parameters so they can be overridden from launch
        self.declare_parameter('input_topic', '/oak/points')
        self.declare_parameter('output_topic', '/oak/points_reliable')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # BEST_EFFORT subscription (matches OAK depth_image_proc output)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # RELIABLE publication (matches ground_segmentation_ros2 default)
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, pub_qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.callback, sub_qos)

        self.message_count = 0
        self.get_logger().info(
            f'QoS Bridge ready: {input_topic} (BEST_EFFORT) → {output_topic} (RELIABLE)'
        )

        # Log status every 5 seconds
        self.create_timer(5.0, self.log_status)

    def callback(self, msg):
        self.pub.publish(msg)
        self.message_count += 1

    def log_status(self):
        self.get_logger().info(f'Bridged {self.message_count} messages so far')


def main():
    rclpy.init()
    node = QosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()