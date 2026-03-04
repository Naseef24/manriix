#!/usr/bin/env python3
"""
Publishes robot pose in map frame by looking up TF transform.
This bridges the gap between TF and web interface.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/robot_pose_map', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.publish_pose)  # 20 Hz
        self.get_logger().info('Pose publisher started - publishing /robot_pose_map at 20Hz')

    def publish_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                # 'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.1))
                'map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.publisher.publish(pose)
        except Exception as e:
            pass  # Silent fail - TF might not be ready

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
