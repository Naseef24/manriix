#!/usr/bin/env python3
"""
docstring->
publisher_node.py - A simple ROS 2 publisher node that publishes a String message to a topic at regular intervals.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('manriix_learn_publisher')
        self.pub = self.create_publisher(String, '/learn/status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.count = 0
        self.get_logger().info('Publisher node has been started and publishing at 1 Hz.')

    def publish_status(self):
        """called every 1.0 seconds by the timer to publish a message to the topic."""
        self.count += 1
        msg = String()
        msg.data = f'state:idle Count: {self.count} Robot:manriix'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    """Entry point- called by ros2 run command."""
    rclpy.init(args=args)
    node = PublisherNode()
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

