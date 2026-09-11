#!/usr/bin/env python3
"""subscribes to the /learn/status topic and logs the received messages."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('manriix_learn_subscriber')
        self.sub = self.create_subscription(String, '/learn/status', self.listener_callback, 10)
        self.get_logger().info('Subscriber node has been started and is listening to /learn/status topic.') 

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
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