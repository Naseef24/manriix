#!/usr/bin/env python3
"""publishes RobotStatus and serves the ResetCounter service
"""
import rclpy
from rclpy.node import Node
from manriix_learn.msg import RobotStatus
from manriix_learn.srv import ResetCounter

class StatusNode(Node):
    def __init__(self):
        super().__init__('manriix_learn_status')
        self.count = 0
        self.camera_active = False

        #publish - RobotStatus on /learn/status at 1Hz
        self.pub = self.create_publisher(RobotStatus, '/learn/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

        #service server - ResetCounter on /learn/reset_counter
        self.srv = self.create_service(ResetCounter, '/learn/reset_counter', self.reset_counter_callback)
        self.get_logger().info('StatusNode started')
    
    def publish_status(self):
        self.count += 1
        msg = RobotStatus()
        msg.state = 'idle'
        msg.session_count = self.count
        msg.battery = 87.5
        msg.camera_active = self.camera_active
        self.pub.publish(msg)
        self.get_logger().info(f'Published: count={msg.session_count} camera_active={msg.camera_active}')

    def reset_counter_callback(self, request, response):
        old = self.count
        self.count = request.reset_value
        response.success = True
        response.message = f'Reset from {old} to {self.count}'
        self.get_logger().info(response.message)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
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
