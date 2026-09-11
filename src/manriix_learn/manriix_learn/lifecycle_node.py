#!/usr/bin/env python3
"""
lifecycle_node.py - LifecycleNode publisher

Demonstrates managed startup using LifecycleNode.
Mirrors the pattern used in photo_intelligence_system.py.

States:
  UNCONFIGURED → configure → INACTIVE  (create publisher)
  INACTIVE → activate → ACTIVE         (start timer)
  ACTIVE → deactivate → INACTIVE       (stop timer)
  INACTIVE → cleanup → UNCONFIGURED    (destroy publisher)
"""

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String     

# ANSI color codes — no external library needed
class C:
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    # Lifecycle states
    UNCONFIGURED = '\033[90m'   # dark grey
    INACTIVE     = '\033[33m'   # yellow
    ACTIVE       = '\033[32m'   # green
    SHUTDOWN     = '\033[31m'   # red
    # General
    INFO    = '\033[36m'        # cyan
    WARN    = '\033[33m'        # yellow
    ERROR   = '\033[31m'        # red

class LifecyclePublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__('manriix_learn_lifecycle')
        self.pub = None
        self.timer = None
        self.count = 0
        # self.get_logger().info('LifecyclePubNode created - UNCONFIGURED')
        self.get_logger().info(
            f'{C.UNCONFIGURED}{C.BOLD}[UNCONFIGURED]{C.RESET} '
            f'LifecyclePubNode created')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """create publisher. called when: ros2 lifecycle set /lifecycle_node configure"""
        try:
            self.pub = self.create_publisher(String, '/learn/status', 10)
            # self.get_logger().info('onconfigure: publisher created - INACTIVE')
            self.get_logger().info(
                f'{C.INACTIVE}{C.BOLD}[INACTIVE]{C.RESET} '
                f'on_configure: publisher created on /learn/status')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """start timer. Called when: ros2 lifecycle set /lifecycle_node activate"""
        try:
            self.timer = self.create_timer(0.5, self.publish_status)
            # self.get_logger().info('on_activate: timer started - ACTIVE') 
            self.get_logger().info(
                f'{C.ACTIVE}{C.BOLD}[ACTIVE]{C.RESET} '
                f'on_activate: timer started at 2Hz')  
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_activate failed: {e}')
            return TransitionCallbackReturn.FAILURE  

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """stop timer. called when: ros2 lifecycle set /lifecycle_node deactivate"""
        try:
            if self.timer:
                self.timer.cancel()
                self.timer = None
            # self.get_logger().info('on_deactivate: timer stopped - INACTIVE')
            self.get_logger().info(
                f'{C.INACTIVE}{C.BOLD}[INACTIVE]{C.RESET} '
                f'on_deactivate: timer stopped — paused')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_deactivate failed: {e}')
            return TransitionCallbackReturn.FAILURE      
        
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """destroy publisher. Called when: ros2 lifecycle set /lifecycle_node cleanup"""
        try:
            if self.timer:
                self.timer.cancel()
                self.timer = None
            self.pub = None
            self.count = 0
            # self.get_logger().info('on_cleanup: resources released - UNCONFIGURED')
            self.get_logger().info(
                f'{C.UNCONFIGURED}{C.BOLD}[UNCONFIGURED]{C.RESET} '
                f'on_cleanup: resources released')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_cleanup failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        """Final cleanup. Called on SIGINT or ros2 lifecycle set /lifecycle_node shutdown"""
        try:
            if self.timer:
                self.timer.cancel()
                self.timer = None
            self.pub = None
            # self.get_logger().info('on_shutdown: node shutting down')
            self.get_logger().info(
                f'{C.SHUTDOWN}{C.BOLD}[SHUTDOWN]{C.RESET} '
                f'on_shutdown: node shutting down')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_shutdown failed: {e}')
            return TransitionCallbackReturn.FAILURE               
        
    def publish_status(self):
        """called every 0.5 seconds by timer when ACTIVE"""
        self.count += 1
        msg = String()
        msg.data = f'state:idle Count: {self.count} Robot:manriix'
        self.pub.publish(msg)
        self.get_logger().info(
            f'{C.ACTIVE}→ Published{C.RESET} '
            f'count={C.BOLD}{self.count}{C.RESET} '
            f'state=idle')
        
def main(args=None):
    """Entry point- called by ros2 run command."""
    rclpy.init(args=args)
    node = LifecyclePublisherNode()
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
