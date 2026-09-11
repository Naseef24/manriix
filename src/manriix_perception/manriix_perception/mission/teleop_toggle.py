#!/usr/bin/env python3
"""
Teleop Override Toggle — Press 'P' to pause/resume autonomous mission.

Usage:
    python3 teleop_toggle.py
    
    Then press 'P' to toggle between:
      PAUSED:  Robot stops, manual teleop enabled
      RUNNING: Autonomous mission resumes with fresh POIs
    
    Press 'Q' to quit this script.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import termios
import tty


class TeleopToggle(Node):
    
    def __init__(self):
        super().__init__('teleop_toggle')
        self.publisher = self.create_publisher(Bool, '/teleop/override', 10)
        self.paused = False
        self.get_logger().info(
            "\n" + "="*50 + "\n"
            "  TELEOP OVERRIDE TOGGLE\n"
            "  Press 'P' to PAUSE/RESUME\n"
            "  Press 'Q' to QUIT\n"
            + "="*50)
    
    def toggle(self):
        self.paused = not self.paused
        msg = Bool()
        msg.data = self.paused
        self.publisher.publish(msg)
        
        if self.paused:
            self.get_logger().warn("🎮 PAUSED — manual teleop enabled")
        else:
            self.get_logger().warn("🎮 RESUMED — autonomous operation")


def get_key():
    """Read a single keypress without Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    rclpy.init()
    node = TeleopToggle()
    
    try:
        while True:
            key = get_key()
            if key.lower() == 'p':
                node.toggle()
            elif key.lower() == 'q':
                print("\nExiting teleop toggle.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()