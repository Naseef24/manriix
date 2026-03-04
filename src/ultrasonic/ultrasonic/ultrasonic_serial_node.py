#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class UltrasonicSerialNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_serial_node')

        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/ultrasonic', 10
        )

        self.serial_port = serial.Serial(
            '/dev/ttyACM0', 115200, timeout=0.1
        )

        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if not line:
            return

        try:
            values = [float(v) for v in line.split(',')]
            if len(values) != 6:
                return

            msg = Float32MultiArray()
            msg.data = values
            self.publisher_.publish(msg)

        except ValueError:
            pass

def main():
    rclpy.init()
    node = UltrasonicSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
