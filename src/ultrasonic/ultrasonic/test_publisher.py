#!/usr/bin/env python3
"""
Test Publisher for Ultrasonic Safety System

Publishes simulated ultrasonic sensor data for testing without hardware.
Use this to verify the E-Stop and Costmap nodes are working correctly.

Usage:
    ros2 run ultrasonic_safety test_publisher

This publishes to /ultrasonic with simulated sensor readings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import math


class TestPublisher(Node):
    """
    Test publisher that simulates ultrasonic sensor readings.
    """
    
    def __init__(self):
        super().__init__('ultrasonic_test_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('simulate_obstacle', False)
        self.declare_parameter('obstacle_sensor', 1)  # front_center
        self.declare_parameter('obstacle_distance', 10.0)  # cm
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.num_sensors = self.get_parameter('num_sensors').value
        self.simulate_obstacle = self.get_parameter('simulate_obstacle').value
        self.obstacle_sensor = self.get_parameter('obstacle_sensor').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        
        # Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'ultrasonic',
            10
        )
        
        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )
        
        # State
        self.count = 0
        self.base_distances = [100.0, 150.0, 120.0, 200.0, 180.0, 130.0]
        
        self.get_logger().info(
            f'Test publisher started at {self.publish_rate}Hz\n'
            f'Simulate obstacle: {self.simulate_obstacle}'
        )

    def timer_callback(self):
        """Publish simulated sensor data"""
        msg = Float32MultiArray()
        
        # Generate readings with some noise
        distances = []
        for i in range(self.num_sensors):
            base = self.base_distances[i]
            noise = random.gauss(0, 2)  # ±2cm noise
            distance = base + noise + 20 * math.sin(self.count * 0.05 + i)
            
            # Simulate obstacle on specified sensor
            if self.simulate_obstacle and i == self.obstacle_sensor:
                distance = self.obstacle_distance + random.gauss(0, 0.5)
            
            # Clamp to valid range
            distance = max(2.0, min(400.0, distance))
            distances.append(distance)
        
        msg.data = distances
        self.publisher.publish(msg)
        
        self.count += 1
        
        # Log periodically
        if self.count % 100 == 0:
            self.get_logger().info(
                f'Published {self.count} messages. '
                f'Front center: {distances[1]:.1f}cm'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
