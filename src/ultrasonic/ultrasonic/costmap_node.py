#!/usr/bin/env python3
"""
Ultrasonic to Costmap Node

Converts ultrasonic sensor readings to PointCloud2 messages for 
integration with Nav2 costmap layers.

Features:
- Transforms sensor readings to robot base frame
- Publishes PointCloud2 for costmap obstacle layer
- Broadcasts static TF transforms for sensor frames
- Configurable sensor positions and orientations

Topics:
    Subscribed:
        /ultrasonic (std_msgs/Float32MultiArray): Raw sensor readings
    Published:
        /ultrasonic_pointcloud (sensor_msgs/PointCloud2): Obstacle points

TF Frames:
    Publishes static transforms from base_link to each sensor frame

Parameters:
    sensor_positions (list): [x, y, angle] for each sensor in cm and radians
    max_range (float): Maximum valid range in cm
    min_range (float): Minimum valid range in cm
    base_frame (str): Robot base frame name
    sensor_height (float): Height of sensors in meters

Author: Hype
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import struct
import math


class CostmapNode(Node):
    """
    Costmap Integration Node
    
    Converts ultrasonic readings to PointCloud2 for Nav2 costmap.
    """
    
    def __init__(self):
        super().__init__('ultrasonic_costmap_node')
        
        # ============== PARAMETERS ==============
        
        # Sensor positions relative to base_link
        # Format: [x_cm, y_cm, angle_rad] for each sensor
        # x: positive = forward, negative = backward
        # y: positive = left, negative = right
        # angle: direction sensor faces (0 = forward, π/2 = left, π = backward)
        
        # Default positions - ADJUST THESE TO YOUR ROBOT
        default_positions = [
            30.0,  20.0,  0.52,     # front_left: 30cm forward, 20cm left, facing 30° left
            35.0,   0.0,  0.0,      # front_center: 35cm forward, centered, facing forward
            30.0, -20.0, -0.52,     # front_right: 30cm forward, 20cm right, facing 30° right
           -30.0, -20.0, -2.62,     # rear_right: 30cm back, 20cm right, facing 150° right
           -35.0,   0.0,  3.14159,  # rear_center: 35cm back, centered, facing backward
           -30.0,  20.0,  2.62,     # rear_left: 30cm back, 20cm left, facing 150° left
        ]
        
        self.declare_parameter('sensor_positions', default_positions)
        self.declare_parameter('max_range', 400.0)   # cm
        self.declare_parameter('min_range', 2.0)     # cm
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_height', 0.10)  # meters
        self.declare_parameter('invalid_reading_value', 999.0)
        
        # Sensor names for TF frames
        self.declare_parameter(
            'sensor_names',
            ['front_left', 'front_center', 'front_right',
             'rear_right', 'rear_center', 'rear_left']
        )
        
        # Get parameters
        positions = self.get_parameter('sensor_positions').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_height = self.get_parameter('sensor_height').value
        self.invalid_value = self.get_parameter('invalid_reading_value').value
        self.sensor_names = self.get_parameter('sensor_names').value
        
        # Parse sensor positions into list of tuples
        self.sensor_positions = [
            (positions[i], positions[i+1], positions[i+2])
            for i in range(0, len(positions), 3)
        ]
        
        self.num_sensors = len(self.sensor_positions)
        
        # ============== QOS PROFILES ==============
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # ============== PUBLISHERS ==============
        
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'ultrasonic_pointcloud',
            10
        )
        
        # ============== SUBSCRIBERS ==============
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ultrasonic',
            self.ultrasonic_callback,
            sensor_qos
        )
        
        # ============== TF ==============
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_sensor_frames()
        
        # ============== STARTUP ==============
        
        self.get_logger().info(
            f'Costmap node started\n'
            f'  Base frame: {self.base_frame}\n'
            f'  Range: {self.min_range} - {self.max_range} cm\n'
            f'  Sensor height: {self.sensor_height}m\n'
            f'  Sensors: {self.num_sensors}'
        )

    def publish_sensor_frames(self):
        """
        Publish static TF transforms for each sensor position.
        This allows RViz to visualize sensor locations.
        """
        transforms = []
        
        for i, (x, y, angle) in enumerate(self.sensor_positions):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = f'ultrasonic_{self.sensor_names[i]}'
            
            # Convert cm to meters
            t.transform.translation.x = x / 100.0
            t.transform.translation.y = y / 100.0
            t.transform.translation.z = self.sensor_height
            
            # Convert angle to quaternion (rotation around Z axis only)
            # q = [0, 0, sin(θ/2), cos(θ/2)]
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(angle / 2.0)
            t.transform.rotation.w = math.cos(angle / 2.0)
            
            transforms.append(t)
            
            self.get_logger().debug(
                f'TF: {self.base_frame} -> ultrasonic_{self.sensor_names[i]} '
                f'at ({x:.1f}, {y:.1f}) cm, {math.degrees(angle):.1f}°'
            )
        
        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(f'Published {len(transforms)} sensor TF frames')

    def ultrasonic_callback(self, msg: Float32MultiArray):
        """
        Convert ultrasonic readings to PointCloud2.
        
        Each valid reading generates a point in the base_link frame
        at the detected obstacle position.
        """
        if len(msg.data) != self.num_sensors:
            self.get_logger().warn(
                f'Expected {self.num_sensors} sensors, got {len(msg.data)}'
            )
            return
        
        points = []
        
        for i, distance in enumerate(msg.data):
            # Skip invalid readings
            if distance < self.min_range or distance > self.max_range:
                continue
            
            # Skip no-echo readings
            if distance >= self.invalid_value:
                continue
            
            # Get sensor position and orientation
            sensor_x, sensor_y, angle = self.sensor_positions[i]
            
            # Convert sensor position from cm to m
            sensor_x_m = sensor_x / 100.0
            sensor_y_m = sensor_y / 100.0
            
            # Convert distance from cm to m
            distance_m = distance / 100.0
            
            # Calculate obstacle point in base_link frame
            # Obstacle is at: sensor_position + distance * sensor_direction
            obstacle_x = sensor_x_m + distance_m * math.cos(angle)
            obstacle_y = sensor_y_m + distance_m * math.sin(angle)
            obstacle_z = self.sensor_height  # Assume obstacle at sensor height
            
            points.append([obstacle_x, obstacle_y, obstacle_z])
        
        # Publish point cloud if we have points
        if points:
            self.publish_pointcloud(points)

    def publish_pointcloud(self, points: list):
        """
        Create and publish a PointCloud2 message.
        
        Args:
            points: List of [x, y, z] points in meters
        """
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = self.base_frame
        
        # Define point fields (x, y, z as 32-bit floats)
        cloud_msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]
        
        # Point cloud structure
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats × 4 bytes
        cloud_msg.height = 1       # Unordered point cloud
        cloud_msg.width = len(points)
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.is_dense = True  # No invalid points
        
        # Pack point data
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        
        cloud_msg.data = b''.join(buffer)
        
        self.pointcloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = CostmapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Costmap node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
