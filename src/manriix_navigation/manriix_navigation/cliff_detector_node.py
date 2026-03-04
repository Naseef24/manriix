#!/usr/bin/env python3
"""
Manriix Cliff Detector Node
Uses ZED X front camera depth image to detect cliffs/drops/stairs
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Bool, Header
from cv_bridge import CvBridge
import numpy as np
import struct
from collections import deque


class CliffDetector(Node):
    def __init__(self):
        super().__init__('cliff_detector')
        
        # Parameters
        self.declare_parameter('depth_topic', '/zedx_front/zed_node/depth/depth_registered')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cliff_threshold', 0.40)
        self.declare_parameter('floor_distance', 2.65)
        self.declare_parameter('max_valid_depth', 8.0)
        self.declare_parameter('roi_top_ratio', 0.65)
        self.declare_parameter('roi_bottom_ratio', 0.95)
        self.declare_parameter('roi_left_ratio', 0.1)
        self.declare_parameter('roi_right_ratio', 0.9)
        self.declare_parameter('min_valid_points', 100)
        self.declare_parameter('cliff_point_ratio', 0.3)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('no_data_is_cliff', True)
        self.declare_parameter('virtual_obstacle_distance', 0.35)
        self.declare_parameter('virtual_obstacle_width', 0.6)
        self.declare_parameter('virtual_obstacle_height', 0.3)
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cliff_threshold = self.get_parameter('cliff_threshold').value
        self.floor_distance = self.get_parameter('floor_distance').value
        self.max_valid_depth = self.get_parameter('max_valid_depth').value
        self.roi_top = self.get_parameter('roi_top_ratio').value
        self.roi_bottom = self.get_parameter('roi_bottom_ratio').value
        self.roi_left = self.get_parameter('roi_left_ratio').value
        self.roi_right = self.get_parameter('roi_right_ratio').value
        self.min_valid_points = self.get_parameter('min_valid_points').value
        self.cliff_point_ratio = self.get_parameter('cliff_point_ratio').value
        self.smoothing_window = self.get_parameter('smoothing_window').value
        self.no_data_is_cliff = self.get_parameter('no_data_is_cliff').value
        self.virt_obs_dist = self.get_parameter('virtual_obstacle_distance').value
        self.virt_obs_width = self.get_parameter('virtual_obstacle_width').value
        self.virt_obs_height = self.get_parameter('virtual_obstacle_height').value
        
        self.bridge = CvBridge()
        self.cliff_history = deque(maxlen=self.smoothing_window)
        self.cliff_detected = False
        self.last_depth_time = None
        self.frames_processed = 0
        self.cliffs_detected_count = 0
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos)
        
        # Publishers
        self.cliff_detected_pub = self.create_publisher(Bool, '/cliff_detected', 10)
        self.cliff_points_pub = self.create_publisher(PointCloud2, '/cliff_points', 10)
        
        # Watchdog
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Manriix Cliff Detector Initialized')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
        self.get_logger().info(f'  Floor distance: {self.floor_distance:.2f}m')
        self.get_logger().info(f'  Cliff threshold: {self.cliff_threshold:.2f}m')
        self.get_logger().info(f'  ROI: top={self.roi_top:.0%}, bottom={self.roi_bottom:.0%}')
        self.get_logger().info('='*60)

    def depth_callback(self, msg):
        self.last_depth_time = self.get_clock().now()
        self.frames_processed += 1
        
        try:
            if msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_image = depth_image.astype(np.float32) / 1000.0
            else:
                self.get_logger().warn(f'Unexpected depth encoding: {msg.encoding}')
                return
            
            height, width = depth_image.shape
            roi_top_px = int(height * self.roi_top)
            roi_bottom_px = int(height * self.roi_bottom)
            roi_left_px = int(width * self.roi_left)
            roi_right_px = int(width * self.roi_right)
            
            roi = depth_image[roi_top_px:roi_bottom_px, roi_left_px:roi_right_px]
            cliff_detected = self.analyze_for_cliff(roi)
            self.cliff_history.append(cliff_detected)
            
            if len(self.cliff_history) >= 3:
                smoothed_cliff = sum(self.cliff_history) > len(self.cliff_history) / 2
            else:
                smoothed_cliff = cliff_detected
            
            if smoothed_cliff != self.cliff_detected:
                self.cliff_detected = smoothed_cliff
                if smoothed_cliff:
                    self.cliffs_detected_count += 1
                    self.get_logger().warn('⚠️ CLIFF DETECTED - Publishing virtual obstacle')
                else:
                    self.get_logger().info('✓ Cliff cleared')
            
            self.publish_cliff_status(msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def analyze_for_cliff(self, roi):
        valid_mask = (
            np.isfinite(roi) & 
            (roi > 0.1) & 
            (roi < self.max_valid_depth)
        )
        valid_depths = roi[valid_mask]
        valid_count = len(valid_depths)
        
        if valid_count < self.min_valid_points:
            if self.no_data_is_cliff:
                self.get_logger().debug(
                    f'Insufficient valid points ({valid_count}/{self.min_valid_points}) - treating as cliff')
                return True
            return False
        
        cliff_depth_threshold = self.floor_distance + self.cliff_threshold
        cliff_points = np.sum(valid_depths > cliff_depth_threshold)
        cliff_ratio = cliff_points / valid_count
        
        is_cliff = cliff_ratio > self.cliff_point_ratio
        
        if is_cliff:
            self.get_logger().debug(f'Cliff analysis: depth_ratio={cliff_ratio:.2%}')
        
        return is_cliff

    def publish_cliff_status(self, stamp):
        cliff_msg = Bool()
        cliff_msg.data = bool(self.cliff_detected)
        self.cliff_detected_pub.publish(cliff_msg)
        
        if self.cliff_detected:
            self.publish_virtual_obstacle(stamp)

    def publish_virtual_obstacle(self, stamp):
        points = []
        x = self.virt_obs_dist
        y_steps = np.linspace(-self.virt_obs_width/2, self.virt_obs_width/2, 15)
        z_steps = np.linspace(0.0, self.virt_obs_height, 8)
        
        for y in y_steps:
            for z in z_steps:
                points.append([x, y, z])
        
        header = Header()
        header.stamp = stamp
        header.frame_id = self.base_frame
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_data = b''.join([struct.pack('fff', p[0], p[1], p[2]) for p in points])
        
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc2.width
        pc2.data = cloud_data
        pc2.is_dense = True
        
        self.cliff_points_pub.publish(pc2)

    def watchdog_callback(self):
        if self.last_depth_time is None:
            return
        
        time_since_depth = (self.get_clock().now() - self.last_depth_time).nanoseconds / 1e9
        
        if time_since_depth > 2.0:
            self.get_logger().warn(
                f'No depth data for {time_since_depth:.1f}s - cliff detection unavailable!')


def main(args=None):
    rclpy.init(args=args)
    node = CliffDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Cliff Detector shutting down. '
            f'Processed {node.frames_processed} frames, '
            f'detected {node.cliffs_detected_count} cliff events.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
