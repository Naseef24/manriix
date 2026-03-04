#!/usr/bin/env python3
"""
Camera Image Publisher Node

Subscribes to ZED camera image topics and republishes them as compressed images
for the remote visualizer. This reduces network bandwidth while maintaining
visualization capability.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import threading
from typing import Dict


class CameraImagePublisher(Node):
    """
    Republishes ZED camera images as compressed images for remote visualization
    
    Features:
    - Subscribes to raw ZED camera topics
    - Compresses images using JPEG
    - Republishes on /viz/camera/* topics for remote access
    - Configurable compression quality and resize factors
    """
    
    def __init__(self):
        super().__init__('camera_image_publisher')
        
        # Configuration
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('resize_factor', 0.7)  # Scale down to reduce bandwidth
        self.declare_parameter('publish_rate', 15.0)  # Max publish rate (Hz)
        
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.resize_factor = self.get_parameter('resize_factor').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        # Camera configuration (ZED SDK 5.x topic naming)
        self.camera_configs = {
            'zedx_front': {
                'input_topic': '/zedx_front/zed_node/rgb/color/rect/image',
                'output_topic': '/viz/camera/zedx_front/image_rect_color/compressed'
            },
            'zedx_left': {
                'input_topic': '/zedx_left/zed_node/rgb/color/rect/image',
                'output_topic': '/viz/camera/zedx_left/image_rect_color/compressed'
            },
            'zedx_right': {
                'input_topic': '/zedx_right/zed_node/rgb/color/rect/image',
                'output_topic': '/viz/camera/zedx_right/image_rect_color/compressed'
            }
        }
        
        # Subscribers and publishers
        self.subscribers = {}
        self.publishers = {}
        
        # Rate limiting
        self.last_publish_times = {}
        self.min_publish_interval = 1.0 / self.publish_rate
        
        # Threading
        self.processing_lock = threading.Lock()
        
        # Setup publishers and subscribers
        self._setup_publishers_and_subscribers()
        
        self.get_logger().info(f"Camera Image Publisher initialized")
        self.get_logger().info(f"JPEG Quality: {self.jpeg_quality}, Resize: {self.resize_factor}, Max Rate: {self.publish_rate} Hz")
    
    def _setup_publishers_and_subscribers(self):
        """Setup publishers and subscribers for all cameras"""
        import time
        
        for camera_name, config in self.camera_configs.items():
            # Create publisher
            publisher = self.create_publisher(
                CompressedImage,
                config['output_topic'],
                self.sensor_qos
            )
            self.publishers[camera_name] = publisher
            
            # Create subscriber
            subscriber = self.create_subscription(
                Image,
                config['input_topic'],
                lambda msg, cam=camera_name: self._image_callback(msg, cam),
                self.sensor_qos
            )
            self.subscribers[camera_name] = subscriber
            
            # Initialize rate limiting
            self.last_publish_times[camera_name] = 0.0
            
            self.get_logger().info(f"Setup {camera_name}: {config['input_topic']} -> {config['output_topic']}")
    
    def _image_callback(self, msg: Image, camera_name: str):
        """Process incoming camera images and republish as compressed"""
        
        # Rate limiting check
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_publish_times[camera_name] < self.min_publish_interval:
            return
        
        try:
            with self.processing_lock:
                # Convert ROS image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # Resize image to reduce bandwidth
                if self.resize_factor != 1.0:
                    height, width = cv_image.shape[:2]
                    new_width = int(width * self.resize_factor)
                    new_height = int(height * self.resize_factor)
                    cv_image = cv2.resize(cv_image, (new_width, new_height))
                
                # Compress image
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                success, compressed_data = cv2.imencode('.jpg', cv_image, encode_params)
                
                if success:
                    # Create compressed image message
                    compressed_msg = CompressedImage()
                    compressed_msg.header = msg.header
                    compressed_msg.format = 'jpeg'
                    compressed_msg.data = compressed_data.tobytes()
                    
                    # Publish
                    self.publishers[camera_name].publish(compressed_msg)
                    self.last_publish_times[camera_name] = current_time
                    
                else:
                    self.get_logger().warning(f"Failed to compress image from {camera_name}")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image from {camera_name}: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CameraImagePublisher()
        
        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        
        try:
            rclpy.spin(node, executor=executor)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start camera image publisher: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()